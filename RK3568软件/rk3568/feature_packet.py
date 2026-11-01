"""Decode same-frame metadata and independently check candidates against RGB565."""
import array
import struct
import sys

TRAILER_BYTES = 256
RECORD = struct.Struct('<BBHHHHHHBB')
ROAD_KEYS = ('pixel_count', 'gray_sum', 'dark_pixels', 'bright_pixels',
             'white_left_roi', 'white_right_roi', 'white_zebra_roi', 'gaussian_gray_sum',
             'sobel_edges', 'dilated_edges', 'eroded_edges', 'dilated_zebra_roi',
             'red_candidates', 'green_candidates', 'blue_candidates',
             'sobel_left_roi', 'sobel_right_roi', 'sobel_zebra_roi',
             'dilated_left_roi', 'dilated_right_roi')


def image_xor32(image):
    words = array.array('I')
    words.frombytes(image)
    if sys.byteorder != 'little':
        words.byteswap()
    value = 0
    for word in words:
        value ^= word
    return value


def decode_frame(payload, width=1920, height=1080):
    image_bytes = width * height * 2
    if len(payload) != image_bytes + TRAILER_BYTES:
        raise ValueError('Incomplete image/feature payload')
    image, trailer = payload[:image_bytes], payload[image_bytes:]
    magic, source_id, w, h, version, flags, length = struct.unpack_from('<4sIHHBBH', trailer)
    if (magic, w, h, version, length) != (b'FEAT', width, height, 1, 64) or not flags & 8:
        raise ValueError('Invalid feature header or unqualified frame')
    tag, same_id, checksum, layout = struct.unpack_from('<4sIII', trailer, 64)
    if tag != b'FVID' or same_id != source_id or layout not in (0x00010100, 0x00020100):
        raise ValueError('Feature packet does not identify this video frame')
    if checksum != image_xor32(image):
        raise ValueError('Image checksum does not match feature frame')
    lux_tag, lux_version, mode, context = struct.unpack_from('<4sIII', trailer, 80)
    profile, source_kind = context & 65535, context >> 16
    if lux_tag != b'LUXF' or lux_version != 1 or mode > 3 or profile > 3 or source_kind > 1:
        raise ValueError('Invalid enhancement context')
    road = None
    if layout == 0x00020100:
        words = struct.unpack_from('<32I', trailer, 96)
        if words[:4] != (0x31584652, source_id, 1, 0x7f):
            raise ValueError('Invalid or mismatched road preprocessing metadata')
        if words[24:28] != (0x2db44060, (width//2 << 16) | (height*2//3),
                            (height << 16) | (height//3), (height << 16) | width):
            raise ValueError('Unexpected road preprocessing settings/ROI')
        if words[4] != width*height or words[28:32] != ((width-2)*(height-2),
                                                       (width-4)*(height-4), 0, 0):
            raise ValueError('Incomplete road preprocessing frame or invalid padding')
        road = dict(zip(ROAD_KEYS, words[4:24]))
    if any(trailer[224 if road is not None else 96:]):
        raise ValueError('Unexpected trailer padding')
    keys = ('type', 'valid', 'x_min', 'y_min', 'x_max', 'y_max',
            'center_x', 'center_y', 'row_count', 'reserved_score')
    records = [dict(zip(keys, RECORD.unpack_from(trailer, 16 + 16*i))) for i in range(3)]
    for i, record in enumerate(records):
        if record['type'] != i + 1 or record['valid'] != bool(flags & (1 << i)):
            raise ValueError('Feature header/record flags disagree')
        if record['valid'] and not (0 <= record['x_min'] <= record['x_max'] < width and
                                   0 <= record['y_min'] <= record['y_max'] < height):
            raise ValueError('Feature coordinates exceed the image')
    return dict(source_frame_id=source_id, width=width, height=height,
                low_light_mode=mode, low_light_profile=profile, source_kind=source_kind, image_xor32=checksum,
                records=records, road_stats=road)


def reference_candidates(image, width, height):
    try:
        import numpy as np
    except ImportError:
        return reference_candidates_scalar(image, width, height)
    pixels = np.frombuffer(image, dtype='<u2').reshape(height, width)
    channels = np.stack(((pixels >> 11) << 3, ((pixels >> 5) & 63) << 2,
                         (pixels & 31) << 3), axis=2).astype(np.int16)
    white = (channels.min(axis=2) >= 180) & (channels.max(axis=2)-channels.min(axis=2) <= 45)
    starts = white & ~np.pad(white[:, :-1], ((0, 0), (1, 0)))
    runs = starts.sum(axis=1)
    row = np.arange(height)
    half = width // 2
    masks = [
        (row >= height*2//3) & (white[:, :half].sum(axis=1) >= 4) & (runs <= 2),
        (row >= height*2//3) & (white[:, half:].sum(axis=1) >= 4) & (runs <= 2),
        (row >= height//3) & (white.sum(axis=1) >= 12) & (runs >= 3),
    ]
    result = []
    for i, selected in enumerate(masks):
        count = int(selected.sum())
        record = dict(type=i+1, valid=int(count >= (8 if i < 2 else 4)), row_count=min(255, count))
        if record['valid']:
            area = white.copy()
            area[~selected] = False
            if i == 0:
                area[:, half:] = False
            elif i == 1:
                area[:, :half] = False
            ys, xs = np.nonzero(area)
            x0, x1, y0, y1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
            record.update(x_min=x0, x_max=x1, y_min=y0, y_max=y1,
                          center_x=(x0+x1)//2, center_y=(y0+y1)//2)
        result.append(record)
    return result


def reference_candidates_scalar(image, width, height):
    pixels = array.array('H')
    pixels.frombytes(image)
    if sys.byteorder != 'little':
        pixels.byteswap()
    # Batch rows into bounding boxes; this uses neither RTL state nor packets.
    accumulators = [[0, width, 0, height, 0] for _ in range(3)]
    half = width//2
    for y in range(height//3, height):
        left_count = right_count = count = runs = 0
        left_min = right_min = width
        left_max = right_max = 0
        previous = False
        for x in range(width):
            pixel = pixels[y*width+x]
            r, g, b = (pixel >> 11) << 3, ((pixel >> 5) & 63) << 2, (pixel & 31) << 3
            white = r >= 180 and g >= 180 and b >= 180 and max(r,g,b)-min(r,g,b) <= 45
            if white:
                count += 1
                runs += not previous
                if x < half:
                    left_count += 1
                    left_min = min(left_min, x)
                    left_max = x
                else:
                    right_count += 1
                    right_min = min(right_min, x)
                    right_max = x
            previous = white
        choices = (
            (y >= height*2//3 and left_count >= 4 and runs <= 2, left_min, left_max),
            (y >= height*2//3 and right_count >= 4 and runs <= 2, right_min, right_max),
            (count >= 12 and runs >= 3, min(left_min,right_min), max(left_max,right_max)),
        )
        for i, (eligible, x0, x1) in enumerate(choices):
            if eligible:
                a = accumulators[i]
                a[0] += 1
                a[1], a[2], a[3], a[4] = min(a[1],x0), max(a[2],x1), min(a[3],y), y
    result = []
    for i, (count,x0,x1,y0,y1) in enumerate(accumulators):
        record = dict(type=i+1, valid=int(count >= (8 if i < 2 else 4)), row_count=min(255,count))
        if record['valid']:
            record.update(x_min=x0,x_max=x1,y_min=y0,y_max=y1,
                          center_x=(x0+x1)//2,center_y=(y0+y1)//2)
        result.append(record)
    return result


def verify_candidates(payload, width=1920, height=1080):
    decoded = decode_frame(payload, width, height)
    expected = reference_candidates(payload[:width*height*2], width, height)
    for got, reference in zip(decoded['records'], expected):
        for key, value in reference.items():
            if got[key] != value:
                raise ValueError(f"Frame {decoded['source_frame_id']} type {got['type']} "
                                 f'{key}: FPGA={got[key]} reference={value}')
    decoded['reference_matched'] = True
    if decoded['road_stats'] is not None:
        reference = reference_road_stats(payload[:width*height*2], width, height)
        if decoded['road_stats'] != reference:
            differences = {key: (decoded['road_stats'][key], value) for key, value in reference.items()
                           if decoded['road_stats'][key] != value}
            raise ValueError(f'Road preprocessing mismatch: {differences}')
        decoded['road_reference_matched'] = True
    return decoded


def reference_road_stats(image, width, height):
    import numpy as np
    pixels = np.frombuffer(image, dtype='<u2').reshape(height, width)
    r = ((pixels >> 11) << 3).astype(np.int32)
    g = (((pixels >> 5) & 63) << 2).astype(np.int32)
    b = ((pixels & 31) << 3).astype(np.int32)
    gray = (r >> 2) + (g >> 1) + (b >> 2)
    channels = np.stack((r, g, b), axis=2)
    white = (channels.min(axis=2) >= 180) & (channels.max(axis=2)-channels.min(axis=2) <= 45)
    a = [gray[y:y+height-2, x:x+width-2] for y in range(3) for x in range(3)]
    gx = a[2]+2*a[5]+a[8]-a[0]-2*a[3]-a[6]
    gy = a[6]+2*a[7]+a[8]-a[0]-2*a[1]-a[2]
    edges = np.abs(gx)+np.abs(gy) >= 96
    gaussian = (a[0]+2*a[1]+a[2]+2*a[3]+4*a[4]+2*a[5]+a[6]+2*a[7]+a[8]) >> 4
    e = [edges[y:y+height-4, x:x+width-4] for y in range(3) for x in range(3)]
    dilation, erosion = np.logical_or.reduce(e), np.logical_and.reduce(e)
    lane_y, zebra_y, half = height*2//3, height//3, width//2
    values = (width*height, int(gray.sum()), int((gray<64).sum()), int((gray>=180).sum()),
              int(white[lane_y:, :half].sum()), int(white[lane_y:, half:].sum()),
              int(white[zebra_y:].sum()), int(gaussian.sum()), int(edges.sum()),
              int(dilation.sum()), int(erosion.sum()), int(dilation[max(0,zebra_y-2):].sum()),
              int(((r>=160) & (r-g>=48) & (r-b>=48)).sum()),
              int(((g>=160) & (g-r>=48) & (g-b>=48)).sum()),
              int(((b>=160) & (b-r>=48) & (b-g>=48)).sum()),
              int(edges[max(0,lane_y-1):, :max(0,half-1)].sum()),
              int(edges[max(0,lane_y-1):, max(0,half-1):].sum()),
              int(edges[max(0,zebra_y-1):].sum()),
              int(dilation[max(0,lane_y-2):, :max(0,half-2)].sum()),
              int(dilation[max(0,lane_y-2):, max(0,half-2):].sum()))
    return dict(zip(ROAD_KEYS, values))

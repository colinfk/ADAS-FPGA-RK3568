#!/usr/bin/env python3
"""Read and decode the FPGA VID1 diagnostic block at BAR0 + 0xfff0."""

import argparse
import json
import mmap
import os
import struct
from pathlib import Path

BAR_BYTES = 0x10000
STATUS_OFFSET = 0xFFF0
FLAGS = (
    "ddr_init", "hdmi_seen", "fifo_overflow", "writer_error",
    "frame_ready", "reader_busy", "reader_done", "reader_error",
)
DDR_INIT_DIAG = (
    ("gpll_lock", 0), ("phy_pll_lock", 1), ("pll_lock", 2),
    ("rst_gpll_lock", 3), ("ddrphy_cpd_lock", 4),
    ("dbg_ddrphy_init_fail", 5), ("align_error", 6), ("resetn", 7),
    ("core_clk_rst_n", 8), ("heart_beat_led", 9),
)
RX_V2_FLAGS = (
    ("ownership_drop", 18), ("writer_fault", 19),
    ("rx_init_done", 20), ("pixclk_seen", 21),
    ("vs_seen", 22), ("de_seen", 23),
)
RX_V3_FLAGS = (
    ("i2c_error_seen", 24), ("video_live", 25),
    ("pixclk_live", 26), ("geometry_error_seen", 27),
)


def decode(raw):
    return decode_block(raw, b"VID1")


def decode_block(raw, expected_magic):
    if len(raw) != 16:
        raise RuntimeError(f"Expected 16 diagnostic bytes, received {len(raw)}")
    # RTL stores an INTEGER tag (e.g. 32'h56494431), not the byte string
    # b'VID1'. On little-endian RK this appears as b'1DIV'. Keep payload
    # DWORDs little-endian; changing every byte to spell the tag corrupts
    # flags, sequence numbers and geometry. Alternative layouts are useful
    # for exported diagnostic dumps, but native MMIO is checked first.
    tag = int.from_bytes(expected_magic, "big")
    for reverse_words in (False, True):
        words = [raw[i:i + 4] for i in range(0, 16, 4)]
        if reverse_words:
            words.reverse()
        for reverse_bytes in (False, True):
            candidate = b"".join(w[::-1] if reverse_bytes else w for w in words)
            magic, pcie_seq, flags, ddr_seq = struct.unpack("<IIII", candidate)
            if magic == tag:
                return pcie_seq, flags, ddr_seq
    raise RuntimeError(f"{expected_magic.decode()} status not found: {raw.hex()}")


def format_status(status):
    pcie_seq, flags, ddr_seq = status
    prefix = f"pcie_seq={pcie_seq} ddr_seq={ddr_seq}"
    version = (flags >> 28) & 0xF
    if version == 0:
        states = " ".join(f"{name}={(flags >> bit) & 1}"
                          for bit, name in enumerate(FLAGS))
        return f"{prefix} {states}"
    if version not in (1, 2, 3, 4, 5):
        return f"{prefix} version=unknown({version}) flags=0x{flags:08x}"

    states = " ".join(f"{name}={(flags >> bit) & 1}"
                      for bit, name in enumerate(FLAGS))
    diag = (flags >> 8) & 0x3FF
    diag_states = " ".join(f"{name}={(diag >> bit) & 1}"
                           for name, bit in DDR_INIT_DIAG)
    extra_flags = RX_V2_FLAGS if version >= 2 else ()
    if version >= 3:
        extra_flags += RX_V3_FLAGS
    extra = " ".join(f"{name}={(flags >> bit) & 1}" for name, bit in extra_flags)
    result = (f"{prefix} version={version} flags=0x{flags:08x} {states} "
              f"ddr_diag=0x{diag:03x} {diag_states}")
    return f"{result} {extra}" if extra else result


def format_rx(rx, geometry):
    error_info, frequency, control = rx
    picture, recoveries, frames = geometry
    return (
        f"i2c_errors={error_info >> 16} last_error_addr=0x{error_info & 0xffff:04x} "
        f"status_209c_209f=0x{frequency:08x} "
        f"reg1281=0x{control & 0xff:02x} pclk_inv={(control >> 2) & 1} "
        f"controller_state={(control >> 8) & 0x1f} "
        f"recoveries={(control >> 13) & 0xffff} "
        f"last_line_pixels={(picture >> 8) & 0xfff} "
        f"last_frame_lines={(picture >> 20) & 0xfff} "
        f"pixel_signature=0x{picture & 0xff:02x} vs_count={frames} "
        f"geometry_recoveries={recoveries & 0xffff} hs_seen={(recoveries >> 16) & 1}"
    )


def check_resources(bdf):
    device = Path(f"/sys/bus/pci/devices/{bdf}")
    rows = (device / "resource").read_text().splitlines()
    bases = []
    for index, required in ((0, BAR_BYTES), (1, 0x1E4)):
        start, end, flags = (int(x, 16) for x in rows[index].split())
        size = end - start + 1 if start else 0
        if not (flags & 0x200) or size < required:
            raise RuntimeError(f"BAR{index} is unassigned or too small: "
                               f"start=0x{start:x} size={size} required={required}")
        bases.append(start)
    with (device / "config").open("rb", buffering=0) as config:
        config.seek(4)
        command = int.from_bytes(config.read(2), "little")
        config.seek(0x10)
        config_bars = struct.unpack("<II", config.read(8))
    for index, (configured, assigned) in enumerate(zip(config_bars, bases)):
        if configured & ~0xF != assigned:
            raise RuntimeError(f"BAR{index} config/resource mismatch: "
                               f"config=0x{configured:x} resource=0x{assigned:x}; inspect enumeration")
    if not (command & 2):
        raise RuntimeError("PCIe Memory Space is disabled; inspect enumeration before reading BAR")
    return device


def read_status(bdf="0002:21:00.0"):
    resource = Path(f"/sys/bus/pci/devices/{bdf}/resource0")
    fd = os.open(resource, os.O_RDONLY | os.O_SYNC)
    try:
        with mmap.mmap(fd, BAR_BYTES, flags=mmap.MAP_SHARED,
                       prot=mmap.PROT_READ) as bar:
            return decode(bar[STATUS_OFFSET:BAR_BYTES])
    finally:
        os.close(fd)


def read_rx_status(bdf):
    check_resources(bdf)
    resource = Path(f"/sys/bus/pci/devices/{bdf}/resource0")
    fd = os.open(resource, os.O_RDONLY | os.O_SYNC)
    try:
        with mmap.mmap(fd, BAR_BYTES, flags=mmap.MAP_SHARED,
                       prot=mmap.PROT_READ) as bar:
            # Independent telemetry snapshots: repeat each read to avoid
            # decoding a BAR RAM update halfway through a CPU access.
            blocks = []
            for offset, magic in ((0xFFE0, b"RXD1"), (0xFFD0, b"RXD2")):
                for _ in range(8):
                    raw = bytes(bar[offset:offset + 16])
                    if raw == bytes(bar[offset:offset + 16]):
                        blocks.append(decode_block(raw, magic))
                        break
                else:
                    raise RuntimeError(f"{magic.decode()} changed during every read; retry")
            return tuple(blocks)
    finally:
        os.close(fd)


def read_extended_status(bdf="0002:21:00.0"):
    """v4/v5 pages; every block is read twice to avoid a concurrent BAR RAM write.

    CDC payloads are coherent, but pages are published at different times.
    They are not one globally atomic source/DDR/DMA snapshot.
    """
    check_resources(bdf)
    pages = (("writer",0xffc0,b"WRT1"),("first_fault",0xffb0,b"WRF1"),
             ("capture",0xffa0,b"CAP1"),("build",0xff90,b"BLD1"),
             ("queue",0xff80,b"QST1"))
    fd = os.open(f"/sys/bus/pci/devices/{bdf}/resource0", os.O_RDONLY | os.O_SYNC)
    result = {}
    try:
        with mmap.mmap(fd, BAR_BYTES, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ) as bar:
            for name, offset, magic in pages:
                for _ in range(16):
                    raw=bytes(bar[offset:offset+16])
                    if raw==bytes(bar[offset:offset+16]):
                        result[name]=decode_block(raw,magic)
                        break
                else:
                    raise RuntimeError(f"{name} status repeatedly changed")
    finally:
        os.close(fd)
    return decode_extended(result)


def decode_extended(result):
    """Decode the little-endian DWORD payloads of the five v4 pages."""
    protocol, geometry_abort, ownership = result["writer"]
    width, height, rejected = result["capture"]
    context, expected, received = result["first_fault"]
    return dict(protocol_errors=protocol,geometry_aborts=geometry_abort,
                ownership_drops=ownership,preview_drops=result["queue"][1],
                width_errors=width,height_errors=height,rejected_frames=rejected,
                first_fault=dict(reason=context&15,epoch=(context>>4)&1,
                    eof=(context>>5)&1,sof=(context>>6)&1,
                    fill_count=(context>>12)&15,expected=expected,received=received),
                build=result["build"])


def decode_dma_status(raw):
    """v5 BAR0 0xff70: tag, completed-grant sequence, command ACK, flags."""
    frame_seq, command_seq, flags = decode_block(raw, b"DMA5")
    if flags >> 24 != 5:
        raise RuntimeError(f"Unsupported DMA5 protocol version: {flags >> 24}")
    return dict(version=5, frame_seq=frame_seq, command_seq=command_seq,
                active=bool(flags & 1), stopping=bool(flags & 2),
                source_idle=bool(flags & 4), fifo_overflow=bool(flags & 8))


def decode_hdmi_geometry(pages, cfg_hz=10000000):
    """v6 front-end feature pages; VID1/DMA5 and pixel data stay ABI version 5."""
    dims, context, frame_id = pages['geometry_fault']
    flags, vs_cfg_ticks, pixel_hz = pages['clock']
    overlap, hs_pixels, vs_pixels = pages['timing']
    version = flags >> 24
    if version not in (6, 7):
        raise RuntimeError('HDMI geometry pages require front-end feature version 6 or 7')
    reasons = {0: 'none', 1: 'line_width', 2: 'frame_height_or_incomplete_payload',
               3: 'de_during_vs', 4: 'receiver_or_mode_not_ready'}
    reason = context & 255
    result = dict(frontend_version=version, mode_valid=bool(flags & 1),
                pclk_ok=bool(flags & 2), frame_rate_ok=bool(flags & 4),
                clock_measured=bool(flags & 8), period_measured=bool(flags & 16),
                init_done=bool(flags & 32), pixel_clock_hz=pixel_hz,
                vs_period_cfg_ticks=vs_cfg_ticks,
                input_fps=cfg_hz / vs_cfg_ticks if vs_cfg_ticks and flags & 16 else None,
                vs_period_pixels=vs_pixels, hs_period_pixels=hs_pixels,
                de_vs_overlap_cycles=overlap,
                first_geometry_fault=dict(frame_id=frame_id, reason=reason,
                    description=reasons.get(reason, 'unknown'),
                    line_index=(context >> 8) & 4095,
                    de=bool(context & (1 << 20)), vs=bool(context & (1 << 21)),
                    ready=bool(context & (1 << 22)),
                    width=dims & 65535, height=dims >> 16))
    if version == 7:
        result.update(hs_period_ok=bool(flags & 64), vs_pixels_ok=bool(flags & 128),
                      sync_clean=bool(flags & 256), stable_frames=(flags >> 12) & 15)
    return result


def decode_rx_config(raw):
    flags, low, high = decode_block(raw, b'RXC1')
    if flags >> 24 != 1:
        raise RuntimeError('Unsupported receiver configuration diagnostic version')
    values = struct.pack('<II', low, high)
    addresses = (0x000a, 0x1000, 0x102d, 0x1010, 0x1024, 0x0200, 0x0215, 0x1281)
    return dict(version=1, configured=bool(flags & 1), readback_valid=bool(flags & 2),
                status_initialized=bool(flags & 4), recovery_escalated=bool(flags & 8),
                recovery_reason=(flags >> 4) & 15, output_configurations=(flags >> 8) & 255,
                full_reinitializations=(flags >> 16) & 255,
                registers={f'0x{address:04x}': f'0x{value:02x}'
                           for address, value in zip(addresses, values)})


def read_rx_config(bdf='0002:21:00.0'):
    check_resources(bdf)
    fd = os.open(f'/sys/bus/pci/devices/{bdf}/resource0', os.O_RDONLY | os.O_SYNC)
    try:
        with mmap.mmap(fd, BAR_BYTES, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ) as bar:
            for _ in range(16):
                raw = bytes(bar[0xff20:0xff30])
                if raw == bytes(bar[0xff20:0xff30]):
                    return decode_rx_config(raw)
            raise RuntimeError('RXC1 status repeatedly changed; retry')
    finally:
        os.close(fd)


def read_feature_transport(bdf='0002:21:00.0'):
    check_resources(bdf)
    fd = os.open(f'/sys/bus/pci/devices/{bdf}/resource0', os.O_RDONLY | os.O_SYNC)
    try:
        with mmap.mmap(fd, BAR_BYTES, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ) as bar:
            for _ in range(16):
                raw = bytes(bar[0xff10:0xff20])
                if raw == bytes(bar[0xff10:0xff20]):
                    flags, frames, errors = decode_block(raw, b'FAP1')
                    version = flags >> 24
                    if version not in (1, 2):
                        raise RuntimeError('Unsupported feature transport version')
                    return dict(version=version, source_kind=(flags >> 3) & 1, appended_frames=frames,
                                metadata_errors=errors, state=flags & 3)
            raise RuntimeError('FAP1 changed repeatedly; retry')
    finally:
        os.close(fd)


def read_hdmi_geometry(bdf='0002:21:00.0'):
    check_resources(bdf)
    pages = (('geometry_fault', 0xff60, b'GEO1'),
             ('clock', 0xff50, b'CLK1'), ('timing', 0xff40, b'TIM1'))
    fd = os.open(f'/sys/bus/pci/devices/{bdf}/resource0', os.O_RDONLY | os.O_SYNC)
    result = {}
    try:
        with mmap.mmap(fd, BAR_BYTES, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ) as bar:
            for name, offset, magic in pages:
                for _ in range(16):
                    raw = bytes(bar[offset:offset+16])
                    if raw == bytes(bar[offset:offset+16]):
                        result[name] = decode_block(raw, magic)
                        break
                else:
                    raise RuntimeError(f'{name} status repeatedly changed')
    finally:
        os.close(fd)
    return decode_hdmi_geometry(result)


def decode_low_light(raw):
    flags, accepted, rejected = decode_block(raw, b'LUX1')
    if flags >> 24 not in (1, 2, 3, 4):
        raise RuntimeError('Unsupported LUX1 revision')
    return dict(low_light_version=flags >> 24, grid_x=(flags >> 20) & 15,
                grid_y=(flags >> 16) & 15, center_strength=(flags >> 7) & 511,
                map_valid=bool(flags & 64), mode=(flags >> 4) & 3,
                output_ready=bool(flags & 8), accepted_frames=accepted,
                rejected_frames=rejected)


def read_low_light_status(bdf='0002:21:00.0'):
    check_resources(bdf)
    fd = os.open(f'/sys/bus/pci/devices/{bdf}/resource0', os.O_RDONLY | os.O_SYNC)
    try:
        with mmap.mmap(fd, BAR_BYTES, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ) as bar:
            for _ in range(16):
                raw = bytes(bar[0xff30:0xff40])
                if raw == bytes(bar[0xff30:0xff40]):
                    return decode_low_light(raw)
            raise RuntimeError('LUX1 status repeatedly changed; retry')
    finally:
        os.close(fd)


def read_dma_status(bdf="0002:21:00.0"):
    check_resources(bdf)
    fd = os.open(f"/sys/bus/pci/devices/{bdf}/resource0", os.O_RDONLY | os.O_SYNC)
    try:
        with mmap.mmap(fd, BAR_BYTES, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ) as bar:
            for _ in range(16):
                raw = bytes(bar[0xff70:0xff80])
                if raw == bytes(bar[0xff70:0xff80]):
                    return decode_dma_status(raw)
            raise RuntimeError("DMA5 control page repeatedly changed; retry")
    finally:
        os.close(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bdf", nargs="?", default="0002:21:00.0")
    parser.add_argument("--rx", action="store_true", help="Read RXD1/RXD2 from the new receiver build")
    parser.add_argument("--check-bars", action="store_true", help="Validate resource ranges and Memory Space first")
    parser.add_argument("--extended", action="store_true", help="Read v4/v5 error counters, queue and build stamp")
    parser.add_argument("--dma", action="store_true", help="Read v5 command ACK and safe-stop state")
    parser.add_argument('--hdmi-geometry', action='store_true',
                        help='Read v6 PCLK, input frame rate, periods and first geometry fault')
    parser.add_argument('--low-light', action='store_true', help='Read regional tone-map LUX1 status')
    parser.add_argument('--rx-config', action='store_true', help='Read MS7200 configuration readback and recovery counters')
    parser.add_argument('--feature-transport', action='store_true', help='Read FAP1 appended same-frame feature transport')
    parser.add_argument('--require-low-light-version', type=int,
                        help='Fail startup if the FPGA enhancement profile differs from this version')
    args = parser.parse_args()
    if args.check_bars:
        check_resources(args.bdf)
    print(format_status(read_status(args.bdf)))
    if args.rx:
        print(format_rx(*read_rx_status(args.bdf)))
    if args.extended:
        if read_status(args.bdf)[1] >> 28 not in (4, 5):
            raise RuntimeError("Extended pages require VID1 version 4 or 5")
        print(json.dumps(read_extended_status(args.bdf), sort_keys=True))
    if args.dma:
        if read_status(args.bdf)[1] >> 28 != 5:
            raise RuntimeError("DMA5 control page requires VID1 version 5")
        print(json.dumps(read_dma_status(args.bdf), sort_keys=True))
    if args.hdmi_geometry:
        print(json.dumps(read_hdmi_geometry(args.bdf), sort_keys=True))
    if args.rx_config:
        print(json.dumps(read_rx_config(args.bdf), sort_keys=True))
    if args.feature_transport:
        print(json.dumps(read_feature_transport(args.bdf), sort_keys=True))
    if args.low_light or args.require_low_light_version is not None:
        low_light = read_low_light_status(args.bdf)
        print(json.dumps(low_light, sort_keys=True))
        if (args.require_low_light_version is not None and
                low_light['low_light_version'] != args.require_low_light_version):
            raise RuntimeError('FPGA LUX1 version does not match the installed startup profile; check Flash image')


if __name__ == "__main__":
    assert decode(struct.pack("<IIII", 0x56494431, 2, 3, 4)) == (2, 3, 4)
    assert format_status((2, 0x55, 4)) == (
        "pcie_seq=2 ddr_seq=4 ddr_init=1 hdmi_seen=0 fifo_overflow=1 "
        "writer_error=0 frame_ready=1 reader_busy=0 reader_done=1 reader_error=0"
    )
    main()

"""Generate a preferred RGB 1080p30 HDMI EDID and MS7200 EDID RAM writes.

CTA VIC 34: 74250 kHz, 2200x1125 total, 1920x1080 active, 30/29.97 Hz.
The register order and all non-EDID receiver settings remain vendor-derived.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re

ROOT = Path(__file__).resolve().parent


def build_edid():
    data = bytearray(256)
    data[:20] = bytes.fromhex('00 ff ff ff ff ff ff 00 4c 2d ff 0d 58 4d 51 30 1c 1c 01 03')
    # HDMI requires EDID 1.3. Digital RGB, preferred timing; feature bit 0
    # (default GTF support) stays clear. CTA declares RGB only/no deep color.
    data[20:25] = bytes([0x80, 0, 0, 0x78, 0x0a])
    data[25:35] = bytes.fromhex('5f b1 a2 57 4f a2 28 0f 50 54')
    data[35:38] = b'\x20\0\0'  # HDMI-mandatory 640x480p60 compatibility
    data[38:54] = b'\x01\x01' * 8  # no standard 60 Hz timings
    # Pixel clock in 10 kHz units, separate positive H/V sync.
    data[54:72] = bytes.fromhex('01 1d 80 18 71 38 2d 40 58 2c 45 00 00 00 00 00 00 1e')
    data[72:90] = b'\0\0\0\xfc\0' + b'ADAS1080P30\n '
    data[90:108] = b'\0\0\0\xfe\0' + b'RGB 1080p30\n '
    # EDID 1.3 range descriptor: code 0 is the compatible default. The GTF
    # support feature is clear; no standard timings/additional DTDs are listed.
    data[108:126] = bytes.fromhex('00 00 00 fd 00 1d 3d 1f 22 08 00 0a 20 20 20 20 20 20')
    data[126] = 1
    # CTA-861 extension, RGB only, no audio/deep color/YUV/4K capabilities.
    # HDMI VSDB OUI 000c03, physical address 0.0.0.0, TMDS max 75 MHz.
    # VIC 1 is mandatory for HDMI conformity. It never passes the capture
    # timing gate. VCDB advertises underscan/selectable RGB; CDB includes sRGB.
    blocks = bytes.fromhex('42 a2 01 67 03 0c 00 00 00 00 0f e2 00 4a e3 05 00 20')
    data[128:132] = bytes([2, 3, 4 + len(blocks), 0x81])
    data[132:132+len(blocks)] = blocks
    for start in (0, 128):
        data[start+127] = (-sum(data[start:start+127])) & 255
    assert len(data) == 256
    return bytes(data)


def embedded_edid(source):
    writes = re.findall(r"cmd_data\s*=\s*\{16'h([0-9a-f]+),\s*8'h([0-9a-f]+)\}", source, re.I)
    ram = {int(a, 16): int(b, 16) for a, b in writes if 0x0d00 <= int(a, 16) <= 0x0dff}
    return bytes(ram[a] for a in range(0x0d00, 0x0e00))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args()
    data = build_edid()
    path = ROOT / 'ms7200_ctl.v'
    source = path.read_text(encoding='utf-8')
    if args.write:
        def replace(match):
            addr = int(match[2], 16)
            return match[1] + match[2] + match[3] + f'{data[addr-0x0d00]:02X}' + match[5]
        source = re.sub(r"(cmd_data\s*=\s*\{16'h)(0[Dd][0-9A-Fa-f]{2})(,\s*8'h)([0-9A-Fa-f]{2})(\})", replace, source)
        path.write_text(source, encoding='utf-8', newline='\n')
        (ROOT / 'edid_1080p30.hex').write_text('\n'.join(data[i:i+16].hex(' ') for i in range(0, 256, 16))+'\n')
        (ROOT / 'edid_1080p30.json').write_text(json.dumps(dict(
            active=[1920,1080], total=[2200,1125], pixel_clock_hz=74250000,
            refresh_hz=30, fractional_refresh_supported=True, cta_vics=[34,1],
            capture_vics=[34], compatibility_mode='640x480p60, rejected by fixed 1080p capture',
            color='RGB 8 bits/channel', hdmi_max_tmds_mhz=75,
            sha256=hashlib.sha256(data).hexdigest(),
            block_checksums=[sum(data[i:i+128])%256 for i in (0,128)]), indent=2)+'\n')
    if embedded_edid(source) != data:
        raise SystemExit('Embedded EDID differs; regenerate with --write')
    print('PASS EDID: 256 bytes, two valid checksums, preferred 1080p30, mandatory VGA compatibility')


if __name__ == '__main__':
    main()

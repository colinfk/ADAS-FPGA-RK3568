#!/usr/bin/env python3
"""Inspect ADAS enumeration, or prepare an unbound endpoint for the driver.

The default is read-only. --repair only rescans an unbound, non-bus-mastering
endpoint with stale resources. BAR bases are restored by the driver from PCI
core resources. No hard-coded BDF/address, sizing probes, resets or DMA starts.
"""
import argparse
import json
import os
from pathlib import Path
import struct
import sys
import time

PCI = Path('/sys/bus/pci/devices')
EXPECTED = (65536, 512)


def endpoints():
    result = []
    for dev in sorted(PCI.iterdir()):
        try:
            if ((dev / 'vendor').read_text().strip(), (dev / 'device').read_text().strip()) == ('0x0755', '0x0755'):
                result.append(dev)
        except FileNotFoundError:
            continue
    if len(result) > 1:
        raise RuntimeError('multiple 0755:0755 endpoints; automatic selection refused')
    return result


def snapshot(dev):
    with (dev / 'config').open('rb', buffering=0) as config:
        raw = config.read(64)
    if len(raw) != 64 or struct.unpack_from('<I', raw)[0] != 0x07550755:
        raise RuntimeError('configuration space inaccessible or identity changed')
    resources = [tuple(int(field, 16) for field in line.split())
                 for line in (dev / 'resource').read_text().splitlines()[:6]]
    bars = struct.unpack_from('<6I', raw, 16)
    command = struct.unpack_from('<H', raw, 4)[0]
    driver = (dev / 'driver').resolve().name if (dev / 'driver').exists() else None
    valid = len(resources) == 6
    for i, (start, end, flags) in enumerate(resources):
        if i < 2:
            valid &= (start > 0 and end - start + 1 == EXPECTED[i] and
                      (flags & 0x200) != 0 and (flags & (0x100000 | 0x2000 | 0x20000000)) == 0)
            valid &= (bars[i] & 0xF) == 0
        else:
            valid &= (start == 0 and end == 0 and bars[i] == 0)
    return dict(bdf=dev.name, driver=driver, command=command, bars=list(bars),
                resources=resources, resource_contract_ok=bool(valid),
                perst_gpio_configured=any(
                    (parent / 'of_node/reset-gpios').exists() for parent in dev.resolve().parents))


def prepare(repair=False, wait_seconds=20):
    deadline = time.monotonic() + wait_seconds
    rescans = 0
    events = []
    while True:
        found = endpoints()
        if not found:
            if repair and rescans == 0:
                # No endpoint exists yet. Ask PCI core to rediscover it once.
                Path('/sys/bus/pci/rescan').write_text('1\n')
                rescans += 1
                events.append('rescan_absent_endpoint')
            if not repair or time.monotonic() >= deadline:
                raise RuntimeError('endpoint missing; power/configure FPGA before starting service')
            time.sleep(0.5)
            continue
        dev = found[0]
        state = snapshot(dev)
        if state['resource_contract_ok']:
            if state['driver'] not in (None, 'adas_video'):
                raise RuntimeError('endpoint owned by another driver')
            if state['driver'] is None and state['command'] & 4:
                raise RuntimeError('unbound endpoint still has bus mastering enabled; stop/reset DMA first')
            state['events'] = events
            return state
        if state['driver']:
            raise RuntimeError('stale resources with a bound driver; stop playback and unload driver first')
        if not repair:
            state['events'] = events
            return state
        if state['command'] & 4:
            raise RuntimeError('refusing removal of an endpoint with bus mastering enabled')
        if rescans >= 2 or time.monotonic() >= deadline:
            raise RuntimeError('BAR sizes/types still differ from 64KiB/512B, BAR2-5 disabled; '
                               'Flash may contain an older FPGA image. JTAG is volatile; program matching Flash')
        # Recheck identity and ownership after allowing configuration to settle.
        time.sleep(1)
        again = snapshot(dev)
        if again['driver'] or again['command'] & 4:
            raise RuntimeError('ownership changed; refusing rescan')
        if again['resource_contract_ok']:
            continue
        bridge = dev.resolve().parent
        scoped_rescan = bridge / 'rescan'
        if not scoped_rescan.exists():
            raise RuntimeError('parent bridge rescan unavailable; automatic removal refused')
        (dev / 'remove').write_text('1\n')
        scoped_rescan.write_text('1\n')
        rescans += 1
        events.append('remove_rescan_stale_resources')
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repair', action='store_true')
    parser.add_argument('--wait-seconds', type=float, default=20)
    args = parser.parse_args()
    if not 0 <= args.wait_seconds <= 60:
        parser.error('--wait-seconds must be between 0 and 60')
    lock = None
    try:
        if args.repair:
            import fcntl
            if os.geteuid() != 0:
                raise RuntimeError('--repair requires root')
            lock = open('/run/adas-pcie-prepare.lock', 'w')
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        state = prepare(args.repair, args.wait_seconds)
        print(json.dumps(state, sort_keys=True), flush=True)
        return 0 if state['resource_contract_ok'] else 2
    except (OSError, RuntimeError) as exc:
        print(json.dumps({'error': str(exc)}), file=sys.stderr, flush=True)
        return 1
    finally:
        if lock:
            lock.close()


if __name__ == '__main__':
    sys.exit(main())

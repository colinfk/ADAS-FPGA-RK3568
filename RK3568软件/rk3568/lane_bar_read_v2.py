#!/usr/bin/env python3
"""Read-only PCI sysfs BAR0 reader. No DMA or device control writes."""
import argparse
import json
import mmap
import struct
import time

NAMES=('NONE','DASHED','SOLID','INVALID')
BASE=0xfec0

def read_snapshot(read32, attempts=50):
    for _ in range(attempts):
        first=read32(BASE+8)
        if first&1:
            continue
        words=[read32(BASE+i*4) for i in range(12)]
        last=read32(BASE+8)
        if first!=last or first!=words[2] or first!=words[10] or last&1:
            continue
        if words[0]!=0x314e414c or words[1]&0xffff!=1:
            raise ValueError('LAN1 ABI not present; load a CNN-enabled image first')
        classes=words[5]
        return dict(sequence=last,model_version=words[1]>>16,flags=words[3],frame_id=words[4],
                    pipeline_ready=bool(words[3]&1),input_fault=bool(words[3]&4),
                    drop_seen=bool(words[3]&8),
                    left_class=NAMES[classes&3],right_class=NAMES[(classes>>2)&3],
                    left_valid=bool(classes&16),right_valid=bool(classes&32),
                    left_confidence=words[6]&65535,right_confidence=words[6]>>16,
                    roi_config_version=words[7],left_margin=words[8],right_margin=words[9],
                    drop_count=words[11])
    raise TimeoutError('No coherent BAR snapshot; writer busy or reset')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('resource0',help='/sys/bus/pci/devices/<BDF>/resource0')
    p.add_argument('--watch',type=float,default=0,help='poll interval in seconds; 0 reads once')
    a=p.parse_args()
    if a.watch<0:p.error('--watch must be nonnegative')
    with open(a.resource0,'rb',buffering=0) as f:
        with mmap.mmap(f.fileno(),65536,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ) as bar:
            previous=None
            while True:
                result=read_snapshot(lambda offset:struct.unpack_from('<I',bar,offset)[0])
                if result['sequence']!=previous:
                    print(json.dumps(result,ensure_ascii=False),flush=True)
                    previous=result['sequence']
                if not a.watch:break
                time.sleep(a.watch)

if __name__=='__main__':main()

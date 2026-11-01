"""Rasterize local system-font glyphs once; board runtime needs no font libraries."""
import argparse
from pathlib import Path
import re
from PIL import Image,ImageDraw,ImageFont
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--font',default='C:/Windows/Fonts/msyh.ttc')
parser.add_argument('--cruise-font',default='C:/Windows/Fonts/NotoSansSC-VF.ttf')
args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
source=(root/'adas_hud.c').read_text(encoding='utf-8')
chars=set(chr(i) for i in range(32,127))
chars.update(c for s in re.findall(r'"([^"\n]*)"',source) for c in s if ord(c)>127)
font=ImageFont.truetype(args.font,28)
width,height=32,38
lines=['/* Generated glyph raster for this HUD. Regenerate with build_hud_font.py. */',
       '#ifndef HUD_FONT_DATA_H','#define HUD_FONT_DATA_H','#include <stdint.h>',
       '#define HUD_FONT_W 32','#define HUD_FONT_H 38',
       'struct hud_glyph { uint32_t code; uint8_t advance; uint8_t alpha[HUD_FONT_W*HUD_FONT_H]; };',
       'static const struct hud_glyph hud_glyphs[]={']
for char in sorted(chars,key=ord):
    im=Image.new('L',(width,height));ImageDraw.Draw(im).text((0,-1),char,font=font,fill=255)
    advance=round(font.getlength(char))+1
    assert 1<=advance<=width,(char,advance)
    lines.append('{'+str(ord(char))+','+str(advance)+',{'+','.join(map(str,im.tobytes()))+'}},')
lines.extend(['};','#endif'])
(root/'hud_font.h').write_text('\n'.join(lines)+'\n',encoding='utf-8')
print(f'Generated {len(chars)} glyphs, {len(chars)*width*height} alpha bytes')

# Cruise uses native pixel sizes; existing rasterization above remains unchanged.
font_path=Path(args.cruise_font)
if not font_path.exists():
    font_path=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
if not font_path.exists():raise FileNotFoundError('Noto CJK font required for cruise glyph export')
cruise=['/* Generated native-size Noto CJK alpha glyphs. */','#ifndef CRUISE_FONT_H',
        '#define CRUISE_FONT_H',
        'struct cruise_glyph { uint32_t code,offset; unsigned char advance; };',
        'struct cruise_font { int size,w,h,count; const struct cruise_glyph *glyphs; const unsigned char *alpha; };']
for size in (14,16,18,22,32):
    face=ImageFont.truetype(str(font_path),size)
    try:face.set_variation_by_axes([700 if size in (14,32) else 400])
    except (OSError,ValueError):pass
    w,h=size+4,size+6
    top=face.getbbox('巡')[1]
    data=bytearray();entries=[]
    for char in sorted(chars,key=ord):
        im=Image.new('L',(w,h));ImageDraw.Draw(im).text((0,-top),char,font=face,fill=255)
        entries.append('{'+f'{ord(char)},{len(data)},{round(face.getlength(char))}'+'}')
        data.extend(im.tobytes())
    cruise.append(f'static const unsigned char cruise_alpha_{size}[]={{')
    cruise.extend(','.join(map(str,data[k:k+128]))+',' for k in range(0,len(data),128))
    cruise.extend(['};',f'static const struct cruise_glyph cruise_glyphs_{size}[]={{',','.join(entries),'};'])
cruise.append('static const struct cruise_font cruise_fonts[]={')
for size in (14,16,18,22,32):
    cruise.append('{'+f'{size},{size+4},{size+6},{len(chars)},cruise_glyphs_{size},cruise_alpha_{size}'+'},')
cruise.extend(['};','#endif'])
(root/'cruise_font.h').write_text('\n'.join(cruise)+'\n',encoding='utf-8')
print('Generated native cruise fonts: 14/16/18/22/32 px from',font_path)

#include "key_force_drive.h"
void key_drive_reset(KeyDrive *s) {
 s->initialized=0;s->last_ms=0;s->last_red_ms=-1000000;s->red_context=0;
 s->armed=0;s->motion=0;s->route=0;s->manual=0;s->violation=0;
 s->alarm=0;s->event_count=0;s->last_command=0;s->signal_red=0;
}
void key_drive_step(KeyDrive *s,int t,int light,int confirmed,int ego,int command) {
 if(t<0 || light<0 || light>3 || ego<0 || ego>2 || command<0 || command>4) {
  s->alarm=0;return;
 }
 if(s->initialized && t<s->last_ms)key_drive_reset(s);
 s->initialized=1;s->last_ms=t;
 int red=confirmed && light==1;
 s->signal_red=red;
 if(red){s->last_red_ms=t;s->red_context=1;}
 else if((confirmed && (light==2 || light==3)) || t-s->last_red_ms>1000) {
  s->red_context=0;s->armed=0;s->violation=0;
 }
 /* Recorded motion only seeds the initial state. It can never override a key. */
 if(!s->manual && ego)s->motion=ego;
 if(red && s->motion==1)s->armed=1;
 if(command==1) {
  s->manual=1;s->motion=1;s->violation=0;
  if(red)s->armed=1;
 } else if(command>=2) {
  s->manual=1;s->motion=2;s->route=command-3;
  if(s->red_context && s->armed && !s->violation) {
   s->violation=1;
   if(red)s->event_count++;
  }
 }
 s->last_command=command;
 s->alarm=red && s->armed && s->violation && s->motion==2;
}

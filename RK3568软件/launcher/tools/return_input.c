/* Input-only X11 surface: any left click/touch returns from C processing. */
#include "adas_hud.h"
#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <X11/extensions/XInput2.h>
#include <stdio.h>

static int x_error;
static int handle_error(Display *display,XErrorEvent *event) {
    char text[128];XGetErrorText(display,event->error_code,text,sizeof(text));
    fprintf(stderr,"Return input X11 error: %s (request %u)\n",text,event->request_code);
    x_error=1; return 0;
}
int main(void) {
    Display *display=XOpenDisplay(NULL);
    if(!display){fputs("Cannot open the RK X11 display\n",stderr);return 2;}
    XSetErrorHandler(handle_error);
    Window root=DefaultRootWindow(display);
    if(DisplayWidth(display,DefaultScreen(display))!=ADAS_W ||
       DisplayHeight(display,DefaultScreen(display))!=ADAS_H) {
        fputs("Return control requires the 1920x1080 HDMI desktop\n",stderr);
        XCloseDisplay(display);return 2;
    }
    XSetWindowAttributes attributes={0};
    attributes.override_redirect=True;
    attributes.event_mask=ButtonPressMask;
    Window button=XCreateWindow(display,root,0,0,
        ADAS_W,ADAS_H,0,0,InputOnly,CopyFromParent,
        CWOverrideRedirect|CWEventMask,&attributes);
    KeyCode escape=XKeysymToKeycode(display,XK_Escape);
    int opcode,event_base,error_base,major=2,minor=0;
    if(!XQueryExtension(display,"XInputExtension",&opcode,&event_base,&error_base) ||
       XIQueryVersion(display,&major,&minor)!=Success) {
        fputs("XInput2 is required for the physical Esc key\n",stderr);XCloseDisplay(display);return 3;
    }
    unsigned char mask[XIMaskLen(XI_RawKeyPress)]={0};
    XISetMask(mask,XI_RawKeyPress);
    XIEventMask events={XIAllMasterDevices,sizeof(mask),mask};
    XISelectEvents(display,root,&events,1);
    XMapRaised(display,button);
    XSync(display,False);
    if(x_error){fputs("Cannot bind the return control/Esc\n",stderr);XCloseDisplay(display);return 3;}
    puts("READY");fflush(stdout);
    XEvent event;
    while(!x_error) {
        XNextEvent(display,&event);
        int click=event.type==ButtonPress && event.xbutton.window==button && event.xbutton.button==Button1;
        int key=0;
        if(event.type==GenericEvent && event.xcookie.extension==opcode && XGetEventData(display,&event.xcookie)) {
            XIRawEvent *raw=event.xcookie.data;
            key=event.xcookie.evtype==XI_RawKeyPress && raw->detail==escape;
            XFreeEventData(display,&event.xcookie);
        }
        if(click||key) {
            puts(click?"RETURN_BUTTON":"RETURN_ESC");fflush(stdout);
            XCloseDisplay(display);return 0;
        }
    }
    XCloseDisplay(display);return 3;
}

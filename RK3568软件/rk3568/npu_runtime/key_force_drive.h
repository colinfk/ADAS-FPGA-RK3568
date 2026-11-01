#ifndef KEY_FORCE_DRIVE_H
#define KEY_FORCE_DRIVE_H
/* Key commands change SIMULATED vehicle state; never modify recorded trajectory.
 * command: 0 observe, 1 STOP, 2 FORCE_LEFT, 3 FORCE_STRAIGHT, 4 FORCE_RIGHT.
 * ego: 0 unknown, 1 recorded stopped, 2 recorded moving.
 * light: 0 unknown, 1 red, 2 green, 3 yellow. Inputs must be from this junction.
 * A confirmed red + stopped state arms the alarm. Force movement then latches
 * a violation, immediately alarming, even if the recorded camera stays still.
 * STOP clears the violation; confirmed nonred or >1s unknown clears red context.
 * Call reset on source change/seek. Millisecond clock is VIDEO time (may pause).
 */
typedef struct {
 int initialized,last_ms,last_red_ms,red_context,armed,motion,route,manual;
 int violation,alarm,event_count,last_command,signal_red;
} KeyDrive;
void key_drive_reset(KeyDrive *s);
void key_drive_step(KeyDrive *s,int time_ms,int light,int confirmed,int ego,int command);
/* Same core exported to WebAssembly for the video overlay. */



#endif

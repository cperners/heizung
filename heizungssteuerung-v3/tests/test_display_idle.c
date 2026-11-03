#include <assert.h>
#include <stdio.h>
#include "display_idle.h"
int main(void){
 display_idle_t s;display_idle_init(&s,100);
 assert(display_idle_step(&s,60099,false,false)==DISPLAY_IDLE_NONE);
 assert(display_idle_step(&s,60100,false,false)==DISPLAY_IDLE_BLANK);
 display_idle_commit(&s,DISPLAY_IDLE_BLANK,false,60100);assert(display_idle_accept_touch(&s));
 display_idle_commit(&s,DISPLAY_IDLE_BLANK,true,60100);assert(!display_idle_accept_touch(&s));
 assert(display_idle_step(&s,61000,true,false)==DISPLAY_IDLE_WAKE);
 display_idle_commit(&s,DISPLAY_IDLE_WAKE,false,61000);assert(s.blanked);
 display_idle_commit(&s,DISPLAY_IDLE_WAKE,true,62000);assert(!s.blanked&&!display_idle_accept_touch(&s));
 /* Held wake touch and empty polls never reach the underlying controls. */
 assert(display_idle_step(&s,62100,true,false)==DISPLAY_IDLE_NONE);assert(!display_idle_accept_touch(&s));
 display_idle_step(&s,62500,false,false);assert(!display_idle_accept_touch(&s));
 display_idle_step(&s,63000,false,true);assert(display_idle_accept_touch(&s));
 display_idle_step(&s,64000,true,false);assert(display_idle_accept_touch(&s));
 assert(display_idle_step(&s,123999,false,false)==DISPLAY_IDLE_NONE);
 assert(display_idle_step(&s,124000,false,false)==DISPLAY_IDLE_BLANK);
 /* Monotonic 32-bit millisecond counter wraps without premature blanking. */
 display_idle_init(&s,UINT32_MAX-1000);
 assert(display_idle_step(&s,1000,false,false)==DISPLAY_IDLE_NONE);
 assert(display_idle_step(&s,58999,false,false)==DISPLAY_IDLE_BLANK);
 puts("PASS: 60-s-Abschaltung, Aktivitaet, fehlgeschlagenes Schalten, Aufwecken ohne Bedienaktion, echte Freigabe nach Loslassen und Zeitueberlauf.");
}

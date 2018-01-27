/*
 * tc.h
 *
 * Created: 2018-01-27 15:19:23
 *  Author: ak93830
 */ 





#ifndef TC_H_
#define TC_H_


#include <asf.h>
#include "compiler.h"

#  define TC_CHANNEL                  0
#  define EXAMPLE_TC                  (&AVR32_TC)
#  define EXAMPLE_TC_IRQ_GROUP        AVR32_TC_IRQ_GROUP
#  define EXAMPLE_TC_IRQ              AVR32_TC_IRQ0
#  define FPBA                        FOSC0          // FOSC0 est a 12Mhz
#  define FALSE                       0

__attribute__((__interrupt__))

  // Configuration du peripherique TC
  static const tc_waveform_opt_t WAVEFORM_OPT =
  {
	  .channel  = TC_CHANNEL,                        // Channel selection.

	  .bswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOB.
	  .beevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOB.
	  .bcpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOB.
	  .bcpb     = TC_EVT_EFFECT_NOOP,                // RB compare effect on TIOB.

	  .aswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOA.
	  .aeevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOA.
	  .acpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOA: toggle.
	  .acpa     = TC_EVT_EFFECT_NOOP,                // RA compare effect on TIOA: toggle
	  .wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,// Waveform selection: Up mode with automatic trigger(reset) on RC compare.
	  .enetrg   = FALSE,                             // External event trigger enable.
	  .eevt     = 0,                                 // External event selection.
	  .eevtedg  = TC_SEL_NO_EDGE,                    // External event edge selection.
	  .cpcdis   = FALSE,                             // Counter disable when RC compare.
	  .cpcstop  = FALSE,                             // Counter clock stopped with RC compare.

	  .burst    = FALSE,                             // Burst signal selection.
	  .clki     = FALSE,                             // Clock inversion.
	  .tcclks   = TC_CLOCK_SOURCE_TC4                // Internal source clock 3, connected to fPBA / 8.
  };

  static const tc_interrupt_t TC_INTERRUPT =
  {
	  .etrgs = 0,
	  .ldrbs = 0,
	  .ldras = 0,
	  .cpcs  = 1,
	  .cpbs  = 0,
	  .cpas  = 0,
	  .lovrs = 0,
	  .covfs = 0
  };

#endif /* TC_H_ */
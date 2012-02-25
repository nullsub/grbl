/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon
  
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "dev_misc.h"
#include "stepper.h"
#include "config.h"
#include "settings.h"
#include <math.h>
#include <stdlib.h>
#include "nuts_bolts.h"
#include "planner.h"
#include "limits.h"

#include <stm32f10x.h>
#include <stm32f10x_conf.h>

#define STEPPER_X_A1 	GPIO_Pin_0	
#define STEPPER_X_A2 	GPIO_Pin_1	
#define STEPPER_X_B1 	GPIO_Pin_2	
#define STEPPER_X_B2 	GPIO_Pin_3

#define STEPPER_Y_A1    GPIO_Pin_4
#define STEPPER_Y_A2 	GPIO_Pin_5
#define STEPPER_Y_B1 	GPIO_Pin_6
#define STEPPER_Y_B2 	GPIO_Pin_7

#define STEPPER_Z_A1    GPIO_Pin_8
#define STEPPER_Z_A2 	GPIO_Pin_9
#define STEPPER_Z_B1 	GPIO_Pin_10 
#define STEPPER_Z_B2 	GPIO_Pin_11

#define STEPPER_X_ENABLE GPIO_Pin_8
#define STEPPER_Y_ENABLE GPIO_Pin_9

#define STEPPER_ALL_PINS (STEPPER_X_A1|STEPPER_X_A2|STEPPER_X_B1|STEPPER_X_B2|STEPPER_Y_A1|STEPPER_Y_A2|STEPPER_Y_B1|STEPPER_Y_B2|STEPPER_X_ENABLE|STEPPER_Y_ENABLE)

#define STEPPER_PORT 	GPIOC

#define DIR_FORWARD	1
#define DIR_BACKWARD	0

static int do_int = 0;
void do_full_step(int direction, int axis);
void do_half_step(int direction, int axis);// Some useful constants


#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)

#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define CYCLES_PER_ACCELERATION_TICK ((TICKS_PER_MICROSECOND*1000000)/ACCELERATION_TICKS_PER_SECOND)

static block_t *current_block;  // A pointer to the block currently being traced

// Variables used by The Stepper Driver Interrupt
static uint8_t out_bits;        // The next stepping-bits to be output
static int32_t counter_x,       // Counter variables for the bresenham line tracer
               counter_y, 
               counter_z;       
static uint32_t step_events_completed; // The number of step events executed in the current block
static volatile uint8_t busy; // true when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.

// Variables used by the trapezoid generation
static uint32_t cycles_per_step_event;        // The number of machine cycles between each step event
static uint32_t trapezoid_tick_cycle_counter; // The cycles since last trapezoid_tick. Used to generate ticks at a steady
                                              // pace without allocating a separate timer
static uint32_t trapezoid_adjusted_rate;      // The current rate of step_events according to the trapezoid generator
static uint32_t min_safe_rate;  // Minimum safe rate for full deceleration rate reduction step. Otherwise halves step_rate.
static uint8_t cycle_start;     // Cycle start flag to indicate program start and block processing.

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates by block->rate_delta
//  during the first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is always +/- block->rate_delta and is applied at a constant rate following the midpoint rule
//  by the trapezoid generator, which is called ACCELERATION_TICKS_PER_SECOND times per second.

static void set_step_events_per_minute(uint32_t steps_per_minute);

// Stepper state initialization
static void st_wake_up() 
{
  // Initialize stepper output bits
  out_bits = (0) ^ (settings.invert_mask); 
  // Enable steppers by resetting the stepper disable port
  //STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT);
  // Enable stepper driver interrupt
	do_int = 1; 
}

// Stepper shutdown
static void st_go_idle() 
{
  // Cycle finished. Set flag to false.
  cycle_start = false; 
  // Disable stepper driver interrupt
 	do_int = 0;
  // Force stepper dwell to lock axes for a defined amount of time to ensure the axes come to a complete
  // stop and not drift from residual inertial forces at the end of the last movement.
  #if STEPPER_IDLE_LOCK_TIME
    delay_ms(STEPPER_IDLE_LOCK_TIME);   
  #endif
  // Disable steppers by setting stepper disable
  //STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT);
}

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
static void trapezoid_generator_reset() 
{
  trapezoid_adjusted_rate = current_block->initial_rate;
  min_safe_rate = current_block->rate_delta + (current_block->rate_delta >> 1); // 1.5 x rate_delta
  trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2; // Start halfway for midpoint rule.
  set_step_events_per_minute(trapezoid_adjusted_rate); // Initialize cycles_per_step_event
}

// This function determines an acceleration velocity change every CYCLES_PER_ACCELERATION_TICK by
// keeping track of the number of elapsed cycles during a de/ac-celeration. The code assumes that 
// step_events occur significantly more often than the acceleration velocity iterations.
static uint8_t iterate_trapezoid_cycle_counter() 
{
  trapezoid_tick_cycle_counter += cycles_per_step_event;  
  if(trapezoid_tick_cycle_counter > CYCLES_PER_ACCELERATION_TICK) {
    trapezoid_tick_cycle_counter -= CYCLES_PER_ACCELERATION_TICK;
    return(true);
  } else {
    return(false);
  }
}          

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. It is executed at the rate set with
// config_step_timer. It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
// It is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port after each pulse. 
// The bresenham line tracer algorithm controls all three stepper outputs simultaneously with these two interrupts.
  
void tim2_isr() __attribute__ ((optimize(0)));// this interrupt breaks if it is optimized!!!!!!!!!!!!
void tim2_isr()
{
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);	//Interrupt Flag von TIM2 Löschen

  	if (busy) { return; } // The busy-flag is used to avoid reentering this interrupt
	
	toggle_led();	

	if(!do_int)
		return;

	if(out_bits & (1<<X_STEP_BIT)) {
		if(out_bits & (1<<X_DIRECTION_BIT)) {
			do_full_step(DIR_FORWARD, X_AXIS);
		} else {
			do_full_step(DIR_BACKWARD, X_AXIS);
		}
	}	
	if(out_bits & (1<<Y_STEP_BIT)) {
		if(out_bits & (1<<Y_DIRECTION_BIT)) {
			do_full_step(DIR_FORWARD, Y_AXIS);
		} else {
			do_full_step(DIR_BACKWARD, Y_AXIS);
		}	
	}
	if(out_bits & (1<<Z_STEP_BIT)) {
		if(out_bits & (1<<Z_DIRECTION_BIT)) {
			do_full_step(DIR_FORWARD, Y_AXIS);
		} else {
			do_full_step(DIR_BACKWARD, Y_AXIS);
		}	
	}
  

  busy = true;
  // Re-enable interrupts to allow ISR_TIMER2_OVERFLOW to trigger on-time and allow serial communications
  // regardless of time in this handler. The following code prepares the stepper driver for the next
  // step interrupt compare and will always finish before returning to the main program.
  //sei();
  
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer? If so, initialize next motion.
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      step_events_completed = 0;     
    } else {
      st_go_idle();
    }    
  } 

  if (current_block != NULL) {
    // Execute step displacement profile by bresenham line algorithm
    out_bits = current_block->direction_bits;
    counter_x += current_block->steps_x;
    if (counter_x > 0) {
      out_bits |= (1<<X_STEP_BIT);
      counter_x -= current_block->step_event_count;
    }
    counter_y += current_block->steps_y;
    if (counter_y > 0) {
      out_bits |= (1<<Y_STEP_BIT);
      counter_y -= current_block->step_event_count;
    }
    counter_z += current_block->steps_z;
    if (counter_z > 0) {
      out_bits |= (1<<Z_STEP_BIT);
      counter_z -= current_block->step_event_count;
    }
    
    step_events_completed++; // Iterate step events

    // While in block steps, check for de/ac-celeration events and execute them accordingly.
    if (step_events_completed < current_block->step_event_count) {
      // The trapezoid generator always checks step event location to ensure de/ac-celerations are 
      // executed and terminated at exactly the right time. This helps prevent over/under-shooting
      // the target position and speed. 
      // NOTE: By increasing the ACCELERATION_TICKS_PER_SECOND in config.h, the resolution of the 
      // discrete velocity changes increase and accuracy can increase as well to a point. Numerical 
      // round-off errors can effect this, if set too high. This is important to note if a user has 
      // very high acceleration and/or feedrate requirements for their machine.
      if (step_events_completed < current_block->accelerate_until) {
        // Iterate cycle counter and check if speeds need to be increased.
        if ( iterate_trapezoid_cycle_counter() ) {
          trapezoid_adjusted_rate += current_block->rate_delta;
          if (trapezoid_adjusted_rate >= current_block->nominal_rate) {
            // Reached nominal rate a little early. Cruise at nominal rate until decelerate_after.
            trapezoid_adjusted_rate = current_block->nominal_rate;
          }
          set_step_events_per_minute(trapezoid_adjusted_rate);
        }
      } else if (step_events_completed >= current_block->decelerate_after) {
        // Reset trapezoid tick cycle counter to make sure that the deceleration is performed the
        // same every time. Reset to CYCLES_PER_ACCELERATION_TICK/2 to follow the midpoint rule for
        // an accurate approximation of the deceleration curve.
        if (step_events_completed == current_block-> decelerate_after) {
          trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2;
        } else {
          // Iterate cycle counter and check if speeds need to be reduced.
          if ( iterate_trapezoid_cycle_counter() ) {  
            // NOTE: We will only do a full speed reduction if the result is more than the minimum safe 
            // rate, initialized in trapezoid reset as 1.5 x rate_delta. Otherwise, reduce the speed by
            // half increments until finished. The half increments are guaranteed not to exceed the 
            // CNC acceleration limits, because they will never be greater than rate_delta. This catches
            // small errors that might leave steps hanging after the last trapezoid tick or a very slow
            // step rate at the end of a full stop deceleration in certain situations. The half rate 
            // reductions should only be called once or twice per block and create a nice smooth 
            // end deceleration.
            if (trapezoid_adjusted_rate > min_safe_rate) {
              trapezoid_adjusted_rate -= current_block->rate_delta;
            } else {
              trapezoid_adjusted_rate >>= 1; // Bit shift divide by 2
            }
            if (trapezoid_adjusted_rate < current_block->final_rate) {
              // Reached final rate a little early. Cruise to end of block at final rate.
              trapezoid_adjusted_rate = current_block->final_rate;
            }
            set_step_events_per_minute(trapezoid_adjusted_rate);
          }
        }
      } else {
        // No accelerations. Make sure we cruise exactly at the nominal rate.
        if (trapezoid_adjusted_rate != current_block->nominal_rate) {
          trapezoid_adjusted_rate = current_block->nominal_rate;
          set_step_events_per_minute(trapezoid_adjusted_rate);
        }
      }
            
    } else {   
      // If current block is finished, reset pointer 
      current_block = NULL;
      plan_discard_current_block();
    }
  }
  out_bits ^= settings.invert_mask;  // Apply stepper invert mask    
  busy=false;
}

// This interrupt is set up by ISR_TIMER1_COMPAREA when it sets the motor port bits. It resets
// the motor port after a short period (settings.pulse_microseconds) completing one step cycle.
// TODO: It is possible for the serial interrupts to delay this interrupt by a few microseconds, if
// they execute right before this interrupt. Not a big deal, but could use some TLC at some point.
/*
ISR(TIMER2_OVF_vect)
{
  // Reset stepping pins (leave the direction pins)
  STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (settings.invert_mask & STEP_MASK); 
  TCCR2B = 0; // Disable Timer2 to prevent re-entering this interrupt when it's not needed. 
}
*/

// Initialize and start the stepper motor subsystem
void st_init()
{

	GPIO_InitTypeDef gpio_init;

	gpio_init.GPIO_Pin = STEPPER_ALL_PINS;	
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(STEPPER_PORT, &gpio_init);
	
	STEPPER_PORT->BRR = STEPPER_ALL_PINS;

	TIM_TimeBaseInitTypeDef timer_settings;
	/* Time base configuration */
	timer_settings.TIM_Period = MINIMUM_STEPS_PER_MINUTE;
	timer_settings.TIM_Prescaler = 1-1;
	timer_settings.TIM_ClockDivision = TIM_CKD_DIV1;
	timer_settings.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &timer_settings);

	/* Clear TIM2 update pending flag */
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	TIM_Cmd(TIM2, ENABLE);
  
  set_step_events_per_minute(MINIMUM_STEPS_PER_MINUTE);
  trapezoid_tick_cycle_counter = 0;
  current_block = NULL;
  busy = false;
  
  // Start in the idle state
  st_go_idle();
}

// Block until all buffered steps are executed
void st_synchronize()
{
  while(plan_get_current_block()) { sleep_mode(); }    
}

// Configures the prescaler and ceiling of timer 1 to produce the given rate as accurately as possible.
// Returns the actual number of cycles per interrupt
/*
static uint32_t config_step_timer(uint32_t cycles)
{
  uint16_t ceiling;
  uint16_t prescaler;
  uint32_t actual_cycles;
  if (cycles <= 0xffffL) {
    ceiling = cycles;
    prescaler = 0; // prescaler: 0
    actual_cycles = ceiling;
  } else if (cycles <= 0x7ffffL) {
    ceiling = cycles >> 3;
    prescaler = 1; // prescaler: 8
    actual_cycles = ceiling * 8L;
  } else if (cycles <= 0x3fffffL) {
    ceiling =  cycles >> 6;
    prescaler = 2; // prescaler: 64
    actual_cycles = ceiling * 64L;
  } else if (cycles <= 0xffffffL) {
    ceiling =  (cycles >> 8);
    prescaler = 3; // prescaler: 256
    actual_cycles = ceiling * 256L;
  } else if (cycles <= 0x3ffffffL) {
    ceiling = (cycles >> 10);
    prescaler = 4; // prescaler: 1024
    actual_cycles = ceiling * 1024L;    
  } else {
    // Okay, that was slower than we actually go. Just set the slowest speed
    ceiling = 0xffff;
    prescaler = 4;
    actual_cycles = 0xffff * 1024;
  }
  // Set prescaler
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | ((prescaler+1)<<CS10);
  // Set ceiling
  OCR1A = ceiling;
  return(actual_cycles);
}
*/
static void set_step_events_per_minute(uint32_t steps_per_minute) 
{
  if (steps_per_minute < MINIMUM_STEPS_PER_MINUTE) { steps_per_minute = MINIMUM_STEPS_PER_MINUTE; }
  //(1/(steps_per_minute*60))/(1/(F_CPU/PRESCALER)) 
  cycles_per_step_event = ((24000000/60)/(steps_per_minute));
  TIM2->ARR =  cycles_per_step_event;
}

void st_go_home()
{
  limits_go_home();  
  plan_set_current_position(0,0,0);
}

// Planner external interface to start stepper interrupt and execute the blocks in queue.
void st_cycle_start() 
{
  if (!cycle_start) {
    cycle_start = true;
    st_wake_up();
  }
}

const int stepper_pins[3][4] = {
		{STEPPER_X_A1, STEPPER_X_A2, STEPPER_X_B1, STEPPER_X_B2},
		{STEPPER_Y_A1, STEPPER_Y_A2, STEPPER_Y_B1, STEPPER_Y_B2},
		{STEPPER_Z_A1, STEPPER_Z_A2, STEPPER_Z_B1, STEPPER_Z_B2},
		};

void do_half_step(int direction, int axis) 
{
	static int crrnt_step[3] = {0,0,0};
	
	if(direction == DIR_FORWARD) {
		crrnt_step[axis] ++;
		if (crrnt_step[axis] >= 8)
			crrnt_step[axis] = 0;
	}
	else{
		if(crrnt_step[axis] == 0)
			crrnt_step[axis] = 8;
		crrnt_step[axis] --;			
	}

		//BRR is clear and BSRR is set half step is probably broken.. I dont use it.
	switch(crrnt_step[axis]) {
		case 0:
			STEPPER_PORT->BRR = stepper_pins[axis][0];
			STEPPER_PORT->BSRR = stepper_pins[axis][3];
			break;
		case 1:
			STEPPER_PORT->BRR =  stepper_pins[axis][0] | stepper_pins[axis][2];
; 	
			break;
		case 2:
			STEPPER_PORT->BRR = stepper_pins[axis][2];
			STEPPER_PORT->BSRR = stepper_pins[axis][0];
			break;	
		case 3:
			STEPPER_PORT->BRR =  stepper_pins[axis][1] | stepper_pins[axis][2];
			break;
		case 4:		
			STEPPER_PORT->BRR = stepper_pins[axis][1];
			STEPPER_PORT->BSRR = stepper_pins[axis][2];
			break;	
		case 5: 
			STEPPER_PORT->BRR =  stepper_pins[axis][1] | stepper_pins[axis][3];
			break;
		case 6:	
			STEPPER_PORT->BRR = stepper_pins[axis][3];
			STEPPER_PORT->BSRR = stepper_pins[axis][1];
			break;	
		case 7:
			STEPPER_PORT->BRR =  stepper_pins[axis][0] | stepper_pins[axis][3];
			break;
	}
}

void do_full_step(int direction, int axis) 
{
	static unsigned int crrnt_step[3] = {0,0,0};
	
	if(direction == DIR_FORWARD) {
		crrnt_step[axis] ++;
		if (crrnt_step[axis] >= 4)
			crrnt_step[axis] = 0;
	}
	else{
		if(crrnt_step[axis] == 0)
			crrnt_step[axis] = 4;
		crrnt_step[axis] --;			
	}
	
	switch(crrnt_step[axis]) {
		case 0:
			STEPPER_PORT->BRR = stepper_pins[axis][0] | stepper_pins[axis][2];
			STEPPER_PORT->BSRR = stepper_pins[axis][3] | stepper_pins[axis][1];
			break;
		case 1:			
			STEPPER_PORT->BRR = stepper_pins[axis][1] | stepper_pins[axis][2];
			STEPPER_PORT->BSRR = stepper_pins[axis][0] | stepper_pins[axis][3];
			break;
		case 2:				
			STEPPER_PORT->BRR = stepper_pins[axis][1] | stepper_pins[axis][3];
			STEPPER_PORT->BSRR = stepper_pins[axis][2] | stepper_pins[axis][0];
			break;
		case 3:	
			STEPPER_PORT->BRR = stepper_pins[axis][0] | stepper_pins[axis][3];
			STEPPER_PORT->BSRR = stepper_pins[axis][1] | stepper_pins[axis][2];
			break;
	}
}


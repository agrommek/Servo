/*
  Copyright (c) 2015 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#if defined(ARDUINO_ARCH_SAMD)

#include <Arduino.h>
#include <Servo.h>


/* 
 * Use different prescalers depending on F_CPU (avoid overflowing 16-bit counter).
 * Make it general, so the library also works when over- or underclocking the
 * CPU.
 * 
 * Note: There are no prescalers of size 32 or 128 for TC instances!
 */
#if(F_CPU >  197000000) // between 197 MHz and ~7.7 GHz (more than any SAMD MCU can do)
    #define GCLK_PRESCALER (256)
    #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV256
#elif(F_CPU > 49000000) // between 49 MHz and 197 MHz (--> SAMD51@120MHz)
    #define GCLK_PRESCALER (64)
    #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV64
#elif(F_CPU > 24000000) // between 24 MHz and 49 MHz (--> SAMD21@48MHz)
    #define GCLK_PRESCALER (16)
    #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV16
#elif(F_CPU > 12000000) // between 12 MHz and 24 MHz
    #define GCLK_PRESCALER (8)
    #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV8
#elif(F_CPU >  6000000) // between 6 MHz and 12 MHz
    #define GCLK_PRESCALER (4)
    #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV4
#elif(F_CPU >  3000000) // between 3 MHz and 6 MHz
    #define GCLK_PRESCALER (2)
    #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV2
#elif(F_CPU <= 3000000) // less than 3 MHz
    #define GCLK_PRESCALER (1)
    #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV1
#else
    #error unsupported CPU clock frequency
#endif

/*
 * Nomenclature:
 * 
 * timer         The TC instance in use. Every TC instance can handle up to 12 servos.
 *               TC instances in used by the servo library start numbering at 0. 
 * CC_register   Each timer has exacty two Compare/Capture Channels. 
 *               The corresponding registers on the SAMD architecture are denoted 
 *               CC0 and CC1. Each CCx register can handle up to 6 servos by multiplexing.
 * CC_channel    Index of servo within a given timer AND within a given CC_register.
 *               CC_channel is 0-based, the number runs from 0 to 5 (6 servos per
 *               CC_register).
 * servo_index   Unique number for different servos. The total number of servos depends
 *               on the number of timers available. For a single timer, servo_index runs 
 *               from 0 to 11. For two timers, servo_index runs from 0 to 23, etc.
 * 
 *               Example:
 *               The servo with the servo index 20 would be handled by:
 *               - timer == 1 (i.e. the second timer)
 *               - CC_register == CC1 (i.e. the second CC register of the TC instance) 
 *               - CC_channel == 2 (i.e. the third pulse on the CC register)
 */
 
// macro to convert microseconds to ticks
#define usToTicks(_us)    ((clockCyclesPerMicrosecond() * _us) / (GCLK_PRESCALER))

// macro to convert ticks back to microseconds
#define ticksToUs(_ticks) (((unsigned) _ticks * (GCLK_PRESCALER)) / clockCyclesPerMicrosecond())

// wrap around the counter after this number of tick corresponding PWM
// frequency of 50 Hz <--> a refresh interval of 20 ms
#define REFRESH_INTERVAL_TICKS  (usToTicks(REFRESH_INTERVAL))

// compensation ticks to trim adjust for digitalWrite delays
#define TRIM_DURATION  5

// When initializing a timer, use this delay to schedule the very first interrupt.
// The value itself is not so important. Must be large enough that the first 
// interrupt is not missed, but small enough that the timer does not overflow.
// Anything from a few hundred microseconds to a couple of milliseconds 
// should work.
#define FIRST_PULSE_TICKS  (usToTicks(500)) // 500 microseconds

// number of servos per CC register
#define CC_CHANNELS_PER_CC_REGISTER             (SERVOS_PER_TIMER / 2)

// macro to calculate timer which handles servo at given index
#define SERVO_INDEX_TO_TIMER(_servo_nbr)        ((timer16_Sequence_t)(_servo_nbr / SERVOS_PER_TIMER))

// macro to calculate servo_index from timer, CC_register, and CC_channel
#define SERVO_INDEX(_timer,_cc_register,_cc_channel)    ( (_timer * (SERVOS_PER_TIMER)) + ((_cc_register) * (CC_CHANNELS_PER_CC_REGISTER)) + (_cc_channel) )

// macro to calculate servo instance (i.e. the object) from timer, CC_register, and CC_channel
#define SERVO(_timer,_cc_register,_cc_channel) (servos[(SERVO_INDEX(_timer,_cc_register,_cc_channel))])

// macro to check if servo is configured by comparing servo index to global variable ServoCount
#define SERVO_EXISTS(_timer,_cc_register,_cc_channel) (SERVO_INDEX(_timer,_cc_register,_cc_channel) < ServoCount)

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)   // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)   // maximum value in uS for this servo

// static array of servo structures
static servo_t servos[MAX_SERVOS];

// the total number of configured servos
volatile uint8_t ServoCount = 0;

// "phase" for each timer and CC_register
// indicates which servo to pulse and which value to set the CC register to for 
// the next interrupt trigger
static volatile uint8_t phase_index[_Nbr_16timers][_Nbr_CC_registers];

// Flags to request a rollover/reset-to-zero of TC COUNT16 register.
// COUNT16 for a given timer x should be reset when rollover_flag[x][_cc0]
// and rollover_flag[x][_cc1] are both true.
static volatile bool rollover_flag[_Nbr_16timers][_Nbr_CC_registers] {
    #if defined(_useTimer1)
        false, false, 
    #endif
    #if defined (_useTimer2)
        false, false,
    #endif
};

/************ static functions common to all instances ***********************/

/* 
 * Read the COUNT16 register of a TC instance and return it.
 * Reading the value takes about...
 *  ... 2-4 microseconds on SAMD21 @ 48 MHz
 *  ... 0-2 microseconds on SAMD51 @ 120-200 MHz
 */
static inline uint16_t __attribute__((always_inline)) getTcCounter16Value(Tc *tc) {
    #if defined(__SAMD21__)
        tc->COUNT16.READREQ.reg = TC_READREQ_RREQ |                         // request read...
                                  TC_READREQ_ADDR(TC_COUNT16_COUNT_OFFSET); // ...of COUNT16 register
        while(tc->COUNT16.STATUS.bit.SYNCBUSY);                             // wait for sync to complete
    #elif defined(__SAMD51__)   
        tc->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBCLR_CMD_READSYNC_Val;        // issue read request command COUNT register synchronization to CTRLB register
        while(tc->COUNT16.SYNCBUSY.bit.CTRLB);                              // wait until CTRLB register is write synchronized
    #endif
    return (uint16_t) tc->COUNT16.COUNT.reg;                                // finally, read out the synchronized value
}

/* 
 * Reset the COUNT16 register of a TC instance to 0x00.
 */
static inline void __attribute__((always_inline)) resetTcCounter16(Tc *tc) {
    tc->COUNT16.COUNT.reg = (uint16_t) TC_COUNT16_COUNT_RESETVALUE;
    #if defined(__SAMD21__)
        while(tc->COUNT16.STATUS.bit.SYNCBUSY);
    #elif defined(__SAMD51__)
        while(tc->COUNT16.SYNCBUSY.bit.COUNT);
    #endif
}

/*
 * Set the a Compare Channel register (CC register) of a TC instance in
 * 16-bit-mode to the given 16-bit-value.
 */
static inline void __attribute__((always_inline)) setCCRegisterValue16(Tc *tc, CC_register_t cc_register, uint16_t new_value) {
    tc->COUNT16.CC[cc_register].reg = new_value;
    #if defined(__SAMD21__)
        while(tc->COUNT16.STATUS.bit.SYNCBUSY);
    #elif defined(__SAMD51__)
        if(cc_register == 0) {
            while(tc->COUNT16.SYNCBUSY.bit.CC0);
        } else if(cc_register == 1) {
            while(tc->COUNT16.SYNCBUSY.bit.CC1);
        }
    #endif
}

/*
Features:
- Start of pulse train at COUNT == 0 for each CC register
- seven interrupts for six pulses per cycle

- points 1..5 are handled identically:
  -- set pin of servo in phase i-1 LOW
  -- the next interrupt is set to happen at COUNT = COUNT + current servo ticks
- points 0 and 6 are different:
  - at point 0:
   -- there is no pin to set LOW, as we are ending the REFRESH_INTERVAL
   -- rollover of COUNT is triggered
   -- next interrupt is set to happen at COUNT == current servo ticks
  - at point 6:
   -- there is no pin to set HIGH
   -- the next interrupt is set to happen after REFRESH_INTERVAL (i.e. at 20 ms)
 
_______________________________
|    |    |    |    |    |    |          |
| C0 | C1 | C2 | C3 | C4 | C5 | REFRESH  |
|____|____|____|____|____|____|__________|  
*    *    *    *    *    *    *          *      
0    1    2    3    4    5    6          0      point i

interrupts always occur at COUNT == *
COUNT at point i==0: 20 milliseconds --> reset to 0 milliseconds
*/
void handle_interrupt(Tc *pTc, timer16_Sequence_t timer, CC_register_t cc_register) {
    // set previous servo LOW
    if (phase_index[timer][cc_register] > 0) {
        // check if previous servo exists
        // if it does, set pulse LOW, no matter what the current state is
        if (SERVO_EXISTS(timer, cc_register, phase_index[timer][cc_register]-1)) {
            digitalWrite(SERVO(timer, cc_register, phase_index[timer][cc_register]-1).Pin.nbr, LOW);
        }
    }
    uint16_t next_interrupt_ticks = 0;
    if (SERVO_EXISTS(timer, cc_register, phase_index[timer][cc_register])) {
        // get a pointer to the current servo
        servo_t *current_servo = &(SERVO(timer, cc_register, phase_index[timer][cc_register]));
        // handle phase 0
        if (phase_index[timer][cc_register] == 0) {
            if (current_servo->Pin.isActive == true) {      // if current servo is active...
                digitalWrite(current_servo->Pin.nbr, HIGH); // ...set pin HIGH
            }
            rollover_flag[timer][cc_register] = true;       // trigger rollover of counter
            next_interrupt_ticks = current_servo->ticks;    // set next interrupt to end of first pulse
            phase_index[timer][cc_register]++;              // increment phase
        }
        // handle phases 1..5
        else if (phase_index[timer][cc_register] < CC_CHANNELS_PER_CC_REGISTER) {
            if (current_servo->Pin.isActive == true) {      // if current servo is active...
                digitalWrite(current_servo->Pin.nbr, HIGH); // ...set pin HIGH
            }
            // set next interrupt to current counter value + length of current pulse
            next_interrupt_ticks = getTcCounter16Value(pTc) + current_servo->ticks;
            phase_index[timer][cc_register]++;              // increment phase
        }
        // handle phase 6
        else if (phase_index[timer][cc_register] == CC_CHANNELS_PER_CC_REGISTER) {
            next_interrupt_ticks = REFRESH_INTERVAL_TICKS;  // set next interrupt to end of refresh cycle (20 ms)
            phase_index[timer][cc_register] = 0;            // wrap around phase to 0
        }
    }
    else { // servo does not exist 
           // --> servos in later phases do not exist either
           // --> "fast forward" to phase 0
        next_interrupt_ticks = REFRESH_INTERVAL_TICKS;      // set next interrupt to end of refresh cycle (20 ms)
        phase_index[timer][cc_register] = 0;                // wrap around phase to 0
    }
    // update value in compare/capture register for next match interrupt
    setCCRegisterValue16(pTc, cc_register, next_interrupt_ticks);
}

/*
 * On SAMD architecture, all interrupts generated from a single TC instance
 * are ORed together. This means, that the ISR has to figure out by itself, 
 * which type of interrupt(s) was/were actually triggered.
 * This function does the "figuring out" and calls the final handler 
 * function handle_interrupt() with the proper parameters.
 */
static inline void __attribute__((always_inline)) dispatch_interrupt(Tc *pTc, timer16_Sequence_t timer) {
    // Note:
    // Store bitpattern for INTFLAGs in a local variable. For some reason,
    // *all* INTFLAGs appear to be reset when setting new values to CCx.
    // MC0 appears to be reset when reading from/writing to CC1 and vice versa.
    // This is not documented in the data sheet.
    // Solution: Store INTFLAGs at the beginning of ISR, analyze later.
    uint8_t intflags = pTc->COUNT16.INTFLAG.reg;
    if (intflags & TC_INTFLAG_MC0) {        // we have a match/capture interrupt on CC0
        handle_interrupt(pTc, timer, _cc0); // handle the interrupt
        pTc->COUNT16.INTFLAG.bit.MC0 = 1;   // reset match/capture interrupt flag of CC0
    }
    if (intflags & TC_INTFLAG_MC1) {        // we have a match/capture interrupt on CC1
        handle_interrupt(pTc, timer, _cc1); // handle the interrupt
        pTc->COUNT16.INTFLAG.bit.MC1 = 1;   // reset match/capture interrupt flag of CC1
    }
    // reset COUNT only if handle_interrupt() signals rollover for *both* 
    // CC registers of a given timer instance 
    if (rollover_flag[timer][_cc0] && rollover_flag[timer][_cc1]) {
        resetTcCounter16(pTc);
        rollover_flag[timer][_cc0] = false;
        rollover_flag[timer][_cc1] = false;
    }
}

#if defined (_useTimer1)
void TIMER1_HANDLER(void) {
    dispatch_interrupt(TIMER1_TC, _timer1);
}
#endif
#if defined (_useTimer2)
void TIMER2_HANDLER(void) {
    dispatch_interrupt(TIMER2_TC, _timer2);
}
#endif

static inline void resetTC (Tc* TCx) {
    // Disable TCx
    TCx->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    #if defined(__SAMD21__)
        while(TCx->COUNT16.STATUS.bit.SYNCBUSY)
    #elif defined(__SAMD51__)
        while(TCx->COUNT16.SYNCBUSY.bit.ENABLE);
    #endif
    // Reset TCx
    TCx->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    #if defined(__SAMD21__)
        while(TCx->COUNT16.STATUS.bit.SYNCBUSY)
    #elif defined(__SAMD51__)
        while(TCx->COUNT16.SYNCBUSY.bit.SWRST);
    #endif
    while (TCx->COUNT16.CTRLA.bit.SWRST);
}

static void _initISR(Tc *tc, IRQn_Type irqn, uint8_t gcmForTimer, timer16_Sequence_t timer) {
    // Select GCLK0 as timer/counter input clock source
    #if defined(__SAMD21__)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(gcmForTimer));
        while (GCLK->STATUS.bit.SYNCBUSY);
    #elif defined(__SAMD51__)
        int idx = gcmForTimer;           // see datasheet Table 14-9
        GCLK->PCHCTRL[idx].bit.GEN  = 0; // Select GCLK0 as periph clock source
        GCLK->PCHCTRL[idx].bit.CHEN = 1; // Enable peripheral
        while(!GCLK->PCHCTRL[idx].bit.CHEN);
    #endif
    // Reset the timer to default settings
    resetTC(tc);
    // Set timer counter mode to 16 bits
    tc->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
    #if defined(__SAMD21__)
        // Set timer compare mode as normal frequency.
        tc->COUNT16.CTRLA.bit.WAVEGEN = TC_CTRLA_WAVEGEN_NFRQ_Val; 
        // Set the prescaler factor. At nominal 48 MHz GCLK_TC this is 3000 ticks per millisecond.
        tc->COUNT16.CTRLA.reg |= (uint16_t) TC_CTRLA_PRESCALER_USED;
    #elif defined(__SAMD51__)
        // Set timer compare mode as normal frequency.
        tc->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_NFRQ_Val;
        // set the prescaler factor. At nominal 120 MHz GLCK_TC this is 1875 ticks per millisecond.
        tc->COUNT16.CTRLA.reg |= (uint32_t) TC_CTRLA_PRESCALER_USED;
    #endif
    // set direction to "counting upwards" by clearing the "count downwards" bit
    tc->COUNT16.CTRLBCLR.bit.DIR = 1;
    #if defined(__SAMD21__)
        while(tc->COUNT16.STATUS.bit.SYNCBUSY)
    #elif defined(__SAMD51__)
        while(tc->COUNT16.SYNCBUSY.bit.CTRLB);
    #endif
    // configure very first compare values for CC0 and CC1
    setCCRegisterValue16(tc, _cc0, FIRST_PULSE_TICKS);
    setCCRegisterValue16(tc, _cc1, FIRST_PULSE_TICKS);
    // configure interrupt requests
    NVIC_DisableIRQ(irqn);
    NVIC_ClearPendingIRQ(irqn);
    NVIC_SetPriority(irqn, 0);
    NVIC_EnableIRQ(irqn);
    // enable the match channel interrupt request for both CC registers
    tc->COUNT16.INTENSET.reg = (TC_INTENSET_MC0 | TC_INTENSET_MC1);
    // (re)set phases for this timer
    phase_index[timer][_cc0] = 0;
    phase_index[timer][_cc1] = 0;
    // (re)set overflow flags for this timer
    rollover_flag[timer][_cc0] = false;
    rollover_flag[timer][_cc1] = false;
    // enable the timer and start it
    tc->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    #if defined(__SAMD21__)
        while(tc->COUNT16.STATUS.bit.SYNCBUSY);
    #elif defined(__SAMD51__)
        while(tc->COUNT16.SYNCBUSY.bit.ENABLE);
    #endif
}

static void initISR(timer16_Sequence_t timer) {
    #if defined (_useTimer1)
        if (timer == _timer1) {
            _initISR(TIMER1_TC, TIMER1_IRQn, TIMER1_GCLK_ID, _timer1);
        }
    #endif
    #if defined (_useTimer2)
        if (timer == _timer2) {
            _initISR(TIMER2_TC, TIMER2_IRQn, TIMER2_GCLK_ID, _timer2);
        }
    #endif
}

static void finISR(timer16_Sequence_t timer) {
    // Disable timer by disabling the the match interrupt requests for both CC registers.
    // The timer is still running, but does not bother the CPU anymore.
    (void)timer;
    #if defined (_useTimer1)
        if (timer == _timer1) {
            TIMER1_TC->COUNT16.INTENCLR.reg = TC_INTENCLR_MC0 | TC_INTENCLR_MC1;
        }
    #endif
    #if defined (_useTimer2)
        if (timer == _timer1) {
            TIMER2_TC->COUNT16.INTENCLR.reg = TC_INTENCLR_MC0 | TC_INTENCLR_MC1;
        }
    #endif
}

static boolean isTimerActive(timer16_Sequence_t timer) {
    // returns true if any servo is active on this timer
    // loop over all registers (0..1)
    for (uint8_t cc_register=0; cc_register < _Nbr_CC_registers; cc_register++) {
        // loop over all channels on this register (0..5)
        for (uint8_t channel=0; channel < CC_CHANNELS_PER_CC_REGISTER; channel++) {
            if ((SERVO(timer, cc_register, channel)).Pin.isActive == true) {
                return true;
            }
        }
    }
    return false;
}

/****************** end of static functions ******************************/

Servo::Servo()
{
  if (ServoCount < MAX_SERVOS) {
    this->servoIndex = ServoCount++;                    // assign a servo index to this instance
    servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);   // store default values
  } else {
    this->servoIndex = INVALID_SERVO;  // too many servos
  }
}

uint8_t Servo::attach(int pin)
{
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int min, int max)
{
  timer16_Sequence_t timer;

  if (this->servoIndex < MAX_SERVOS) {
    pinMode(pin, OUTPUT);                                   // set servo pin to output
    servos[this->servoIndex].Pin.nbr = pin;
    // todo min/max check: abs(min - MIN_PULSE_WIDTH) /4 < 128
    this->min  = (MIN_PULSE_WIDTH - min)/4; //resolution of min/max is 4 uS
    this->max  = (MAX_PULSE_WIDTH - max)/4;
    // initialize the timer if it has not already been initialized
    timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if (isTimerActive(timer) == false) {
      initISR(timer);
    }
    servos[this->servoIndex].Pin.isActive = true;  // this must be set after the check for isTimerActive
  }
  return this->servoIndex;
}

void Servo::detach()
{
  timer16_Sequence_t timer;

  servos[this->servoIndex].Pin.isActive = false;
  timer = SERVO_INDEX_TO_TIMER(servoIndex);
  if(isTimerActive(timer) == false) {
    finISR(timer);
  }
}

void Servo::write(int value)
{
  // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
  if (value < MIN_PULSE_WIDTH)
  {
    if (value < 0)
      value = 0;
    else if (value > 180)
      value = 180;

    value = map(value, 0, 180, SERVO_MIN(), SERVO_MAX());
  }
  writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value)
{
  // calculate and store the values for the given channel
  byte channel = this->servoIndex;
  if( (channel < MAX_SERVOS) )   // ensure channel is valid
  {
    if (value < SERVO_MIN())          // ensure pulse width is valid
      value = SERVO_MIN();
    else if (value > SERVO_MAX())
      value = SERVO_MAX();

    value = value - TRIM_DURATION;
    value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead
    servos[channel].ticks = value;
  }
}

int Servo::read() // return the value as degrees
{
  return map(readMicroseconds()+1, SERVO_MIN(), SERVO_MAX(), 0, 180);
}

int Servo::readMicroseconds()
{
  unsigned int pulsewidth;
  if (this->servoIndex != INVALID_SERVO)
    pulsewidth = ticksToUs(servos[this->servoIndex].ticks)  + TRIM_DURATION;
  else
    pulsewidth  = 0;

  return pulsewidth;
}

bool Servo::attached()
{
  return servos[this->servoIndex].Pin.isActive;
}

#endif // ARDUINO_ARCH_SAMD

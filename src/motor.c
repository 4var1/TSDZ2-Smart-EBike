/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *  TODO - battery current measurement is very low - so the motor runs away as motor current (batt/pwm) is almost always zero or close to zero
 *  TODO - why is the pedal weight + c.60?
 */

#include <stdint.h>
#include <stdio.h>
#include "motor.h"
#include "interrupts.h"
#include "stm8s_gpio.h"
#include "stm8s_tim1.h"
#include "stm8s_wwdg.h"
#include "motor.h"
#include "ebike_app.h"
#include "pins.h"
#include "brake.h"
#include "pwm.h"
#include "config.h"
#include "adc.h"
#include "utils.h"
#include "uart.h"
#include "adc.h"
#include "watchdog.h"
#include "math.h"
#include "main.h"

#define SVM_TABLE_LEN   256
#define SIN_TABLE_LEN   60

static const uint8_t ui8_svm_table[SVM_TABLE_LEN] = { 199, 200, 202, 203, 204, 205, 206, 207, 208, 208, 209, 210, 210, 211, 211, 211,
        212, 212, 212, 212, 212, 212, 212, 212, 211, 211, 211, 210, 210, 209, 208, 208, 207, 206, 206, 205, 204, 203,
        202, 201, 200, 199, 196, 192, 188, 184, 180, 176, 172, 167, 163, 159, 154, 150, 146, 141, 137, 133, 128, 124,
        119, 115, 110, 106, 102, 97, 93, 88, 84, 79, 75, 71, 66, 62, 58, 53, 49, 45, 40, 36, 32, 28, 24, 20, 16, 13, 12,
        11, 10, 9, 8, 7, 6, 6, 5, 4, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7, 8, 9,
        10, 12, 13, 14, 13, 12, 10, 9, 8, 7, 6, 5, 4, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4,
        4, 5, 6, 6, 7, 8, 9, 10, 11, 12, 13, 16, 20, 24, 28, 32, 36, 40, 45, 49, 53, 58, 62, 66, 71, 75, 79, 84, 88, 93,
        97, 102, 106, 110, 115, 119, 124, 128, 133, 137, 141, 146, 150, 154, 159, 163, 167, 172, 176, 180, 184, 188,
        192, 196, 199, 200, 201, 202, 203, 204, 205, 206, 206, 207, 208, 208, 209, 210, 210, 211, 211, 211, 212, 212,
        212, 212, 212, 212, 212, 212, 211, 211, 211, 210, 210, 209, 208, 208, 207, 206, 205, 204, 203, 202, 200, 199,
        198 };


uint8_t ui8_asin_table [128] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9,
 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 16, 16, 16, 17, 17, 18, 18, 18, 19,
  19, 19, 20, 20, 20, 21, 21, 21, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 26, 26, 27, 27, 27, 28, 28, 29, 29, 29,
   30, 30, 31, 31, 32, 32, 33, 33, 33, 34, 34, 35, 35, 36, 36, 37, 38, 38, 39, 39, 40, 40, 41, 42, 42, 43, 43, 44,
    45, 46, 46, 47, 48, 49, 50, 51, 52, 53, 54, 56, 58, 59};

//uint8_t ui8_adc_battery_current_acc =0;
uint8_t ui8_adc_battery_current_filtered = 0;
uint8_t ui8_adc_motor_phase_current = 0;


uint16_t ui16_PWM_cycles_counter = 1;
uint16_t ui16_PWM_cycles_counter_6 = 1;
uint16_t ui16_PWM_cycles_counter_total = 0xffff;

uint16_t ui16_max_motor_speed_erps = (uint16_t) MOTOR_OVER_SPEED_ERPS;
static volatile uint16_t ui16_motor_speed_erps = 0;
//uint8_t ui8_svm_table_index = 0;
uint8_t ui8_motor_rotor_absolute_angle;
uint8_t ui8_motor_rotor_angle;

volatile uint8_t ui8_g_foc_angle = 0;
uint8_t ui8_interpolation_angle = 0;
uint16_t ui16_foc_angle_accumulated = 0;

uint8_t ui8_motor_commutation_type = BLOCK_COMMUTATION;

volatile uint8_t ui8_g_hall_sensors_state = 0;
//uint8_t ui8_hall_sensors_state_last = 0;

uint8_t ui8_half_erps_flag = 0;

volatile uint8_t ui8_g_duty_cycle = 0;
static volatile uint8_t ui8_m_duty_cycle_target;
uint16_t ui16_duty_cycle_ramp_up_inverse_step;
uint16_t ui16_duty_cycle_ramp_down_inverse_step;
uint16_t ui16_counter_duty_cycle_ramp_up = 0;
uint16_t ui16_counter_duty_cycle_ramp_down = 0;

volatile uint8_t ui8_g_field_weakening_angle = 0;
volatile uint8_t ui8_g_field_weakening_enable = 0;
volatile uint8_t ui8_g_field_weakening_enable_state = 0;
uint16_t ui16_counter_field_weakening_ramp_up = 0;
uint16_t ui16_counter_field_weakening_ramp_down = 0;

uint16_t ui16_value;

uint16_t ui16_counter_adc_current_ramp_up = 0;
uint16_t ui16_controller_adc_max_current = 0;

uint8_t ui8_first_time_run_flag = 1;

volatile uint16_t ui16_main_loop_wdt_cnt_1 = 0;

volatile uint8_t ui8_adc_battery_voltage_cut_off = 0xff; // safe value so controller will not discharge the battery if not receiving a lower value from the LCD
uint16_t ui16_adc_battery_voltage_accumulated = 0;
uint16_t ui16_adc_battery_voltage_filtered_10b;

uint16_t ui16_adc_battery_current_accumulated = 0;
volatile uint16_t ui16_g_adc_battery_current_filtered;
uint16_t ui16_adc_motor_current_accumulated = 0;
volatile uint16_t ui16_g_adc_motor_current_filtered;

volatile uint16_t ui16_g_adc_battery_current;
volatile uint16_t ui16_g_adc_motor_current;
uint8_t ui8_current_controller_counter = 0;
uint16_t ui16_motor_speed_controller_counter = 0;

volatile uint16_t ui16_g_adc_target_battery_max_current;
volatile uint16_t ui16_g_adc_target_battery_max_current_fw;

volatile uint16_t ui16_g_adc_target_motor_max_current;
volatile uint16_t ui16_g_adc_target_motor_max_current_fw;

static uint8_t ui8_m_pas_state;
static uint8_t ui8_m_pas_state_old;
static uint8_t ui8_m_pas_after_first_pulse = 0;
static uint16_t ui16_m_pas_counter = (uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
volatile uint8_t ui8_g_pas_tick_counter = 0;

volatile uint8_t ui8_g_pas_pedal_right = 0;
uint8_t ui8_m_pedaling_direction = 0;

static uint8_t ui8_m_pas_min_cadence_flag = 0;
static uint16_t ui16_m_pas_min_cadence_pwm_cycles_ticks = 0;

// wheel speed
uint8_t ui8_wheel_speed_sensor_state = 1;
uint8_t ui8_wheel_speed_sensor_state_old = 1;
uint16_t ui16_wheel_speed_sensor_counter = 0;
uint8_t ui8_wheel_speed_sensor_change_counter = 0;

void read_battery_voltage(void);
void read_battery_current(void);
void read_motor_current(void);
void calc_foc_angle(void);
uint8_t asin_table(uint8_t ui8_inverted_angle_x128);

volatile uint8_t  ui8_hall_state_irq = 0;
volatile uint8_t  ui8_hall_60_ref_irq[2];

// Last rotor complete revolution Hall ticks
static uint16_t ui16_hall_360_ref;

// Last Hall sensor state
static uint8_t  ui8_hall_sensors_state_last = 7; // Invalid value, force execution of Hall code at the first run

// Hall counter value of last Hall transition
static uint16_t ui16_hall_60_ref_old;

// Hall Timer counter value calculated for the 6 different Hall transitions intervals
volatile uint16_t ui16_hall_calib_cnt[6];

// phase angle for rotor positions 30, 90, 150, 210, 270, 330 degrees
volatile uint8_t ui8_hall_ref_angles[6] = {
		PHASE_ROTOR_ANGLE_30,
		PHASE_ROTOR_ANGLE_90,
		PHASE_ROTOR_ANGLE_150,
		PHASE_ROTOR_ANGLE_210,
		PHASE_ROTOR_ANGLE_270,
		PHASE_ROTOR_ANGLE_330};

// Hall counter offset for states 6,2,3,1,5,4 (value configured from Android App)
volatile uint8_t ui8_hall_counter_offsets[6] = {
        HALL_COUNTER_OFFSET_UP,
        HALL_COUNTER_OFFSET_DOWN,
        HALL_COUNTER_OFFSET_UP,
        HALL_COUNTER_OFFSET_DOWN,
        HALL_COUNTER_OFFSET_UP,
        HALL_COUNTER_OFFSET_DOWN};

// Hall offset for current Hall state
static uint8_t ui8_hall_counter_offset;

// temporay variables (at the end of down irq stores phase a,b,c voltages)
static uint16_t ui16_a;
static uint16_t ui16_b;
static uint16_t ui16_c;

static uint8_t ui8_temp = 0;

// motor variables
uint8_t ui8_hall_360_ref_valid = 0;
static uint8_t ui8_motor_phase_absolute_angle;
volatile uint16_t ui16_hall_counter_total = 0xffff;
// volatile uint16_t ui16_motor_speed_erps = 0;


void motor_controller(void)
{
      if (((uint8_t)(ui16_hall_counter_total>>8)) & 0x80)
        ui16_motor_speed_erps = 0;
    else
        // Reduce operands to 16 bit (Avoid slow _divulong() library function)
        ui16_motor_speed_erps = (uint16_t)(HALL_COUNTER_FREQ >> 2) / (uint16_t)(ui16_hall_counter_total >> 2);
  read_battery_voltage();
  read_battery_current();
  read_motor_current();
  calc_foc_angle();
}

// PWM cycle interrupt
// TIM1 clock is 16MHz and count mode is "Center Aligned"
// Every cycle TIM1 counts up from 0 to 420 and then down from 420 to 0 (26.25+26.25us = 52.5us total time)
// The interrupt fires two times every cycle in the middle of the counter (when reaches 210 up and down)
// ADC conversion is automatically started by the rising edge of TRGO signal which is aligned with the Down interrupt signal.
// Both interrupts are used to read HAL sensors and update rotor position counters (max 26us rotor position offset error)
// and then:
// Up interrupt is used for:
//  - read and filter battery current
//  - read PAS sensor and cadence computation
//  - check brake (coaster brake and brake input signal)
//  - update duty cycle
// Down interrupt is used for:
//  - calculate rotor position (based on HAL sensors state and interpolation based on counters)
//  - Apply phase voltage and duty cycle to TIM1 outputs according to rotor position
//  - Read Wheel speed sensor and wheel speed computation

#ifdef __CDT_PARSER__
#define __interrupt(x)  // Disable Eclipse syntax check on interrupt keyword
#endif

// Interrupt routines called on Hall sensor state change (Highest priority)
// - read the Hall transition reference counter value (ui8_hall_60_ref_irq)
// - read the hall signal state (ui8_hall_state_irq)
//      - Hall A: bit 0
//      - Hall B: bit 1
//      - Hall C: bit 2
void HALL_SENSOR_A_PORT_IRQHandler(void)  __interrupt(EXTI_HALL_A_IRQ) {
    ui8_hall_60_ref_irq[0] = TIM3->CNTRH;
    ui8_hall_60_ref_irq[1] = TIM3->CNTRL;
    ui8_hall_state_irq &= (unsigned char)~0x01;
    if (HALL_SENSOR_A__PORT->IDR & HALL_SENSOR_A__PIN)
        ui8_hall_state_irq |= (unsigned char)0x01;
}

void HALL_SENSOR_B_PORT_IRQHandler(void) __interrupt(EXTI_HALL_B_IRQ)  {
    ui8_hall_60_ref_irq[0] = TIM3->CNTRH;
    ui8_hall_60_ref_irq[1] = TIM3->CNTRL;
    ui8_hall_state_irq &= (unsigned char)~0x02;
    if (HALL_SENSOR_B__PORT->IDR & HALL_SENSOR_B__PIN)
        ui8_hall_state_irq |= (unsigned char)0x02;
}

void HALL_SENSOR_C_PORT_IRQHandler(void) __interrupt(EXTI_HALL_C_IRQ)  {
    ui8_hall_60_ref_irq[0] = TIM3->CNTRH;
    ui8_hall_60_ref_irq[1] = TIM3->CNTRL;
    ui8_hall_state_irq &= (unsigned char)~0x04;
    if (HALL_SENSOR_C__PORT->IDR & HALL_SENSOR_C__PIN)
        ui8_hall_state_irq |= (unsigned char)0x04;
}

void TIM1_CAP_COM_IRQHandler(void) __interrupt(TIM1_CAP_COM_IRQHANDLER)
{
      // bit 5 of TIM1->CR1 contains counter direction (0=up, 1=down)
  if (TIM1->CR1 & 0x10) 
  {
    #ifndef __CDT_PARSER__ // disable Eclipse syntax check
    __asm
        push cc             // save current Interrupt Mask (I1,I0 bits of CC register)
        sim                 // disable interrupts  (set I0,I1 bits of CC register to 1,1)
                            // Hall GPIO interrupt is buffered during this interval
        mov _ui8_temp+0, _ui8_hall_state_irq+0
        mov _ui16_b+0, _ui8_hall_60_ref_irq+0
        mov _ui16_b+1, _ui8_hall_60_ref_irq+1
        pop cc              // enable interrupts (restores previous value of Interrupt mask)
                            // Hall GPIO buffered interrupt could fire now
        mov _ui16_a+0, 0x5328 // TIM3->CNTRH
        mov _ui16_a+1, 0x5329 // TIM3->CNTRL
    __endasm;
    #endif
      // ui8_temp stores the current Hall sensor state (was ui8_g_hall_sensors_state)
      // ui16_b stores the Hall sensor counter value of the last transition
      // ui16_a stores the current Hall sensor counter value

      /****************************************************************************/
      // run next code only when the hall state changes
      // hall sensors sequence with motor forward rotation: C, CB, B, BA, A, AC, ..
      // ui8_temp (hall sensor state):
      //      bit 0 0x01 Hall sensor A
      //      bit 1 0x02 Hall sensor B
      //      bit 2 0x04 Hall sensor C
      // ui8_hall_sensors_state sequence with motor forward rotation: 0x06, 0x02, 0x03, 0x01, 0x05, 0x04
      //                                              rotor position:  30,   90,   150,  210,  270,  330 degrees
      if (ui8_hall_sensors_state_last != ui8_temp) {
          // Check first the state with the heaviest computation
          if (ui8_temp == 0x01) {
              // if (ui8_hall_360_ref_valid && (ui8_hall_sensors_state_last == 0x03)) {
              if (ui8_hall_sensors_state_last == ui8_hall_360_ref_valid) { // faster check
                  ui16_hall_counter_total = ui16_b - ui16_hall_360_ref;
                  ui8_motor_commutation_type = SINEWAVE_INTERPOLATION_60_DEGREES;
              }
              ui8_hall_360_ref_valid = 0x03;
              ui8_motor_phase_absolute_angle = ui8_hall_ref_angles[3]; // Rotor at 210 deg
              // set hall counter offset for rotor interpolation based on current hall state
              ui8_hall_counter_offset = ui8_hall_counter_offsets[3];
              ui16_hall_360_ref = ui16_b;
              // calculate hall ticks between the last two Hall transitions (for Hall calibration)
              ui16_hall_calib_cnt[3] = ui16_hall_360_ref - ui16_hall_60_ref_old;
          } else
              switch (ui8_temp) {
                  case 0x02:
                      ui8_motor_phase_absolute_angle = ui8_hall_ref_angles[1]; // Rotor at 90 deg
                      // set hall counter offset for rotor interpolation based on current hall state
                      ui8_hall_counter_offset = ui8_hall_counter_offsets[1];
                      // calculate hall ticks between the last two Hall transitions (for Hall calibration)
                      ui16_hall_calib_cnt[1] = ui16_b - ui16_hall_60_ref_old;
                      break;
                  case 0x03:
                      ui8_motor_phase_absolute_angle = ui8_hall_ref_angles[2]; // Rotor at 150 deg
                      ui8_hall_counter_offset = ui8_hall_counter_offsets[2];
                      ui16_hall_calib_cnt[2] = ui16_b - ui16_hall_60_ref_old;
                      break;
                  case 0x04:
                      ui8_motor_phase_absolute_angle = ui8_hall_ref_angles[5]; // Rotor at 330 deg
                      ui8_hall_counter_offset = ui8_hall_counter_offsets[5];
                      ui16_hall_calib_cnt[5] = ui16_b - ui16_hall_60_ref_old;
                      break;
                  case 0x05:
                      ui8_motor_phase_absolute_angle = ui8_hall_ref_angles[4]; // Rotor at 270 deg
                      ui8_hall_counter_offset = ui8_hall_counter_offsets[4];
                      ui16_hall_calib_cnt[4] = ui16_b - ui16_hall_60_ref_old;
                      break;
                  case 0x06:
                      ui8_motor_phase_absolute_angle = ui8_hall_ref_angles[0]; // Rotor at 30 deg
                      ui8_hall_counter_offset = ui8_hall_counter_offsets[0];
                      ui16_hall_calib_cnt[0] = ui16_b - ui16_hall_60_ref_old;
                      break;
                  default:
                      return;
              }

          // update last hall sensor state
          #ifndef __CDT_PARSER__ // disable Eclipse syntax check
          __asm
              // speed optimization ldw, ldw -> mov,mov
              // ui16_hall_60_ref_old = ui16_b;
              mov _ui16_hall_60_ref_old+0, _ui16_b+0
              mov _ui16_hall_60_ref_old+1, _ui16_b+1
          __endasm;
          #endif
          ui8_hall_sensors_state_last = ui8_temp;
      } else {
          // Verify if rotor stopped (< 10 ERPS)
          // ui16_a - ui16_b = Hall counter ticks from the last Hall sensor transition;
          if ((ui16_a - ui16_b) > (HALL_COUNTER_FREQ/MOTOR_ROTOR_INTERPOLATION_MIN_ERPS/6)) {
              ui8_motor_commutation_type = BLOCK_COMMUTATION;
              ui8_g_foc_angle = 0;
              ui8_hall_360_ref_valid = 0;
              ui16_hall_counter_total = 0xffff;
          }
      }


      /****************************************************************************/
      // - calculate interpolation angle and sine wave table index

      /*
      ui8_temp = 0; // interpolation angle
      if (ui8_motor_commutation_type != BLOCK_COMMUTATION) {
          // ---------
          // uint8_t ui8_temp = ((uint32_t)ui16_a << 8) / ui16_hall_counter_total;
          // ---------
          // Avoid to use the slow _divulong library function.
          // Faster implementation of the above operation based on the following assumptions:
          // 1) ui16_a < 8192 (only 13 of 16 significants bits)
          // 2) LSB of (ui16_a << 8) is obviously 0x00
          // 3) The result to should be less than 64 (90 degrees)
          uint8_t ui8_cnt = 6; //max 6 loops: result < 64
          // Add Field Weakening counter offset (fw angle increases with rotor speed)
          // ui16_a - ui16_b = Hall counter ticks from the last Hall sensor transition;
          ui16_a = ((uint8_t)(ui8_fw_hall_counter_offset + ui8_hall_counter_offset) + (ui16_a - ui16_b)) << 2;

          do {
              ui16_a <<= 1;
              ui8_temp <<= 1;
              if (ui16_hall_counter_total <= ui16_a) {
                  ui16_a -= ui16_hall_counter_total;
                  ui8_temp |= (uint8_t)0x01;
              }
          } while (--ui8_cnt);
      }
      // we need to put phase voltage 90 degrees ahead of rotor position, to get current 90 degrees ahead and have max torque per amp
      ui8_svm_table_index = ui8_temp + ui8_motor_phase_absolute_angle + ui8_g_foc_angle;
      */
      #ifndef __CDT_PARSER__ // disable Eclipse syntax check
      __asm
          clr _ui8_temp+0
          tnz _ui8_motor_commutation_type+0
          jreq 00011$
          // ui16_a = ((ui16_a - ui16_b) + ui8_fw_hall_counter_offset + ui8_hall_counter_offset) << 2;
          ld  a, _ui8_g_field_weakening_angle+0 //_ui8_fw_hall_counter_offset+0
          add a, _ui8_hall_counter_offset+0
          clrw    x
          ld  xl, a
          addw    x, _ui16_a+0
          subw    x, _ui16_b+0
          sllw x
          sllw x
          mov _ui16_b+0, #6
      00012$:
          sllw x
          sll  _ui8_temp+0
          cpw x, _ui16_hall_counter_total+0
          jrc  00013$
          bset    _ui8_temp+0, #0
          subw x, _ui16_hall_counter_total+0
      00013$:
          dec _ui16_b+0
          jrne 00012$
          // now ui8_temp contains the interpolation angle
      00011$: // BLOCK_COMMUTATION
          // ui8_temp = ui8_temp + ui8_motor_phase_absolute_angle + ui8_g_foc_angle;
          ld  a, _ui8_temp+0
          add a, _ui8_motor_phase_absolute_angle+0
          add a, _ui8_g_foc_angle+0
          ld _ui8_temp, a

      // now ui8_temp contains ui8_svm_table_index

      /****************************************************************************/
      // calculate final PWM duty_cycle values to be applied to TIMER1
      // scale and apply PWM duty_cycle for the 3 phases
      // phase A is advanced 240 degrees over phase B
      // Max of SVM table is 202 and ui8_tmp goes from 0 to 100 (101*254/256) and
      // ui8_phase_x_voltage goes from 0 (MIDDLE_PWM_COUNTER - ui8_temp) to 200 (MIDDLE_PWM_COUNTER + ui8_temp)

      /*
      // Phase A is advanced 240 degrees over phase B
      ui8_temp = ui8_svm_table[(uint8_t) (ui8_svm_table_index + 171)]; // 240 deg
      if (ui8_temp > MIDDLE_SVM_TABLE) {
          ui16_a = (uint16_t)((uint8_t)(ui8_temp - MIDDLE_SVM_TABLE) * (uint8_t)ui8_g_duty_cycle);
          ui16_a = (uint8_t)(MIDDLE_PWM_COUNTER + (uint8_t) (ui16_a >> 8)) << 1;
      } else {
          ui16_a = (uint16_t)((uint8_t)(MIDDLE_SVM_TABLE - ui8_temp) * (uint8_t)ui8_g_duty_cycle);
          ui16_a = (uint8_t)(MIDDLE_PWM_COUNTER - (uint8_t) (ui16_a >> 8)) << 1;
      }
      */

          // ui8_temp = ui8_svm_table[(uint8_t) (ui8_svm_table_index + 171)];
          add a, #0xab
          clrw x
          ld  xl, a
          ld  a, (_ui8_svm_table+0, x)
          cp  a, #MIDDLE_SVM_TABLE    // if (ui8_temp > MIDDLE_SVM_TABLE)
          jrule   00020$
          // ui16_a = (uint16_t)((uint8_t)(ui8_temp - MIDDLE_SVM_TABLE) * (uint8_t)ui8_g_duty_cycle);
          sub a, #MIDDLE_SVM_TABLE
          ld  xl, a
          ld  a, _ui8_g_duty_cycle+0
          mul x, a
          // ui16_a = (uint8_t)(MIDDLE_PWM_COUNTER + (uint8_t) (ui16_a >> 8)) << 1;
          ld  a, xh
          clr _ui16_a+0
          add a, #MIDDLE_PWM_COUNTER
          jrpl 00022$
          mov _ui16_a+0, #0x01  // result is negative (bit 7 is set)
      00022$:
          sll a
          ld  _ui16_a+1, a
          jra 00021$
      00020$:             // } else {
          // ui16_a = (uint16_t)((uint8_t)(MIDDLE_SVM_TABLE - ui8_temp) * (uint8_t)ui8_g_duty_cycle);
          ld  _ui16_a+1, a
          ld  a, #MIDDLE_SVM_TABLE
          sub a, _ui16_a+1
          ld  xl, a
          ld  a, _ui8_g_duty_cycle+0
          mul x, a
          // ui16_a = (uint8_t)(MIDDLE_PWM_COUNTER - (uint8_t) (ui16_a >> 8)) << 1;
          ld  a, xh
          ld  _ui16_a+1, a
          ld  a, #MIDDLE_PWM_COUNTER
          clr _ui16_a+0
          sub a, _ui16_a+1
          jrpl 00023$
          mov _ui16_a+0, #0x01
      00023$:
          sll a
          ld  _ui16_a+1, a
      00021$:

      /*
      // phase B as reference phase
      ui8_temp = ui8_svm_table[ui8_svm_table_index];
      if (ui8_temp > MIDDLE_SVM_TABLE) {
          ui16_b = (uint16_t) ((uint8_t)(ui8_temp - MIDDLE_SVM_TABLE) * (uint8_t)ui8_g_duty_cycle);
          ui16_b = (uint8_t)(MIDDLE_PWM_COUNTER + (uint8_t) (ui16_b >> 8)) << 1;
      } else {
          ui16_b = (uint16_t) ((uint8_t)(MIDDLE_SVM_TABLE - ui8_temp) * (uint8_t)ui8_g_duty_cycle);
          ui16_b = (uint8_t)(MIDDLE_PWM_COUNTER - (uint8_t)(ui16_b >> 8)) << 1;
      }
      */

          ld a, _ui8_temp+0   // ui8_svm_table_index is stored in ui8_temp
          clrw x              // ui8_temp = ui8_svm_table[ui8_svm_table_index];
          ld  xl, a
          ld  a, (_ui8_svm_table+0, x)
          cp  a, #MIDDLE_SVM_TABLE    // if (ui8_temp > MIDDLE_SVM_TABLE)
          jrule   00024$
          // ui16_b = (uint16_t)((uint8_t)(ui8_temp - MIDDLE_SVM_TABLE) * (uint8_t)ui8_g_duty_cycle);
          sub a, #MIDDLE_SVM_TABLE
          ld  xl, a
          ld  a, _ui8_g_duty_cycle+0
          mul x, a
          // ui16_b = (uint8_t)(MIDDLE_PWM_COUNTER + (uint8_t)(ui16_b >> 8)) << 1;
          ld  a, xh
          clr _ui16_b+0
          add a, #MIDDLE_PWM_COUNTER
          jrpl 00026$
          mov _ui16_b+0, #0x01
      00026$:
          sll a
          ld  _ui16_b+1, a
          jra 00025$
      00024$:             // } else {
          // ui16_b = (uint16_t)((uint8_t)(MIDDLE_SVM_TABLE - ui8_temp) * (uint8_t)ui8_g_duty_cycle);
          ld  _ui16_b+1, a
          ld  a, #MIDDLE_SVM_TABLE
          sub a, _ui16_b+1
          ld  xl, a
          ld  a, _ui8_g_duty_cycle+0
          mul x, a
          // ui16_b = (uint8_t)(MIDDLE_PWM_COUNTER - (uint8_t) (ui16_b >> 8)) << 1;
          ld  a, xh
          ld  _ui16_b+1, a
          ld  a, #MIDDLE_PWM_COUNTER
          clr _ui16_b+0
          sub a, _ui16_b+1
          jrpl 00027$
          mov _ui16_b+0, #0x01
      00027$:
          sll a
          ld  _ui16_b+1, a
      00025$:

      /*
      // phase C is advanced 120 degrees over phase B
      ui8_temp = ui8_svm_table[(uint8_t) (ui8_svm_table_index + 85 )]; // 120 deg
      if (ui8_temp > MIDDLE_SVM_TABLE) {
          ui16_c = (uint16_t) ((uint8_t)(ui8_temp - MIDDLE_SVM_TABLE) * (uint8_t)ui8_g_duty_cycle);
          ui16_c = (uint8_t)(MIDDLE_PWM_COUNTER + (uint8_t) (ui16_c >> 8)) << 1;
      } else {
          ui16_c = (uint16_t) ((uint8_t)(MIDDLE_SVM_TABLE - ui8_temp) * (uint8_t)ui8_g_duty_cycle);
          ui16_c = (uint8_t)(MIDDLE_PWM_COUNTER - (uint8_t) (ui16_c >> 8)) << 1;
      }
      */

          ld a, _ui8_temp+0     // ui8_svm_table_index is stored in ui8_temp
          add a, #0x55        // ui8_temp = ui8_svm_table[(uint8_t) (ui8_svm_table_index + 85 /* 120º */)];
          clrw x
          ld  xl, a
          ld  a, (_ui8_svm_table+0, x)
          cp  a, #MIDDLE_SVM_TABLE    // if (ui8_temp > MIDDLE_SVM_TABLE)
          jrule   00028$
          // ui16_c = (uint16_t)((uint8_t)(ui8_temp - MIDDLE_SVM_TABLE) * (uint8_t)ui8_g_duty_cycle);
          sub a, #MIDDLE_SVM_TABLE
          ld  xl, a
          ld  a, _ui8_g_duty_cycle+0
          mul x, a
          // ui16_c = (uint8_t)(MIDDLE_PWM_COUNTER + (uint8_t)(ui16_c >> 8)) << 1;
          ld  a, xh
          clr _ui16_c+0
          add a, #MIDDLE_PWM_COUNTER
          jrpl 00030$
          mov _ui16_c+0, #0x01
      00030$:
          sll a
          ld  _ui16_c+1, a
          jra 00029$
      00028$:             // } else {
          // ui16_c = (uint16_t)((uint8_t)(MIDDLE_SVM_TABLE - ui8_temp) * (uint8_t)ui8_g_duty_cycle);
          ld  _ui16_c+1, a
          ld  a, #MIDDLE_SVM_TABLE
          sub a, _ui16_c+1
          ld  xl, a
          ld  a, _ui8_g_duty_cycle+0
          mul x, a
          // ui16_c = (uint8_t)(MIDDLE_PWM_COUNTER - (uint8_t) (ui16_c >> 8)) << 1;
          ld  a, xh
          ld  _ui16_c+1, a
          ld  a, #MIDDLE_PWM_COUNTER
          clr _ui16_c+0
          sub a, _ui16_c+1
          jrpl 00031$
          mov _ui16_c+0, #0x01
      00031$:
          sll a
          ld  _ui16_c+1, a
      00029$:
      __endasm;
      #endif

      #ifdef PWM_TIME_DEBUG
          #ifndef __CDT_PARSER__ // avoid Eclipse syntax check
          __asm
              ld  a, 0x5250
              and a, #0x10 // counter direction end irq
              or  a, 0x525e // TIM1->CNTRH
              ld  _ui16_pwm_cnt_down_irq+0, a      // ui16_pwm_cnt_down_irq MSB = TIM1->CNTRH | direction
              mov _ui16_pwm_cnt_down_irq+1, 0x525f // ui16_pwm_cnt_down_irq LSB = TIM1->CNTRL
          __endasm;
          #endif
      #endif

    }
    else // Up side of the interrupt ******************************************************************************
    {
      // CRITICAL SECTION !
      // Disable GPIO Hall interrupt during PWM counter update
      // The whole update is completed in 9 CPU cycles
      // set final duty_cycle value
      /*
      // phase B
      TIM1->CCR3H = (uint8_t)(ui16_b >> 8);
      TIM1->CCR3L = (uint8_t)(ui16_b);
      // phase C
      TIM1->CCR2H = (uint8_t)(ui16_c >> 8);
      TIM1->CCR2L = (uint8_t)(ui16_c);
      // phase A
      TIM1->CCR1H = (uint8_t)(ui16_a >> 8);
      TIM1->CCR1L = (uint8_t)(ui16_a);
      */
      #ifndef __CDT_PARSER__ // avoid Eclipse syntax check
      __asm
      push cc             // save current Interrupt Mask (I1,I0 bits of CC register)
      sim                 // disable interrupts  (set I0,I1 bits of CC register to 1,1)
                          // Hall GPIO interrupt is buffered during this interval
      mov 0x5269, _ui16_b+0
      mov 0x526a, _ui16_b+1
      mov 0x5267, _ui16_c+0
      mov 0x5268, _ui16_c+1
      mov 0x5265, _ui16_a+0
      mov 0x5266, _ui16_a+1
      pop cc           // enable interrupts (restores previous value of Interrupt mask)
                        // Hall GPIO buffered interrupt could fire now
      __endasm;
      #endif


      //     /********************* *******************************************************/
      //     /*
      //     // Read battery current
      //     // Left alignment: Read MSB first then read LSB !
      //     ui8_temp = ADC1->DB5RH;
      //     ui8_temp <<= 2;
      //     ui8_temp |= ADC1->DB5RL
      //     ui8_adc_battery_current_acc >>= 1;
      //     ui8_adc_battery_current_filtered >>= 1;
      //     ui8_adc_battery_current_acc = (uint8_t)(ui8_temp >> 1) + ui8_adc_battery_current_acc;
      //     ui8_adc_battery_current_filtered = (uint8_t)(ui8_adc_battery_current_acc >> 1) + ui8_adc_battery_current_filtered;

      //     // calculate motor phase current ADC value
      //     if (ui8_g_duty_cycle > 0)
      //         ui8_adc_motor_phase_current = (uint16_t)((uint16_t)((uint16_t)ui8_adc_battery_current_filtered << 6)) / ui8_g_duty_cycle;
      //     else
      //         ui8_adc_motor_phase_current = 0;

      //     // clear EOC flag (and select channel 7)
      //     ADC1->CSR = 0x07;
      //     */
          #ifndef __CDT_PARSER__ // avoid Eclipse syntax check
          __asm
          ld  a, 0x53EA                               // ui8_temp = ADC1->DB5RH;
          sll a                                       // ui8_temp <<= 2;
          sll a
          or  a, 0x53EB                               // ui8_temp |= ADC1->DB5RL;
          ld  _ui8_adc_battery_current_filtered+0, a
          // srl _ui8_adc_battery_current_acc+0          // ui8_adc_battery_current_acc >>= 1;
          // srl a                                       // ui8_adc_battery_current_acc = (uint8_t)(ui8_temp >> 1) + ui8_adc_battery_current_acc;
          // add a, _ui8_adc_battery_current_acc+0
          // ld  _ui8_adc_battery_current_acc+0, a
          // srl _ui8_adc_battery_current_filtered+0     // ui8_adc_battery_current_filtered >>= 1;
          // srl a                                       // ui8_adc_battery_current_filtered = (uint8_t)(ui8_adc_battery_current_acc >> 1) + ui8_adc_battery_current_filtered;
          // add a, _ui8_adc_battery_current_filtered+0
          // ld  _ui8_adc_battery_current_filtered+0, a

          tnz _ui8_g_duty_cycle+0                     // if (ui8_g_duty_cycle > 0)
          jreq 00051$
          clrw    x          // ui8_adc_motor_phase_current = (ui8_adc_battery_current_filtered << 6)) / ui8_g_duty_cycle;
          ld  xh, a
          srlw    x
          srlw    x
          ld  a, _ui8_g_duty_cycle+0
          div    x, a
          ld  a, xl
          ld  _ui8_adc_motor_phase_current+0, a
          jra 00052$
      00051$:
          clr _ui8_adc_motor_phase_current+0      // ui8_adc_motor_phase_current = 0;
      00052$:
          mov 0x5400+0, #0x07                     // ADC1->CSR = 0x07;
          __endasm;
          #endif

          ui16_g_adc_motor_current = ui8_adc_motor_phase_current;
          ui16_g_adc_battery_current = ui8_adc_battery_current_filtered;
      
    // // /****************************************************************************/
    // // // read battery current ADC value | should happen at middle of the PWM duty_cycle
    // // // disable scan mode
    // // ADC1->CR2 &= (uint8_t)(~ADC1_CR2_SCAN);
    
    // // // clear EOC flag first (selected also channel 5)
    // // ADC1->CSR = 0x05;
    // // //
    // // // start ADC1 conversion
    // // ADC1->CR1 |= ADC1_CR1_ADON;
    // // while (!(ADC1->CSR & ADC1_FLAG_EOC)) ;
    // ui16_g_adc_battery_current = UI16_ADC_10_BIT_BATTERY_CURRENT;

    // // we ignore low values of the battery current < 5 to avoid issues with other consumers than the motor (such as integrated 6v lights)
    // // Piecewise linear is better than a step, to avoid limit cycles.
    // // in     --> out
    // // 0 -  5 --> 0 - 0
    // // 5 - 15 --> 0 - 15
    // if (ui16_g_adc_battery_current <= 5)
    // {
    //   ui16_g_adc_battery_current = 0;
    // }  
    // else if (ui16_g_adc_battery_current <= 15)
    // {
    //   ui16_g_adc_battery_current -= 5; // 5 - 15 --> 0 - 10
    //   ui16_g_adc_battery_current += (ui16_g_adc_battery_current >> 1); // multiply by 1.5: 0 - 10 --> 0 - 15
    // }
      
    // // this shoud work but does not.......
    // //  ui16_g_adc_battery_current = (((uint16_t) ADC1->DRH) << 8) | ((uint16_t) ADC1->DRL);

    // // calculate motor current ADC value
    // if (ui8_g_duty_cycle > 0)
    // {
    //   ui16_g_adc_motor_current = ((ui16_g_adc_battery_current << 8) / ((uint16_t) ui8_g_duty_cycle));
    // }
    // else
    // {
    //   ui16_g_adc_motor_current = 0;
    // }

    // // /****************************************************************************/
    // // // trigger ADC conversion of all channels (scan conversion, buffered)
    // //ADC1->CR2 |= ADC1_CR2_SCAN; // enable scan mode
    // ADC1->CSR = 0x07; // clear EOC flag first (selected also channel 7)
    // //ADC1->CR1 |= ADC1_CR1_ADON; // start ADC1 conversion

    /****************************************************************************/
    // check brakes state

    // if brake sensors are active
    ui8_g_brakes_state = ((BRAKE__PORT->IDR & (uint8_t)BRAKE__PIN) == 0);

    if (ui8_g_coast_brake_enable)
    {
      // check if coaster brake is engaged
      if (UI16_ADC_10_BIT_TORQUE_SENSOR < (ui16_g_adc_torque_sensor_min_value - ((uint16_t) ui8_g_adc_coast_brake_torque_threshold)))
      {
        ui8_g_brakes_state = 1;
      }
    }
    /****************************************************************************/
    
    /****************************************************************************/
    // PWM duty_cycle controller:
    // - brakes are active
    // - limit battery undervoltage
    // - limit battery max current
    // - limit motor phase max current
    // - limit motor max phase current
    // - limit motor max ERPS
    // - ramp up/down PWM duty_cycle value

    // check to enable field weakening state
    if (ui8_g_field_weakening_enable &&
        (ui16_motor_speed_erps > MOTOR_SPEED_FIELD_WEAKEANING_MIN)) // do not enable at low motor speed / low cadence
      ui8_g_field_weakening_enable_state = 1;

    ++ui8_current_controller_counter;
    ++ui16_motor_speed_controller_counter;

    if (ui8_g_brakes_state ||
        (ui8_m_pas_min_cadence_flag && (ui8_g_throttle == 0)) ||
        (UI8_ADC_BATTERY_VOLTAGE < ui8_adc_battery_voltage_cut_off))
    {
      if (ui8_g_field_weakening_angle)
      {
        --ui8_g_field_weakening_angle;
      }
      else if (ui8_g_duty_cycle)
      {
        --ui8_g_duty_cycle;
      }
    }
    // do not control current at every PWM cycle, that will measure and control too fast. Use counter to limit
    else if ((ui8_current_controller_counter > 14) &&
        ((ui16_g_adc_battery_current > ui16_g_adc_target_battery_max_current) ||
        (ui16_g_adc_motor_current > ui16_controller_adc_max_current)))
    {
      ui8_debug_data_9 = 1;
      if (ui8_g_field_weakening_angle)
      {
        --ui8_g_field_weakening_angle;
      }
      else if (ui8_g_duty_cycle)
      {
        --ui8_g_duty_cycle;
      }
    }
    else if ((ui16_motor_speed_controller_counter > 2000) && // test about every 100ms
        (ui16_motor_speed_erps > ui16_max_motor_speed_erps))
    {
      ui8_debug_data_9 = 2;
      if (ui8_g_field_weakening_angle)
      {
        --ui8_g_field_weakening_angle;
      }
      else if (ui8_g_duty_cycle)
      {
        --ui8_g_duty_cycle;
      }
    }
    else // nothing to limit, so adjust duty_cycle to duty_cycle_target, including ramping
        // or adjust field weakening
    {
      if ((ui8_g_duty_cycle >= PWM_DUTY_CYCLE_MAX) && // max voltage already applied to motor windings, enter or keep in field weakening state
          ui8_g_field_weakening_enable_state)
      {
        if (ui16_g_adc_motor_current < ui16_controller_adc_max_current)
        {
          ui8_debug_data_9 = 3;
          if (ui16_counter_field_weakening_ramp_up++ >= FIELD_WEAKENING_RAMP_UP_INVERSE_STEP)
          {
            ui16_counter_field_weakening_ramp_up = 0;

            if (ui8_g_field_weakening_angle < FIELD_WEAKENING_ANGLE_MAX)
              ++ui8_g_field_weakening_angle;
          }
        }
        else if (ui16_g_adc_motor_current > ui16_controller_adc_max_current)
        {
          ui8_debug_data_9 = 4;
          if (ui16_counter_field_weakening_ramp_down++ >= FIELD_WEAKENING_RAMP_DOWN_INVERSE_STEP)
          {
            ui16_counter_field_weakening_ramp_down = 0;

            if (ui8_g_field_weakening_angle)
            {
              --ui8_g_field_weakening_angle;
            }
            else
            {
              --ui8_g_duty_cycle; // exit from field weakening state
            }
          }
        }
      }
      else
      {
        if (ui8_m_duty_cycle_target > ui8_g_duty_cycle)
        {
          ui8_debug_data_9 = 5;
          if (ui16_counter_duty_cycle_ramp_up++ >= ui16_duty_cycle_ramp_up_inverse_step)
          {
            ui16_counter_duty_cycle_ramp_up = 0;
            ++ui8_g_duty_cycle;
             
          }
        }
        else if (ui8_m_duty_cycle_target < ui8_g_duty_cycle)
        {
          ui8_debug_data_9 = 6;
          if (ui16_counter_duty_cycle_ramp_down++ >= ui16_duty_cycle_ramp_down_inverse_step)
          {
            ui16_counter_duty_cycle_ramp_down = 0;
            --ui8_g_duty_cycle;
          }
        }
      }
    }

    //ui8_svm_table_index += ui8_g_field_weakening_angle;

    // disable field weakening only after leaving the field weakening state
    if (ui8_g_field_weakening_enable == 0 &&
        ui8_g_duty_cycle < PWM_DUTY_CYCLE_MAX)
      ui8_g_field_weakening_enable_state = 0;

    if (ui8_current_controller_counter > 14)
      ui8_current_controller_counter = 0;

    if (ui16_motor_speed_controller_counter > 2000)
      ui16_motor_speed_controller_counter = 0;

    /****************************************************************************/
    // ramp up ADC battery current

    uint16_t ui16_adc_target_motor_max_current;
    
    // field weakening has a higher current value to provide the same torque
    if (ui8_g_field_weakening_enable_state)
      ui16_adc_target_motor_max_current = ui16_g_adc_target_motor_max_current_fw;
    else
      ui16_adc_target_motor_max_current = ui16_g_adc_target_motor_max_current;

    // now ramp up
    if (ui16_adc_target_motor_max_current > ui16_controller_adc_max_current)
    {
      if (ui16_counter_adc_current_ramp_up++ >= ui16_g_current_ramp_up_inverse_step)
      {
        ui16_counter_adc_current_ramp_up = 0;
        ui8_debug_data_10 = 1;
        ui16_controller_adc_max_current++;
      }
    }
    else if (ui16_adc_target_motor_max_current < ui16_controller_adc_max_current)
    {
      // we are not doing a ramp down here, just directly setting to the target value
      ui16_controller_adc_max_current = ui16_g_adc_target_motor_max_current;
      ui8_debug_data_10 = 2;
    }
    /****************************************************************************/

    // calc PAS timming between each positive pulses, in PWM cycles ticks
    // calc PAS on and off timming of each pulse, in PWM cycles ticks
    ui16_m_pas_counter++;

    // detect PAS signal changes
    if ((PAS1__PORT->IDR & PAS1__PIN) == 0)
      ui8_m_pas_state = 0;
    else
      ui8_m_pas_state = 1;

    // PAS signal did change
    if (ui8_m_pas_state != ui8_m_pas_state_old)
    {
      ui8_m_pas_state_old = ui8_m_pas_state;

      // consider only when PAS signal transition from 0 to 1
      if (ui8_m_pas_state == 1)
      {
        // keep track of first pulse
        if (!ui8_m_pas_after_first_pulse)
        {
          ui8_m_pas_after_first_pulse = 1;
          ui16_g_pas_pwm_cycles_ticks = (uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
        }
        else
        {
          // limit PAS cadence to be less than PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS
          if (ui16_m_pas_counter < ((uint16_t) PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS))
            ui16_g_pas_pwm_cycles_ticks = (uint16_t) PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS;
          else
            ui16_g_pas_pwm_cycles_ticks = ui16_m_pas_counter;

          if (ui8_g_pedal_cadence_fast_stop)
            ui16_m_pas_min_cadence_pwm_cycles_ticks = (ui16_g_pas_pwm_cycles_ticks + (ui16_g_pas_pwm_cycles_ticks >> 2));
          else
            ui16_m_pas_min_cadence_pwm_cycles_ticks = ui16_g_pas_pwm_cycles_ticks << 1;

          ui16_m_pas_counter = 0;

          // see the direction
          if ((PAS2__PORT->IDR & PAS2__PIN) == 0)
            ui8_m_pedaling_direction = 2;
          else
            ui8_m_pedaling_direction = 1;
        }

        // lef/right
        if ((PAS2__PORT->IDR & PAS2__PIN) == 0)
        {
          ui8_g_pas_tick_counter++;
          if (ui8_g_pas_tick_counter > PAS_NUMBER_MAGNETS_X2)
            ui8_g_pas_tick_counter = 1;
        }
        else
        {
          if (ui8_g_pas_tick_counter <= 1)
            ui8_g_pas_tick_counter = PAS_NUMBER_MAGNETS_X2;
          else
            ui8_g_pas_tick_counter--;
        }
      }
      else
      {
        // keep track of first pulse
        if (ui8_m_pas_after_first_pulse)
        {
          // see the direction
          if ((PAS2__PORT->IDR & PAS2__PIN) != 0)
            ui8_m_pedaling_direction = 2;
          else
            ui8_m_pedaling_direction = 1;
        }


        // lef/right
        if ((PAS2__PORT->IDR & PAS2__PIN) != 0)
        {
          ui8_g_pas_tick_counter++;
          if (ui8_g_pas_tick_counter > PAS_NUMBER_MAGNETS_X2)
            ui8_g_pas_tick_counter = 1;
        }
        else
        {
          if (ui8_g_pas_tick_counter <= 1)
            ui8_g_pas_tick_counter = PAS_NUMBER_MAGNETS_X2;
          else
            ui8_g_pas_tick_counter--;
        }
      }

      // define if pedal is right or left
      if (ui8_g_pas_tick_counter > PAS_NUMBER_MAGNETS)
        ui8_g_pas_pedal_right = 0;
      else
        ui8_g_pas_pedal_right = 1;

      // save torque sensor ADC value when pedals are on horizontal
      if ((ui8_g_pas_tick_counter == PAS_NUMBER_MAGNETS_1_4) ||
          (ui8_g_pas_tick_counter == PAS_NUMBER_MAGNETS_3_4))
      {
        ui16_g_adc_torque_sensor_raw_horizontal = UI16_ADC_10_BIT_TORQUE_SENSOR;

        if (ui8_g_torque_sensor_horizontal_cnt < 2)
          ui8_g_torque_sensor_horizontal_cnt++;
      }
    }

    // check for permitted relative min cadence value
    if ((ui8_m_pedaling_direction == 2) || // if rotating pedals backwards
        (ui16_m_pas_counter > ui16_m_pas_min_cadence_pwm_cycles_ticks))
      ui8_m_pas_min_cadence_flag = 1;
    else
      ui8_m_pas_min_cadence_flag = 0;

    // limit min PAS cadence
    if (ui8_m_pas_min_cadence_flag ||
        ui16_m_pas_counter > ((uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS))
    {
      ui16_g_pas_pwm_cycles_ticks = (uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
      ui16_m_pas_min_cadence_pwm_cycles_ticks = (uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
      ui8_m_pas_min_cadence_flag = 0;
      ui8_m_pas_after_first_pulse = 0;
      ui8_m_pedaling_direction = 0;
      ui16_m_pas_counter = 0;
      ui8_g_torque_sensor_horizontal_cnt = 0;
    }
    /****************************************************************************/
    
    // calc wheel speed sensor timming between each positive pulses, in PWM cycles ticks
    ui16_wheel_speed_sensor_counter++;

    // limit min wheel speed
    if (ui16_wheel_speed_sensor_counter > ((uint16_t) WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS))
    {
      ui16_wheel_speed_sensor_pwm_cycles_ticks = (uint16_t) WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS;
      ui16_wheel_speed_sensor_counter = 0;
      ui8_wheel_speed_sensor_change_counter = 0;
    }
    // let´s look if signal state changed
    else
    {
      // detect wheel speed sensor signal changes
      if (WHEEL_SPEED_SENSOR__PORT->IDR & WHEEL_SPEED_SENSOR__PIN)
        ui8_wheel_speed_sensor_state = 1;
      else
        ui8_wheel_speed_sensor_state = 0;

      if (ui8_wheel_speed_sensor_state != ui8_wheel_speed_sensor_state_old) // wheel speed sensor signal did change
      {
        ui8_wheel_speed_sensor_state_old = ui8_wheel_speed_sensor_state;

        if (ui8_wheel_speed_sensor_state == 1) // consider only when wheel speed sensor signal transition from 0 to 1
        {
          // Here we are trying to count 2 consecutive wheel speed signal changes, other way we will have erroneus values on the first
          // signal change. The correct time needs to be measured between 2 consecutive signal changes.
          ui8_wheel_speed_sensor_change_counter++;

          if (ui8_wheel_speed_sensor_change_counter >= 2)
          {
            ui16_wheel_speed_sensor_pwm_cycles_ticks = ui16_wheel_speed_sensor_counter;
            ui16_wheel_speed_sensor_counter = 0;
            ui32_wheel_speed_sensor_tick_counter++;
            ui8_wheel_speed_sensor_change_counter = 1; // keep this counter as 1, meaning we just counted one previous change
          }
        }
      }
    }

    /****************************************************************************/
    // reload watchdog timer, every PWM cycle to avoid automatic reset of the microcontroller
    if (ui8_first_time_run_flag)
    { // from the init of watchdog up to first reset on PWM cycle interrupt,
      // it can take up to 250ms and so we need to init here inside the PWM cycle
      ui8_first_time_run_flag = 0;
      watchdog_init();
    }
    else
    {
      IWDG->KR = IWDG_KEY_REFRESH; // reload watch dog timer counter

      // if the main loop counteris not reset that it is blocked, so, reset the system
      ++ui16_main_loop_wdt_cnt_1;
      if (ui16_main_loop_wdt_cnt_1 > 19061) // 1 second
      {
        // reset system
        //  resets a STM8 microcontroller.
        //  It activates the Window Watchdog, which resets all because its seventh bit is null.
        //  See page 127 of  RM0016 (STM8S and STM8AF microcontroller family) for more details.
        WWDG->CR = 0x80;
      }
    }
    /****************************************************************************/
    }
    // clears the TIM1 interrupt TIM1_IT_UPDATE pending bit
    TIM1->SR1 = (uint8_t)(~(uint8_t)TIM1_IT_CC4);
    
}

void motor_disable_PWM(void)
{
  TIM1_CtrlPWMOutputs(DISABLE);
}

void motor_enable_PWM(void)
{
  TIM1_CtrlPWMOutputs(ENABLE);
}

// void hall_sensor_init(void)
// {
//   GPIO_Init (HALL_SENSOR_A__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_A__PIN, GPIO_MODE_IN_FL_NO_IT);
//   GPIO_Init (HALL_SENSOR_B__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_B__PIN, GPIO_MODE_IN_FL_NO_IT);
//   GPIO_Init (HALL_SENSOR_C__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_C__PIN, GPIO_MODE_IN_FL_NO_IT);
// }


void hall_sensor_init(void) {
    // Init Hall sensor GPIO
    GPIO_Init(HALL_SENSOR_A__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_A__PIN, GPIO_MODE_IN_FL_IT);
    GPIO_Init(HALL_SENSOR_B__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_B__PIN, GPIO_MODE_IN_FL_IT);
    GPIO_Init(HALL_SENSOR_C__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_C__PIN, GPIO_MODE_IN_FL_IT);

    ui8_hall_state_irq = 0;
    if (HALL_SENSOR_A__PORT->IDR & HALL_SENSOR_A__PIN)
        ui8_hall_state_irq |= (unsigned char)0x01;
    if (HALL_SENSOR_B__PORT->IDR & HALL_SENSOR_B__PIN)
        ui8_hall_state_irq |= (unsigned char)0x02;
    if (HALL_SENSOR_C__PORT->IDR & HALL_SENSOR_C__PIN)
        ui8_hall_state_irq |= (unsigned char)0x04;

    // Hall GPIO priority = 3. Priority increases from 1 (min priority) to 3 (max priority)
    ITC_SetSoftwarePriority(EXTI_HALL_A_IRQ, ITC_PRIORITYLEVEL_3);
    ITC_SetSoftwarePriority(EXTI_HALL_B_IRQ, ITC_PRIORITYLEVEL_3);
    ITC_SetSoftwarePriority(EXTI_HALL_C_IRQ, ITC_PRIORITYLEVEL_3);

    // Hall GPIO signal interrupt sensitivity on both rising and falling edges
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOC, EXTI_SENSITIVITY_RISE_FALL);
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_FALL);
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOE, EXTI_SENSITIVITY_RISE_FALL);
    EXTI_SetTLISensitivity(EXTI_TLISENSITIVITY_FALL_ONLY);
}


void motor_init(void)
{
  motor_set_pwm_duty_cycle_ramp_up_inverse_step(PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP); // each step = 64us
  motor_set_pwm_duty_cycle_ramp_down_inverse_step(PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP); // each step = 64us
}

void motor_set_pwm_duty_cycle_target(uint8_t ui8_value)
{
  if (ui8_value > PWM_DUTY_CYCLE_MAX)
    ui8_value = PWM_DUTY_CYCLE_MAX;

  // if brake is active, keep duty_cycle target at 0
  if (ui8_g_brake_is_set)
    ui8_value = 0;

  ui8_m_duty_cycle_target = ui8_value;
}

void motor_set_pwm_duty_cycle_ramp_up_inverse_step(uint16_t ui16_value)
{
  ui16_duty_cycle_ramp_up_inverse_step = ui16_value;
}

void motor_set_pwm_duty_cycle_ramp_down_inverse_step(uint16_t ui16_value)
{
  ui16_duty_cycle_ramp_down_inverse_step = ui16_value;
}

uint16_t ui16_motor_get_motor_speed_erps(void)
{
  return ui16_motor_speed_erps;
}

void read_battery_voltage(void)
{
  // low pass filter the voltage readed value, to avoid possible fast spikes/noise
  ui16_adc_battery_voltage_accumulated -= ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
  ui16_adc_battery_voltage_accumulated += UI16_ADC_10_BIT_BATTERY_VOLTAGE;
  ui16_adc_battery_voltage_filtered_10b = ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
}

void read_battery_current(void)
{
  // low pass filter the positive battery readed value (no regen current), to avoid possible fast spikes/noise
  ui16_adc_battery_current_accumulated -= ui16_adc_battery_current_accumulated >> READ_BATTERY_CURRENT_FILTER_COEFFICIENT;
  ui16_adc_battery_current_accumulated += ui16_g_adc_battery_current;
  ui16_g_adc_battery_current_filtered = ui16_adc_battery_current_accumulated >> READ_BATTERY_CURRENT_FILTER_COEFFICIENT;
}

void read_motor_current(void)
{
  // low pass filter the positive motor readed value (no regen current), to avoid possible fast spikes/noise
  ui16_adc_motor_current_accumulated -= ui16_adc_motor_current_accumulated >> READ_MOTOR_CURRENT_FILTER_COEFFICIENT;
  ui16_adc_motor_current_accumulated += ui16_g_adc_motor_current;
  ui16_g_adc_motor_current_filtered = ui16_adc_motor_current_accumulated >> READ_MOTOR_CURRENT_FILTER_COEFFICIENT;
}

void calc_foc_angle(void)
{
  uint8_t ui8_temp;
  uint16_t ui16_temp;
  uint32_t ui32_temp;
  uint16_t ui16_e_phase_voltage;
  uint32_t ui32_i_phase_current_x2;
  uint32_t ui32_l_x1048576;
  uint32_t ui32_w_angular_velocity_x16;
  uint16_t ui16_iwl_128;

  struct_config_vars *p_configuration_variables;
  p_configuration_variables = get_configuration_variables();

  // FOC implementation by calculating the angle between phase current and rotor magnetic flux (BEMF)
  // 1. phase voltage is calculate
  // 2. I*w*L is calculated, where I is the phase current. L was a measured value for 48V motor.
  // 3. inverse sin is calculated of (I*w*L) / phase voltage, were we obtain the angle
  // 4. previous calculated angle is applied to phase voltage vector angle and so the
  // angle between phase current and rotor magnetic flux (BEMF) is kept at 0 (max torque per amp)

  // calc E phase voltage
  ui16_temp = ui16_adc_battery_voltage_filtered_10b * ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512;
  ui16_temp = (ui16_temp >> 8) * ui8_g_duty_cycle;
  ui16_e_phase_voltage = ui16_temp >> 9;

  // calc I phase current
  if (ui8_g_duty_cycle > 10)
  {
    ui16_temp = ((uint16_t) ui16_g_adc_battery_current_filtered) * ADC10BITS_BATTERY_CURRENT_PER_ADC_STEP_X512;
    ui32_i_phase_current_x2 = ui16_temp / ui8_g_duty_cycle;
  }
  else
  {
    ui32_i_phase_current_x2 = 0;
  }

  // calc W angular velocity: erps * 6.3
  // 101 = 6.3 * 16
  ui32_w_angular_velocity_x16 = ui16_motor_speed_erps * 101;

  // ---------------------------------------------------------------------------------------------------------------------
  // 36 V motor: L = 76uH
  // 48 V motor: L = 135uH
  // ui32_l_x1048576 = 142; // 1048576 = 2^20 | 48V
  // ui32_l_x1048576 = 84; // 1048576 = 2^20 | 36V
  //
  // ui32_l_x1048576 = 142 <--- THIS VALUE WAS verified experimentaly on 2018.07 to be near the best value for a 48V motor
  // Test done with a fixed mechanical load, duty_cycle = 200 and 100 and measured battery current was 16 and 6 (10 and 4 amps)
  // ---------------------------------------------------------------------------------------------------------------------
  
  switch (p_configuration_variables->ui8_motor_type)
  {
    default:
    case 0:
      ui32_l_x1048576 = 142; // 48 V motor
    break;

    case 1:
      ui32_l_x1048576 = 84; // 36 V motor
    break;
  }

  // calc IwL
  ui32_temp = ui32_i_phase_current_x2 * ui32_l_x1048576;
  ui32_temp *= ui32_w_angular_velocity_x16;
  ui16_iwl_128 = ui32_temp >> 18;

  // calc FOC angle
  ui8_temp = asin_table(ui16_iwl_128 / ui16_e_phase_voltage);

  // low pass filter FOC angle
  ui16_foc_angle_accumulated -= (ui16_foc_angle_accumulated >> 4);
  ui16_foc_angle_accumulated += (uint16_t) ui8_temp;
  ui8_g_foc_angle = (uint8_t) (ui16_foc_angle_accumulated >> 4);
}

//calc asin also converts the final result to degrees
uint8_t asin_table (uint8_t ui8_inverted_angle_x128)
{
 return ui8_asin_table[ui8_inverted_angle_x128 & 127U];
}


void motor_set_adc_battery_voltage_cut_off(uint8_t ui8_value)
{
  ui8_adc_battery_voltage_cut_off = ui8_value;
}

uint16_t motor_get_adc_battery_voltage_filtered_10b(void)
{
  return ui16_adc_battery_voltage_filtered_10b;
}

void motor_enable_pwm(void)
{
  TIM1_OC1Init(TIM1_OCMODE_PWM1,
#ifdef DISABLE_PWM_CHANNELS_1_3
         TIM1_OUTPUTSTATE_DISABLE,
         TIM1_OUTPUTNSTATE_DISABLE,
#else
         TIM1_OUTPUTSTATE_ENABLE,
         TIM1_OUTPUTNSTATE_ENABLE,
#endif
         128, // initial duty_cycle value
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCIDLESTATE_RESET,
         TIM1_OCIDLESTATE_SET);

  TIM1_OC2Init(TIM1_OCMODE_PWM1,
         TIM1_OUTPUTSTATE_ENABLE,
         TIM1_OUTPUTNSTATE_ENABLE,
         128, // initial duty_cycle value
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCIDLESTATE_RESET,
         TIM1_OCIDLESTATE_SET);

  TIM1_OC3Init(TIM1_OCMODE_PWM1,
#ifdef DISABLE_PWM_CHANNELS_1_3
         TIM1_OUTPUTSTATE_DISABLE,
         TIM1_OUTPUTNSTATE_DISABLE,
#else
         TIM1_OUTPUTSTATE_ENABLE,
         TIM1_OUTPUTNSTATE_ENABLE,
#endif
         128, // initial duty_cycle value
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCIDLESTATE_RESET,
         TIM1_OCIDLESTATE_SET);
}

void motor_disable_pwm(void)
{
  TIM1_OC1Init(TIM1_OCMODE_PWM1,
         TIM1_OUTPUTSTATE_DISABLE,
         TIM1_OUTPUTNSTATE_DISABLE,
         128, // initial duty_cycle value
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCIDLESTATE_RESET,
         TIM1_OCIDLESTATE_SET);

  TIM1_OC2Init(TIM1_OCMODE_PWM1,
         TIM1_OUTPUTSTATE_DISABLE,
         TIM1_OUTPUTNSTATE_DISABLE,
         128, // initial duty_cycle value
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCIDLESTATE_RESET,
         TIM1_OCIDLESTATE_SET);

  TIM1_OC3Init(TIM1_OCMODE_PWM1,
         TIM1_OUTPUTSTATE_DISABLE,
         TIM1_OUTPUTNSTATE_DISABLE,
         255, // initial duty_cycle value
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCIDLESTATE_RESET,
         TIM1_OCIDLESTATE_SET);

}

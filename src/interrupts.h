/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _INTERRUPTS_H_
#define _INTERRUPTS_H_

#define EXTI_PORTD_IRQHANDLER 6
#define EXTI_PORTE_IRQHANDLER 7
#define TIM1_CAP_COM_IRQHANDLER 	12
#define TIM2_UPD_OVF_TRG_BRK_IRQHANDLER 13
#define UART2_TX_IRQHANDLER 20
#define UART2_RX_IRQHANDLER 21
#define ADC1_IRQHANDLER 22


#define EXTI_HALL_A_IRQ  7              // ITC_IRQ_PORTE - Hall sensor A rise/fall detection
#define EXTI_HALL_B_IRQ  6              // ITC_IRQ_PORTD - Hall sensor B rise/fall detection
#define EXTI_HALL_C_IRQ  5              // ITC_IRQ_PORTC - Hall sensor C rise/fall detection

#define TIM4_OVF_IRQHANDLER 23          // ITC_IRQ_TIM4_OVF - TIM 4 overflow: 1ms counter


#endif

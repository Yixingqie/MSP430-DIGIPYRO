/*
 * msp430_isr.c
 *
 *  Created on: Dec 12, 2020
 *      Author: Miroslav Oljaca
 */

#ifndef SOURCE_MSP430PIR_ISR_C_
#define SOURCE_MSP430PIR_ISR_C_

#include "msp430_isr.h"
#include <msp430.h>


//******************************************************************************
//                  Timer0_B0 ISR (TB0.0)
//******************************************************************************
/*! \brief Timer TB0.0 ISR is main sampling and update period.
 *
 *  This is main ISR that wakes up CPU core. It returns in active mode (AM).
 *  GIE is still set and needs to be clear later.
 *
 *  \return none
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer0_B0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_B0_VECTOR))) Timer0_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    TB0CCTL0 &= ~CCIFG;    // Clear interrupt flag

    __low_power_mode_off_on_exit();     // Clear SCG1, SCG0, OSCOFF and CPUOFF
                                        // Does NOT clear GIE!
}   /* Timer0_B0_ISR() */


//******************************************************************************
//                  Timer1_B0 ISR (TB1.0)
//******************************************************************************
/*! \brief Timer TB1.0 ISR is used for programmable time delay.
 *
 *  This ISR is used to have CPU in low power mode 3 (LPM3) till it is waiting
 *  for time delay. It is used to properly settle analog after enabling
 *  different parts of SAC.It returns in active mode (AM).
 *  GIE is still set and needs to be clear later.
 *
 *  \return none
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_B0_VECTOR
__interrupt void Timer1_B0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_B0_VECTOR))) Timer1_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    P1OUT &= ~BIT0;
    TB1CTL  &= ~MC_3;       // TB2 Stop timer
    TB1CCTL0 &= ~CCIE;      // TB2 Disable interrupt
    TB1CCTL0 &= ~CCIFG;     // TB2 Clear interrupt

}   /* Timer1_B0_ISR() */


//******************************************************************************
//                  Timer2_B0 ISR (TB2.0)
//******************************************************************************
/*! \brief Timer TB2.0 ISR is used for programmable time delay.
 *
 *  It controls delay for LED diodes for UI.It returns in active mode (AM).
 *  GIE is still set.
 *
 *  \return none
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER2_B0_VECTOR
__interrupt void Timer2_B0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER2_B0_VECTOR))) Timer2_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{

    P6OUT &= ~BIT6;
    TB2CTL  &= ~MC_3;       // TB2 Stop timer
    TB2CCTL0 &= ~CCIE;      // TB2 Disable interrupt
    TB2CCTL0 &= ~CCIFG;     // TB2 Clear interrupt

}   /* Timer2_B0_ISR() */


//******************************************************************************
//                  ADC ISR
//******************************************************************************
/*! \brief ADC ISR reads result from conversion.
 *
 *  It reads ADC result and saves in corresponding adc_cannel array.
 *  It returns in active mode (AM). GIE is still set and needs to be clear later.
 *
 *  \return adc_channel[ch]
 */
//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
//#pragma vector = ADC_VECTOR
//__interrupt void ADC_ISR(void)
//#elif defined(__GNUC__)
//void __attribute__ ((interrupt(ADC_VECTOR))) ADC_ISR (void)
//#else
//#error Compiler not supported!
//#endif
//{
//    uint16_t adc_input_channel;
//
//    switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
//    {
//        case ADCIV_NONE:
//        break;
//
//        case ADCIV_ADCOVIFG:
//        break;
//
//        case ADCIV_ADCTOVIFG:
//        break;
//
//        case ADCIV_ADCHIIFG:
//        break;
//
//        case ADCIV_ADCLOIFG:
//        break;
//
//        case ADCIV_ADCINIFG:
//        break;
//
//        case ADCIV_ADCIFG:
//            adc_input_channel = ADCMCTL0 & ADCINCH_15;
//            adc_channel[adc_input_channel] = ADCMEM0;
//        break;
//
//        default:
//        break;
//    }
//    __low_power_mode_off_on_exit();     // Clear SCG1, SCG0, OSCOFF and CPUOFF
//                                        // Does NOT clear GIE!
//}   /* ADC_ISR() */

#endif /* SOURCE_MSP430PIR_ISR_C_ */

/*** end of file ***/

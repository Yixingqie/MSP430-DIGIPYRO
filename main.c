// Example code for reading PYQ-2898 with MSP430F2274
// Direct Link of PYQ-2898 is connected to P1.2 of MSP430F2274
// P2.1 of MSP430F2274 is used to show SMCLK

#include <msp430.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "uart_logging.h"

#define UART_LOGGING

#define PIR_DETECTION_THRESHOLD 50    /* Detection threshold */
#define PIR_LONG_TERM_FILTER_RANGE 16 /* Length of long term filter */
#define LONG_DIVIDER                4

#define PIR_SHORT_TERM_FILTER_RANGE 8 /* Length of short term filter */
#define SHORT_DIVIDER               3
#define MOTION_COUNTER_THRESHOLD 5


#define POS_COUNTER_THRESHOLD 5
#define NEG_COUNTER_THRESHOLD 5

#define ACTIVE_PIR 2
#define NUM_PIR 2

int motion_state[NUM_PIR]                               = {0,0};
long filter_delta[NUM_PIR]                              = {0,0};
unsigned long  long_term_average[NUM_PIR]               = {0,0};
unsigned long  short_term_average[NUM_PIR]              = {0,0};
unsigned long accumulated_long_term_average[NUM_PIR]    = {0,0};
unsigned long accumulated_short_term_average[NUM_PIR]   = {0,0};
int motion_counter[NUM_PIR]                             = {0,0};
int positive_motion_counter[NUM_PIR]                    = {0,0};
int negative_motion_counter[NUM_PIR]                    = {0,0};

bool initialized[NUM_PIR]                               = {false, false};
bool motion[NUM_PIR]                                    = {false, false};
volatile unsigned char *LED_PORT [NUM_PIR]              = {&P1OUT, &P6OUT};
uint16_t LED_PIN [NUM_PIR]                              = {BIT0, BIT6};
volatile unsigned int *TB_CTL [NUM_PIR]                 = {&TB1CTL, &TB2CTL};
volatile unsigned int *TB_CCR [NUM_PIR]                 = {&TB1CCR0, &TB2CCR0};
volatile unsigned int *TB_CCTL [NUM_PIR]                = {&TB1CCTL0, &TB2CCTL0};

uint16_t PYRO_PIN [NUM_PIR]                              = {BIT2, BIT3};


void signal_analysis_ma(uint32_t inst_value, uint16_t pir_idx);
void led_motion_detected_new (uint16_t pir_idx);
uint32_t abs_inst_dev (int32_t xn);

// Data[0]: Channel 1, Data[1]: Channel 0, Data[2]: Temperature
unsigned long Data[2][2];
unsigned int bitmask;
unsigned int idx, bit;
#define NOP()  __no_operation()



void read_PYD(uint16_t bit_input);
char buffer[64];
int main( void )
{

    P1OUT = 0x00; P2OUT = 0x00;
    P3OUT = 0x00; P4OUT = 0x00;
    P5OUT = 0x00; P6OUT = 0x00;
    P1DIR = 0xff; P2DIR = 0xff;
    P3DIR = 0xff; P4DIR = 0xff;
    P5DIR = 0xff; P6DIR = 0xff;

    //LED RED
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;


    //LED GREEN
    P6DIR |= BIT6;
    P6OUT |= BIT6;

    PM5CTL0 &= ~LOCKLPM5;


  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;

  CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_0;// DCOFTRIM=3, DCO Range = 1MHz
 CSCTL2 = FLLD_0 + 30;                   // DCODIV = 1MHz
 __delay_cycles(3);
 __bic_SR_register(SCG0);                // Enable FLL

     CSCTL4 = SELA__VLOCLK | SELMS__DCOCLKDIV; // MCLK=SMCLK=DCO; ACLK=VLO

     TB0CTL |= TBCLR;
       TB0CTL |= MC__UP;

       TB0CTL |= TBSSEL__ACLK;
       TB0CCR0 = 500;


       TB0CCTL0 |= CCIE;

       __enable_interrupt();
       TB0CTL &= ~CCIFG;

       init_ser();

       unsigned int i = 0;
       while (1)
          {
              read_PYD(i);
              signal_analysis_ma(Data[i][0], i);
//              i = 1 - i;
//              read_PYD(1);
//              signal_analysis_ma(Data[1][0], 1);
              __enable_interrupt();
              __low_power_mode_3();   // Returning here from TB0.0 ISR in AM
              __disable_interrupt();

          }


}

void read_PYD(uint16_t bit_input){
    //Set the DL HIGH
  P1OUT |= PYRO_PIN[bit_input];
  P1DIR |= PYRO_PIN[bit_input];

//  __delay_cycles(1);

  for (idx = 0; idx <1; idx++)
  {
    bitmask = 0x2000;
    Data[bit_input][idx] = 0;

    for (bit = 0; bit<14; bit++)
    {
      P1OUT &= ~PYRO_PIN[bit_input]; //Sets Pin LOW
      P1DIR |= PYRO_PIN[bit_input]; // Set as Output

      P1OUT |= PYRO_PIN[bit_input];  //Sets PIN HIGH
      P1DIR &= ~PYRO_PIN[bit_input]; //Sets PIN as INPUT
      NOP();
      if(P1IN & PYRO_PIN[bit_input]){
          Data[bit_input][idx] |= bitmask;
      }
      bitmask>>=1;
    }
  }
  //Set the DL HIGH
    P1DIR |= PYRO_PIN[bit_input];
    P1OUT &= ~PYRO_PIN[bit_input];
//    __delay_cycles (1);         // *** Ref Point # 5 ***
    NOP();
    P1DIR &= ~PYRO_PIN[bit_input]; //Sets PIN as INPUT
}


void signal_analysis_ma(uint32_t inst_value, uint16_t pir_idx){

    if(!initialized[pir_idx]){
          uint16_t i = 0;
          while (i < PIR_LONG_TERM_FILTER_RANGE) {
              unsigned int p;
              for(p = 0; p < NUM_PIR; p++){
                accumulated_long_term_average[p] += inst_value;

                if (i < PIR_SHORT_TERM_FILTER_RANGE) {
                  accumulated_short_term_average[p] += inst_value;
                }
              }
              i++;
          }

          initialized[pir_idx] = true;
    }else{
        accumulated_long_term_average[pir_idx] -=  long_term_average[pir_idx]; /* Subtract 1/x from accumulated filter value */  //divide by 128 by >>7
        accumulated_short_term_average[pir_idx] -= short_term_average[pir_idx]; /* Subtract 1/y from accumulated filter value */ //divide by 16 by >>4

        accumulated_long_term_average[pir_idx] += inst_value;  /* Add new ADC measurement (1/x) to accumulated filter value */
        accumulated_short_term_average[pir_idx] += inst_value; /* Add new ADC measurement (1/y) to accumulated filter value */

        long_term_average[pir_idx] = accumulated_long_term_average[pir_idx] >> LONG_DIVIDER; /* Divide the accumulated_long_term_average on X to create long_term_average */
        short_term_average[pir_idx] = accumulated_short_term_average[pir_idx] >> SHORT_DIVIDER; /* Divide the accumulated_long_term_average on Y to create long_term_average */

        filter_delta[pir_idx] = long_term_average[pir_idx] - short_term_average[pir_idx]; /* Find delta between filters */


        if (abs_inst_dev(filter_delta[pir_idx]) >= PIR_DETECTION_THRESHOLD) {
            if(motion_counter[pir_idx] < MOTION_COUNTER_THRESHOLD){
              motion_counter[pir_idx]++;
            }
        } else {
            if(motion_counter[pir_idx] > 0){
              motion_counter[pir_idx] = 0;
            }
        }

        if(motion_counter[pir_idx] >= MOTION_COUNTER_THRESHOLD && !motion[pir_idx]){
            led_motion_detected_new(pir_idx);
//            *LED_PORT[pir_idx] |= LED_PIN[pir_idx];
            motion[pir_idx] = true;
        }
        else if(motion_counter[pir_idx] <= MOTION_COUNTER_THRESHOLD)
        {
//            *LED_PORT[pir_idx] &= ~LED_PIN[pir_idx];
            motion[pir_idx] = false;
        }

//        if (filter_delta[pir_idx] > PIR_DETECTION_THRESHOLD) {
//            positive_motion_counter[pir_idx]+=2;
//        } else if(filter_delta[pir_idx] < -PIR_DETECTION_THRESHOLD) {
//            negative_motion_counter[pir_idx]+=2;
//        }else{
//            if(negative_motion_counter[pir_idx] > 0){
//                negative_motion_counter[pir_idx]--;
//            }
//
//            if(positive_motion_counter[pir_idx] > 0){
//                positive_motion_counter[pir_idx]--;
//            }
//        }
//
//        if(positive_motion_counter[pir_idx] >= POS_COUNTER_THRESHOLD && negative_motion_counter[pir_idx] >= NEG_COUNTER_THRESHOLD && !motion[pir_idx]){
//            led_motion_detected_new(pir_idx);
////            *LED_PORT[pir_idx] |= LED_PIN[pir_idx];
//            motion[pir_idx] = true;
//        }
//        else if(positive_motion_counter[pir_idx] < POS_COUNTER_THRESHOLD || negative_motion_counter[pir_idx] < NEG_COUNTER_THRESHOLD)
//        {
////            *LED_PORT[pir_idx] &= ~LED_PIN[pir_idx];
//            motion[pir_idx] = false;
//        }
//
        if(pir_idx == 0){
            sprintf(buffer, "long_ave:  %lu   short_ave:  %lu    delta: %ld   pos: %u    neg: %u  \r\n", long_term_average[pir_idx], short_term_average[pir_idx], filter_delta[pir_idx], positive_motion_counter[pir_idx], negative_motion_counter[pir_idx]);
            ser_output(buffer);
        }



    }
}


void
led_motion_detected_new (uint16_t pir_idx)
{
#   define LED_TIME_DELAY_DIVIDER   10000   // 5 s

    *LED_PORT[pir_idx] |= LED_PIN[pir_idx];

    *TB_CTL[pir_idx] |= TBSSEL__ACLK;
    *TB_CTL[pir_idx]  &= ~MC_3;        // TB2 Stop timer
    *TB_CTL[pir_idx]  |= TBCLR;        // Clear TBxR, clock divider state, and the counter direction.


    *TB_CCR[pir_idx] = LED_TIME_DELAY_DIVIDER - 1;   // TB2CL0 (TB1CCR0) defines the time period. The number
                                             // of timer counts in the period is TBxCL0 + 1.

    *TB_CCTL[pir_idx] &= ~CCIFG;         // TB1 Clear interrupt
    *TB_CCTL[pir_idx] |= CCIE;           // TB1 Enable interrupt
    *TB_CTL[pir_idx] |= MC__UP;  // TB1 Start timer

}   /* led_motion_detected() */


uint32_t
abs_inst_dev (int32_t xn)
{
    uint32_t yn;

    if (xn<0)
    {
        yn = (uint32_t) ~xn + 1;
    }
    else
    {
        yn = (uint32_t) xn;
    }

    return(yn);

}   /* abs_inst_dev() */




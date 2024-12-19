/*
 * uart_logging.c
 *
 *  Created on: Dec. 7, 2024
 *      Author: Yixing
 */
#include "uart_logging.h"
#include <msp430.h>


void init_ser(){

//    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
//                                                   GPIO_PIN2 |      // P4.2 UCA1RXD
//                                                   GPIO_PIN3 ,      // P4.3 UCA1TXD
//                                                   GPIO_PRIMARY_MODULE_FUNCTION     // P4SELx = 01
//                                                  );                // This function configures the peripheral module function in the input direction for the selected pin.
    P4SEL0 &= ~BIT2;
    P4SEL1 |= BIT2;

    P4SEL0 &= ~BIT3;
    P4SEL1 |= BIT3;

    UCA1CTLW0 |= UCSWRST;                      // Put eUSCI in reset
    UCA1CTLW0 |= UCSSEL__SMCLK;
    UCA1BRW = 8;         // 8Mhz/69 = 115200 Baud // 1Mhz/8 = 115200 Baud
    UCA1MCTLW = 0xD600;


    P4SEL1 &= ~BIT3;
    P4SEL0 |= BIT3;

    UCA1CTLW0 &= ~UCSWRST;
}

void ser_output(char *str){
    while(*str) {
        while (!(UCA1IFG&UCTXIFG));
        UCA1TXBUF = *str++;
    }
}


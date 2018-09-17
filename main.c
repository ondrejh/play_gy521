//******************************************************************************
//   MSP430G2553 Tank receiver / convertor
//
//   Description: Receive data from FS-A8S receiver
//                Convert CH0 (X), CH1 (Y) to left/right PWM
//				  Send CH2 to SERVO 1 output
//
//                MSP430G2553
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |     P1.1/UCA0RXD|<--- UART ECHO INPUT
//            |     P1.2/UCA0TXD|---> UART OUTPUT
//            |                 |
//            |             P1.0|---> RED LED (RX ON)
//            |             P1.6|---> GREEN LED (PWM ON)
//            |                 |
//            |                 |
//
//   Author: Ondrej Hejda
//   Date started: September 2018
//
//******************************************************************************
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#define LED_INIT() do{P1DIR|=0x01;P1OUT&=~0x01;P1SEL&=~0x01;P1SEL2&=~0x01;}while(0)
//#define LED_INIT() do{P1DIR|=0x41;P1OUT&=~0x41;P1SEL&=~0x41;P1SEL2&=~0x41;}while(0)
#define LED_RED_ON() do{P1OUT|=0x01;}while(0)
#define LED_RED_OFF() do{P1OUT&=~0x01;}while(0)
#define LED_RED_SWAP() do{P1OUT^=0x01;}while(0)
/*#define LED_GREEN_ON() do{P1OUT|=0x40;}while(0)
#define LED_GREEN_OFF() do{P1OUT&=~0x40;}while(0)
#define LED_GREEN_SWAP() do{P1OUT^=0x40;}while(0)*/

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;        // Stop WDT

    if (CALBC1_1MHZ==0xFF) while(1); // If calibration constant erased do not load, trap CPU!!
    DCOCTL = 0;                      // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;           // Set DCO
    DCOCTL = CALDCO_1MHZ;

    // init uart
    P1SEL = BIT1 + BIT2 ;            // P1.1 = RXD, P1.2=TXD
    P1SEL2 = BIT1 + BIT2;
    UCA0CTL1 |= UCSSEL_2;            // SMCLK
    UCA0BR0 = 104;                   // 1MHz 9600
    UCA0BR1 = 0;                     // 1MHz 9600
    UCA0MCTL = UCBRS2 + UCBRS0;      // Modulation UCBRSx = 5
    UCA0CTL1 &= ~UCSWRST;            // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                 // Enable USCI_A0 RX interrupt

    // init i2c
    P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
    P1SEL2|= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
    UCB0CTL1 |= UCSWRST;                      // Enable SW reset
    UCB0CTL0 = UCMST+UCMODE_3+UCSYNC;         // I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2+UCSWRST;              // Use SMCLK, keep SW reset
    UCB0BR0 = 3;                              // fSCL = SMCLK/3 = ~400kHz
    UCB0BR1 = 0;
    UCB0I2CSA = 0x68;                         // Set slave address
    UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
    IE2 |= UCB0RXIE;                          // Enable RX interrupt

    // setup LEDs
    LED_INIT();

    // main loop
    while(1) {
        __bis_SR_register(LPM0_bits + GIE); // Enter LPM0, interrupts enabled
        LED_RED_SWAP();
    }

    return -1;
}

// Echo back RXed character, confirm TX buffer is ready first
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    while (!(IFG2&UCA0TXIFG)); // USCI_A0 TX buffer ready?
    UCA0TXBUF = UCA0RXBUF;     // TX -> RXed character
    __bic_SR_register_on_exit(LPM0_bits);
}

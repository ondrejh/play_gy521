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

unsigned int RxByteCtr;
unsigned int RxWord;

#define TXBUFLEN 16
uint8_t utxbuff[TXBUFLEN];
volatile uint16_t utxbuff_cnt = 0;
volatile uint16_t utxbuff_ptr = 0;

uint8_t u2h(uint8_t u) {
    if (u<10)
        return '0'+u;
    return 'A'-10+u;
}

void uart_start_tx(uint16_t cnt) {
    if (cnt>0) {
        utxbuff_cnt = cnt;
        while (!(IFG2&UCA0TXIFG));
        UCA0TXBUF = utxbuff[0];
        utxbuff_ptr = 1;
        IE2 |= UCA0TXIE;
    }
}

void print_data(uint8_t* data, int dlen) {
    int i, ptr=0;
    for (i=0;i<dlen;i++) {
        utxbuff[ptr++] = u2h(data[i]>>8);
        utxbuff[ptr++] = u2h(data[i]&0x0F);
    }
    utxbuff[ptr++] = '\n';
    utxbuff[ptr++] = '\r';
    uart_start_tx(ptr);
}

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
    IE2 |= UCA0RXIE;      // Enable USCI_A0 RX interrupt

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
        RxByteCtr = 2;                          // Load RX byte counter
        RxWord = 0x1234;
        UCB0CTL1 |= UCTXSTT;                    // I2C start condition
        __bis_SR_register(CPUOFF + GIE);        // Enter LPM0, enable interrupts
                                            // Remain in LPM0 until all data
                                            // is RX'd
        if (RxWord != 0x0)                    // >28C?
            LED_RED_ON();
        else
            LED_RED_OFF();

        print_data((uint8_t*)&RxWord, 2);
    }

    return -1;
}

// Echo back RXed character, confirm TX buffer is ready first
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCIAB0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    if (IFG2&UCA0RXIFG) {
        char c = UCA0RXBUF;
        if (c=='s')
            __bic_SR_register_on_exit(LPM0_bits);
    }
    if ((IFG2 & UCB0RXIFG) && (IE2 & UCB0RXIE)) {
        RxByteCtr--;                              // Decrement RX byte counter

        if (RxByteCtr)
        {
            RxWord = (unsigned int)UCB0RXBUF << 8;  // Get received byte
            if (RxByteCtr == 1)                     // Only one byte left?
            UCB0CTL1 |= UCTXSTP;                  // Generate I2C stop condition
        }
        else
        {
            RxWord |= UCB0RXBUF;                    // Get final received byte,
                                                // Combine MSB and LSB
            __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }
    }
}

// The USCIAB0TX_ISR is structured such that it can be used to receive any
// 2+ number of bytes by pre-loading RxByteCtr with the byte count.
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCIAB0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    if ((IFG2 & UCA0TXIFG) && (IE2 & UCA0TXIE)) {
        if (utxbuff_ptr < utxbuff_cnt) {
            UCA0TXBUF = utxbuff[utxbuff_ptr++];
        }
        else {
            IE2 &= ~UCA0TXIE;
        }
    }
}


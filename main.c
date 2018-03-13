//******************************************************************************
//   MSP430G2553 Tank receiver / convertor
//
//   Description: Receive data from FS-A8S receiver
//                Convert CH0, CH1 to tank PWM
//
//                MSP430G2553
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |     P1.1/UCA0RXD|<--- FS-A8S
//            |                 |
//            |             P1.0|---> RED LED (RX ON)
//            |             P1.5|---> GREEN LED (PWM ON)
//            |                 |
//            |       P2.1/TA1.1|---> PWM LEFT
//            |             P2.2|---> DIR LEFT
//            |       P2.4/TA1.2|---> PWM RIGHT
//            |             P2.3|---> DIR RIGHT
//            |                 |
//
//   Ing. Ondrej Hejda
//   May 2018
//******************************************************************************
#include <msp430.h>
#include <stdint.h>

#define LED_RED_ON() do{P1OUT|=0x01;}while(0)
#define LED_RED_OFF() do{P1OUT&=~0x01;}while(0)
#define LED_RED_SWAP() do{P1OUT^=0x01;}while(0)
#define LED_GREEN_ON() do{P1OUT|=0x40;}while(0)
#define LED_GREEN_OFF() do{P1OUT&=~0x40;}while(0)
#define LED_GREEN_SWAP() do{P1OUT^=0x40;}while(0)

#define CHANNELS 10

#define CHMAX  0x7D0
#define CENTER 0x5DC
#define CHMIN  0x3E8

uint16_t ch[CHANNELS];
int rxcnt = 0;

int main(void)
{
    //WDTCTL = WDTPW + WDTHOLD;        // Stop WDT
    WDTCTL = WDT_MDLY_32;                     // Set Watchdog Timer interval to ~30ms
    IE1 |= WDTIE;                             // Enable WDT interrupt
    if (CALBC1_1MHZ==0xFF) while(1); // If calibration constant erased do not load, trap CPU!!	
    DCOCTL = 0;                      // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;           // Set DCO
    DCOCTL = CALDCO_1MHZ;
    P1SEL = BIT1 + BIT2 ;            // P1.1 = RXD, P1.2=TXD
    P1SEL2 = BIT1 + BIT2;                      
    UCA0CTL1 |= UCSSEL_2;            // SMCLK
    UCA0BR0 = 8;                     // 1MHz 115200
    UCA0BR1 = 0;                     // 1MHz 115200
    UCA0MCTL = UCBRS2 + UCBRS0;      // Modulation UCBRSx = 5
    UCA0CTL1 &= ~UCSWRST;            // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                 // Enable USCI_A0 RX interrupt

    // setup LEDs
    P1DIR |= 0x41; P1OUT &= ~0x41;

    // start timer
    P2DIR |= 0x12;       // P2.1,4 output
    P2SEL |= 0x12;       // P2.1,4 option select
    TA1CCR0 = 1000;
    TA1CCTL1 = OUTMOD_6; // CCR1 toggle/set (P2.1)
    TA1CCR1 = 0;
    TA1CCTL2 = OUTMOD_6; // CCR2 toggle/set (P2.4)
    TA1CCR2 = 0;

    TA1CTL = TASSEL_2 + MC_1;             // SMCLK, upmode

    while(1) {
        __bis_SR_register(LPM0_bits + GIE); // Enter LPM0, interrupts enabled
        int left, right, ch0i, ch1i;
        ch0i = ch[0]-CENTER;
        ch1i = ch[1]-CENTER;
        left  = ch1i + ch0i;
        right = ch1i - ch0i;
        if (left>500)   left  =  500;
        if (left<-500)  left  = -500;
        if (right>500)  right =  500;
        if (right<-500) right = -500;
        if (left>0) {
            P2OUT &= ~0x04;
            if (left>20)
                TA1CCR1 = (left<<1);
            else
                TA1CCR1 = 0;
        }
        else {
            P2OUT |= 0x04;
            if (left<-20)
                TA1CCR1 = ((500+left)<<1);
            else
                TA1CCR1 = 1000;
        }
        if (right>0) {
            P2OUT &= ~0x08;
            if (right>20)
                TA1CCR2 = (right<<1);
            else
                TA1CCR2 = 0;
        }
        else {
            P2OUT |= 0x08;
            if (right<-20)
                TA1CCR2 = ((500+right)<<1);
            else
                TA1CCR2 = 1000;
        }
    }
    return 1;
}

// Watchdog Timer interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(WDT_VECTOR))) watchdog_timer (void)
#else
#error Compiler not supported!
#endif
{
    static int rxcnt_test = 0;
    if (rxcnt_test==rxcnt)
        LED_RED_OFF();
    rxcnt_test = rxcnt;
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
    uint8_t c = UCA0RXBUF;
    static int s = 0;
    static int i = 0;
    static int chn = 0;
    switch (s) {
    case 0:
        if (c==0x20) {
            i = 0;
            s++;
        }
        break;
    case 1:
        if (c==0x40)
            s++;
        else
            s = 0;
        break;
    case 2:
        if (i&1) {
            chn = i>>1;
            ch[chn] = c;
        }
        else
            ch[chn] |= (c<<8);
        i++;
        if (i>(CHANNELS*2)) {
            LED_RED_ON();
            s=0;
            rxcnt++;
            __bic_SR_register_on_exit(LPM0_bits);
        }
        break;
    default:
        s=0;
        break;
    }
}

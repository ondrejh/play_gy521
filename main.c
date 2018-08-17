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
//            |             P1.7|---> RED LED (RX ON)
//            |             P1.6|---> LED GND
//            |             P2.5|---> GREEN LED (PWM ON)
//            |                 |
//            |       P2.1/TA1.1|---> PWM LEFT
//            |             P2.2|---> DIR LEFT
//            |       P2.4/TA1.2|---> PWM RIGHT
//            |             P2.3|---> DIR RIGHT
//            |                 |
//            |       P1.5/TA0.0|---> SERVO 1
//            |       P1.6/TA0.1|---> SERVO 2
//            |                 |
//
//   Ing. Ondrej Hejda
//   May 2018
//******************************************************************************
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

/*#define LED_INIT() do{P1DIR|=0x41;P1OUT&=~0x41;}while(0)
#define LED_RED_ON() do{P1OUT|=0x01;}while(0)
#define LED_RED_OFF() do{P1OUT&=~0x01;}while(0)
#define LED_RED_SWAP() do{P1OUT^=0x01;}while(0)
#define LED_GREEN_ON() do{P1OUT|=0x40;}while(0)
#define LED_GREEN_OFF() do{P1OUT&=~0x40;}while(0)
#define LED_GREEN_SWAP() do{P1OUT^=0x40;}while(0)*/

#define LED_INIT() do{P1DIR|=0xC0;P2DIR|=0x20;P1OUT&=~0xC0;P2OUT&=~0x20;}while(0)
#define LED_RED_ON() do{P2OUT|=0x05;}while(0)
#define LED_RED_OFF() do{P2OUT&=~0x05;}while(0)
#define LED_RED_SWAP() do{P2OUT^=0x05;}while(0)
#define LED_GREEN_ON() do{P1OUT|=0x80;}while(0)
#define LED_GREEN_OFF() do{P1OUT&=~0x80;}while(0)
#define LED_GREEN_SWAP() do{P1OUT^=0x80;}while(0)

#define CHANNELS 10

#define CHMAX  0x7D0
#define CENTER 0x5DC
#define CHMIN  0x3E8

#define PWM_THOLD 50

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
    LED_INIT();

	P1DIR |= 0x60;
	P1OUT &= ~0x60;
	P1SEL |= 0x60;
	TA0CCTL0 = OUTMOD_1;
    TA0CCR0 = 20000;
    TA0CCR1 = 1500;
    TA0CCTL1 = OUTMOD_6;
    TA0CCR2 = 1500;
	TA0CCTL2 = CCIE;
	
    TA0CTL = TASSEL_2 + MC_1 + TAIE;             // SMCLK, upmode

    // start timer
    P2DIR |= 0x1E;       // P2.1,2,3,4 output
    P2OUT &= ~0x0C;
    P2SEL |= 0x12;       // P2.1,4 option select
    TA1CCR0 = 500;
    TA1CCTL1 = OUTMOD_6; // CCR1 toggle/set (P2.1)
    TA1CCR1 = 0;
    TA1CCTL2 = OUTMOD_6; // CCR2 toggle/set (P2.4)
    TA1CCR2 = 0;

    TA1CTL = TASSEL_2 + MC_1;             // SMCLK, upmode

    while(1) {
        __bis_SR_register(LPM0_bits + GIE); // Enter LPM0, interrupts enabled
        int left, right, ch0i, ch1i;
        bool led = false;

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
            if (left>PWM_THOLD) {
                TA1CCR1 = left;
                led = true;
            } else
                TA1CCR1 = 0;
        }
        else {
            P2OUT |= 0x04;
            if (left<-PWM_THOLD) {
                TA1CCR1 = 500+left;
                led = true;
            } else
                TA1CCR1 = 501;
        }

        if (right>0) {
            P2OUT &= ~0x08;
            if (right>PWM_THOLD) {
                TA1CCR2 = right;
                led = true;
            } else
                TA1CCR2 = 0;
        }
        else {
            P2OUT |= 0x08;
            if (right<-PWM_THOLD) {
                TA1CCR2 = 500+right;
                led = true;
            } else
                TA1CCR2 = 501;
        }

        if (led) LED_GREEN_ON();
        else LED_GREEN_OFF();
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
    if (rxcnt_test==rxcnt) {
        LED_RED_OFF();
        TA1CCR1 = 0;
        TA1CCR2 = 0;
        P2OUT &= ~0x0C;
    }
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
        if ((i&1)==0) {
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

// Timer_A2 Interrupt Vector (TA0IV) handler
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A2_VECTOR
__interrupt void Timer_A2(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) Timer_A2 (void)
#else
#error Compiler not supported!
#endif
{
	TA0CCTL0 &= ~0x04;
  /*switch( TA0IV )
  {
  case  2: CCR1 += 1000;                    // Add Offset to CCR1
           break;
  case 10: P1OUT ^= 0x01;                   // Timer_A3 overflow
           break;
 }*/
}

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
//            |             P2.7|---> RED LED (RX ON)
//            |             P2.6|---> GREEN LED (PWM ON)
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

#define LED_INIT() do{P2DIR|=0xC0;P2OUT|=0xC0;P2SEL&=~0xC0;P2SEL2&=~0xC0;}while(0)
#define LED_RED_ON() do{P2OUT|=0x80;}while(0)
#define LED_RED_OFF() do{P2OUT&=~0x80;}while(0)
#define LED_RED_SWAP() do{P2OUT^=0x80;}while(0)
#define LED_GREEN_ON() do{P2OUT|=0x40;}while(0)
#define LED_GREEN_OFF() do{P2OUT&=~0x40;}while(0)
#define LED_GREEN_SWAP() do{P2OUT^=0x40;}while(0)

#define CHANNELS 10

#define CHMAX  0x7D0
#define CENTER 0x5DC
#define CHMIN  0x3E8

#define PWM_THOLD 50
#define PWM_MAX 500
#define PWM_LIMIT 500

uint16_t ch[CHANNELS];
int rxcnt = 0;

void set_servo_1(uint16_t val) {
    TA0CCR1 = val;
}

bool set_pwm_1(int16_t val) {
	bool act = false;

	if (val>0) { // forward
		P2OUT &= ~0x04;
		if (val>PWM_THOLD) {
			TA1CCR1 = val;
			act = true;
		} else
			TA1CCR1 = 0;
	}

	else { // backward
		P2OUT |= 0x04;
		if (val<-PWM_THOLD) {
			TA1CCR1 = PWM_MAX+val;
			act = true;
		} else
			TA1CCR1 = PWM_MAX+1;
	}
	
	return act;
}

bool set_pwm_2(int16_t val) {
	bool act = false;
	
	if (val>0) {
		P2OUT &= ~0x08;
		if (val>PWM_THOLD) {
			TA1CCR2 = val;
			act = true;
		} else
			TA1CCR2 = 0;
	}
	else {
		P2OUT |= 0x08;
		if (val<-PWM_THOLD) {
			TA1CCR2 = PWM_MAX+val;
			act = true;
		} else
			TA1CCR2 = PWM_MAX+1;
	}

	return act;
}

void xy2lr(int16_t x, int16_t y, int16_t *l, int16_t *r) {
	int16_t ch0i = x-CENTER;
	int16_t ch1i = y-CENTER;
	int16_t left  = ch1i + ch0i;
	int16_t right = ch1i - ch0i;
	if (left>PWM_LIMIT)   left  =  PWM_LIMIT;
	if (left<-PWM_LIMIT)  left  = -PWM_LIMIT;
	if (right>PWM_LIMIT)  right =  PWM_LIMIT;
	if (right<-PWM_LIMIT) right = -PWM_LIMIT;
	*l = left;
	*r = right;
}

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
    TA0CCR0 = 20000;
    TA0CCR1 = 0;
    TA0CCTL1 = OUTMOD_7;
	
    TA0CTL = TASSEL_2 + MC_1;             // SMCLK, upmode

    // start timer
    P2DIR |= 0x1E;       // P2.1,2,3,4 output
    P2OUT &= ~0x0C;
    P2SEL |= 0x12;       // P2.1,4 option select
    TA1CCR0 = PWM_MAX;
    TA1CCTL1 = OUTMOD_6; // CCR1 toggle/set (P2.1)
    TA1CCR1 = 0;
    TA1CCTL2 = OUTMOD_6; // CCR2 toggle/set (P2.4)
    TA1CCR2 = 0;

    TA1CTL = TASSEL_2 + MC_1;             // SMCLK, upmode
	
	int loc_rxcnt = rxcnt;
	
    while(1) {
        __bis_SR_register(LPM0_bits + GIE); // Enter LPM0, interrupts enabled
		
		if (loc_rxcnt != rxcnt) { // new channel data arrived
			int16_t left, right;
			bool led = false;

			xy2lr(ch[0], ch[1], &left, &right);

			if (set_pwm_1(left) || set_pwm_2(right))
				led = true;

			if (led) LED_GREEN_ON();
			else LED_GREEN_OFF();
			
			TA0CCR1 = ch[2];
			
			loc_rxcnt = rxcnt;
		}
		else { // no input signal detected
			TA1CCR1 = 0;
			TA1CCR2 = 0;
			P2OUT &= ~0x0C;
			TA0CCR1 = 0;
			LED_RED_SWAP();
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
	static int cnt = 0;
    if (rxcnt_test == rxcnt) {
		if (cnt >= 10) {
			cnt = 0;
			__bic_SR_register_on_exit(LPM0_bits);
		}
		else cnt++;
    }
	else cnt = 0;

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
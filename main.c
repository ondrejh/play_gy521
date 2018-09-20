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

#define TXBUFLEN 64
uint8_t utxbuff[TXBUFLEN];
volatile uint16_t utxbuff_cnt = 0;
volatile uint16_t utxbuff_ptr = 0;

typedef struct {
    int16_t acX;
    int16_t acY;
    int16_t acZ;
    int16_t tmp;
    int16_t gyX;
    int16_t gyY;
    int16_t gyZ;
} mpu6050_data_t;

union {
    uint8_t bytes[14];
    mpu6050_data_t mpu6050;
} data;

uint8_t u2h(uint8_t u) {
    uint8_t su = u&0xF;
    if (su<10)
        return '0'+su;
    return 'A'-10+su;
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
        utxbuff[ptr++] = u2h(data[i]>>4);
        utxbuff[ptr++] = u2h(data[i]&0x0F);
    }
    utxbuff[ptr++] = '\n';
    utxbuff[ptr++] = '\r';
    uart_start_tx(ptr);
}

uint8_t put_word(uint8_t ptr, int16_t w)
{
    uint8_t p = ptr;
    int16_t ww = w;
    if (ww<0) {
        utxbuff[p++]='-';
        ww = -ww;
    }
    int i;
    uint8_t d[5];
    for (i=0;i<5;i++) {
        d[i] = ww%10;
        ww /= 10;
    }
    bool put=false;
    for (i=4;i>0;i--) {
        if (d[i])
            put = true;
        if (put)
            utxbuff[p++] = u2h(d[i]);
    }
    utxbuff[p++] = u2h(d[0]);
    return p;
}

void print_mpu6050(mpu6050_data_t data) {
    uint8_t ptr = 0;
    utxbuff[ptr++] = 'A';
    utxbuff[ptr++] = 'X';
    utxbuff[ptr++] = ':';
    ptr = put_word(ptr, data.acX);
    utxbuff[ptr++] = ' ';
    utxbuff[ptr++] = 'A';
    utxbuff[ptr++] = 'Y';
    utxbuff[ptr++] = ':';
    ptr = put_word(ptr, data.acY);
    utxbuff[ptr++] = ' ';
    utxbuff[ptr++] = 'A';
    utxbuff[ptr++] = 'Z';
    utxbuff[ptr++] = ':';
    ptr = put_word(ptr, data.acZ);
    utxbuff[ptr++] = ' ';
    utxbuff[ptr++] = 'G';
    utxbuff[ptr++] = 'X';
    utxbuff[ptr++] = ':';
    ptr = put_word(ptr, data.gyX);
    utxbuff[ptr++] = ' ';
    utxbuff[ptr++] = 'G';
    utxbuff[ptr++] = 'Y';
    utxbuff[ptr++] = ':';
    ptr = put_word(ptr, data.gyY);
    utxbuff[ptr++] = ' ';
    utxbuff[ptr++] = 'G';
    utxbuff[ptr++] = 'Z';
    utxbuff[ptr++] = ':';
    ptr = put_word(ptr, data.gyZ);
    utxbuff[ptr++] = '\n';
    utxbuff[ptr++] = '\r';
    uart_start_tx(ptr);
}

void print_message(const char* msg)
{
    int i, ptr=0;
    for (i=0;;i++) {
        uint8_t c = *msg++;
        if (c==0)
            break;
        utxbuff[ptr++] = c;
        if (ptr==TXBUFLEN)
            break;
    }
    uart_start_tx(ptr);
}

uint8_t i2c_read_byte(uint8_t adr)
{
    while (UCB0CTL1 & UCTXSTP);
	UCB0CTL1 |= UCTR + UCTXSTT; // I2C TX,START

	while (!(IFG2&UCB0TXIFG));
	UCB0TXBUF = adr;

	while (!(IFG2&UCB0TXIFG));

	UCB0CTL1 &= ~UCTR;          // I2C RX
	UCB0CTL1 |= UCTXSTT;        // I2C RESTART
	IFG2 &= ~UCB0TXIFG;

	while (UCB0CTL1 & UCTXSTT);
	UCB0CTL1 |= UCTXSTP;
	return UCB0RXBUF;
}

uint8_t i2c_read_data(uint8_t adr, uint8_t *data, uint8_t dlen)
{
    uint8_t i=0;
	while (UCB0CTL1 & UCTXSTP);     // Loop until I2C STT is sent
	UCB0CTL1 |= UCTR + UCTXSTT;     // I2C TX, start condition

	while (!(IFG2&UCB0TXIFG));
	IFG2 &= ~UCB0TXIFG;             // Clear USCI_B0 TX int flag
	if(UCB0STAT & UCNACKIFG) return UCB0STAT;
	UCB0TXBUF = adr;

	while (!(IFG2&UCB0TXIFG));
	if(UCB0STAT & UCNACKIFG) return UCB0STAT;

	UCB0CTL1 &= ~UCTR;              // I2C RX
	UCB0CTL1 |= UCTXSTT;            // I2C start condition
	IFG2 &= ~UCB0TXIFG;             // Clear USCI_B0 TX int flag
	while (UCB0CTL1 & UCTXSTT);     // Loop until I2C STT is sent
	for(i=0;i<(dlen-1);i++)
	{
		while (!(IFG2&UCB0RXIFG));
		IFG2 &= ~UCB0TXIFG;         // Clear USCI_B0 TX int flag
		data[i] = UCB0RXBUF;
	}
	while (!(IFG2&UCB0RXIFG));
	IFG2 &= ~UCB0TXIFG;             // Clear USCI_B0 TX int flag
	UCB0CTL1 |= UCTXSTP;            // I2C stop condition after 1st TX
	data[dlen-1] = UCB0RXBUF;
	IFG2 &= ~UCB0TXIFG;             // Clear USCI_B0 TX int flag
	return 0;
}

uint8_t i2c_write_byte(uint8_t adr, uint8_t data)
{
    while (UCB0CTL1 & UCTXSTP);
	UCB0CTL1 |= UCTR + UCTXSTT;

	while (!(IFG2&UCB0TXIFG));
	if(UCB0STAT & UCNACKIFG) return UCB0STAT;
	UCB0TXBUF = adr;


	while (!(IFG2&UCB0TXIFG));
	if(UCB0STAT & UCNACKIFG) return UCB0STAT;
	UCB0TXBUF = data;

	while (!(IFG2&UCB0TXIFG));
	if(UCB0STAT & UCNACKIFG) return UCB0STAT;
	UCB0CTL1 |= UCTXSTP;
	IFG2 &= ~UCB0TXIFG;
	return 0;
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
    //IE2 |= UCA0RXIE;      // Enable USCI_A0 RX interrupt

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
    //IE2 |= UCB0RXIE;                          // Enable RX interrupt

    // setup LEDs
    LED_INIT();

    i2c_write_byte(0x6B, 0x00);

    // main loop
    while(1) {
        LED_RED_SWAP();
        i2c_read_data(0x3B, data.bytes, 14);
        print_mpu6050(data.mpu6050);
        __bis_SR_register(LPM0_bits + GIE); // Enter LPM0, interrupts enabled
    }

    return -1;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCIAB0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    if (utxbuff_ptr < utxbuff_cnt) {
        UCA0TXBUF = utxbuff[utxbuff_ptr++];
    }
    else {
        IE2 &= ~UCA0TXIE;
        __bic_SR_register_on_exit(LPM0_bits);
    }
}


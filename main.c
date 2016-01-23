/*
 * File:   main.c
 * Author: Charles M Douvier
 * Contact at: http://iradan.com
 *
 * Created on Janurary 1, 2015
 *
 * Target Device:
 * 18F14K22 
 *
 * Project: RGB Resistor Clock
 *
 
 * Version:
 * 0.1  Configuration, with reset test
 * 0.2
 *
 */
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 16000000 //16Mhz FRC internal osc
#define __delay_us(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000000.0)))
#define __delay_ms(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000.0)))
#endif


#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


//config bits
#pragma config FOSC=IRC, WDTEN=OFF, PWRTEN=OFF, MCLRE=ON, CP0=OFF, CP1=OFF, BOREN=ON
#pragma config STVREN=ON, LVP=OFF, HFOFST=OFF, IESO=OFF, FCMEN=OFF

#define _XTAL_FREQ 16000000 //defined for delay

/*
 * Variables
 */

    int     device_present;             // 1 = 1-wire device on 1-wire bus
    int     i, x, y, int_temp, an4_value;               //
    long int    decm;
    int     itxdata, txdata;            //int RS232 tx data
    char    rxbuff[10], z[1], buf[4];                 //buffer for T-sense 1-wire device
    float    temperature, f, d;
    volatile unsigned int uart_data;    // use 'volatile' qualifer as this is changed in ISR

/*
 *  Functions
 */

    void interrupt ISR() {

    if (PIR1bits.RCIF)          // see if interrupt caused by incoming data .. unused currently
    {
        uart_data = RCREG;     // read the incoming data
        PIR1bits.RCIF = 0;      // clear interrupt flag
                                //
    }
    // I left this timer interrupt if I needed it later. This is unused.
    if (PIR1bits.TMR1IF)
    {
        //T1CONbits.TMR1ON = 0;
        
        PIR1bits.TMR1IF = 0;
        //T1CONbits.TMR1ON = 1;

    }
}


     void __delay_10ms(unsigned char n)     //__delay functions built-in can't be used for much at this speed... so!
 {
     while (n-- != 0) {
         __delay_ms(10);
     }
 }


void uart_send (unsigned int mydata_byte) {      //bytes

    while(!TXSTAbits.TRMT);    // make sure buffer full bit is high before transmitting
    TXREG = mydata_byte;       // transmit data
}

void write_uart(const char *txt)                //strings
{
                                //this send a string to the TX buffer
                                //one character at a time
       while(*txt)
       uart_send(*txt++);
}

//This code if from Microchip but is unused currently.
void uart_send_hex_ascii(unsigned char display_data)
{

	//unsigned char temp;
	//temp = ((display_data & 0xF0)>>4);
	//if (temp <= 0x09)
	//	Putchar(temp+'0');
	//else
	//	Putchar(temp+'0'+0x07);
        //
	//temp = display_data & 0x0F;
	//if (temp <= 0x09)
	//	Putchar(temp+'0');
	//else
	//	Putchar(temp+'0'+0x07);

	//Putchar('\r');
	//Putchar('\n');
}

void serial_init(void)
{

    // calculate values of SPBRGL and SPBRGH based on the desired baud rate
    //
    // For 8 bit Async mode with BRGH=0: Desired Baud rate = Fosc/64([SPBRGH:SPBRGL]+1)
    // For 8 bit Async mode with BRGH=1: Desired Baud rate = Fosc/16([SPBRGH:SPBRGL]+1)



    TXSTAbits.BRGH=1;       // select low speed Baud Rate (see baud rate calcs below)
    TXSTAbits.TX9=0;        // select 8 data bits
    TXSTAbits.TXEN=1;     // enable transmit
    BAUDCONbits.BRG16=0;

    RCSTAbits.SPEN=1;       // serial port is enabled
    RCSTAbits.RX9=0;        // select 8 data bits
    RCSTAbits.CREN=1;       // receive enabled


    SPBRG=25;               //38,400bps-ish
                            //BRG16=0, 7=31.25k, 25=9.615k

    PIR1bits.RCIF=0;        // make sure receive interrupt flag is clear
    PIE1bits.RCIE=1;        // enable UART Receive interrupt


         __delay_ms(10);        // give time for voltage levels on board to settle

}


void init_io(void) {
    ANSEL = 0x00;         
    ANSELH = 0x00;

    TRISAbits.TRISA0 = 0; // output
    TRISAbits.TRISA1 = 0; // output
    TRISAbits.TRISA2 = 0; // output
    TRISAbits.TRISA4 = 0; // output
    TRISAbits.TRISA5 = 0; // output



    TRISBbits.TRISB4 = 0; // output
    TRISBbits.TRISB5 = 1; // input (RX UART)
    TRISBbits.TRISB6 = 0; // output
    TRISBbits.TRISB7 = 0; // output (TX UART)

    LATC = 0x00;

    TRISCbits.TRISC0 = 1; // 
    TRISCbits.TRISC1 = 1; // 
    TRISCbits.TRISC2 = 0; // 
    TRISCbits.TRISC3 = 0; // 
    TRISCbits.TRISC4 = 0; // 
    TRISCbits.TRISC5 = 0; // output
    TRISCbits.TRISC6 = 1; // input
    TRISCbits.TRISC7 = 1; // input

}


int main(void) {

    init_io();

    // set up oscillator control register, using internal OSC at 16MHz.
    OSCCONbits.IRCF = 0x07; //set OSCCON IRCF bits to select OSC frequency 16MHz
    OSCCONbits.SCS = 0x00; //set the SCS bits to select internal oscillator block
    OSCTUNEbits.PLLEN = 0x01;   //x4 PLL
    //RCONbits.IPEN = 0;          //dsiable priority levels

    INTCONbits.PEIE = 1;        // Enable peripheral interrupt
    INTCONbits.GIE = 1;         // enable global interrupt


    while (1) {


        LATAbits.LATA0 = 0; //this is just for debugging with an LA..

        #asm 
                NOP
        #endasm

        LATAbits.LATA0 = 1; //also confirms oscillator setup is correct.. 1us width
  
        #asm 
                NOP
        #endasm
                
        LATAbits.LATA0 = 0;
        LATAbits.LATA0 = 1;
        LATAbits.LATA0 = 0;
            __delay_10ms(1);
        

    }
    return (EXIT_SUCCESS);
}

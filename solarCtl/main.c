/*
file: controller.c
author: ON4JRT jean.taeymans@telenet.be
date: 20170509
version: 0.92

version:  0.92   20170509   working version
version:  0.91   20170422   tests with hardware
version:  0.90   20160501   initial coding

SOLAR CONTROLLER
================
Little system to control the remote HF transceiver, its battery (11.1V 13Ah, Li/Ion) and a small solar panel
It keeps the time (hours/minutes) in order to switch the receiver on/off at preset times
A SW UART at 2400 baud to communicate with the µBIT-X controller when active is also implemented

The following commands are used over the UART interface:
- H?	responds with the "Hello ... " message
- B? 	responds with the battery voltage, the temperature, the solar panel switch status and the load switch status
- T?    responds with the current time
- Thhmm sets the current time, hours and minutes
- A?    responds with the start time, i.e. when the load is switched on
- Ahhmm sets the start time, hours and minutes
- S?    responds with the stop time, i.e. when the load is switched off
- Shhmm sets the stop time, hours and minutes
- R?    (reboot) wait a bit then switches the load off, wait some more then switches the load on again
- K?	(kill) wait a bit then switches the load off 

When the load switch is switched on, the solar contyroller expects the current time to be updated within the
next 30s, if this is not the case it will perform a "reboot".  If this fails 3 times, it will perform a "kill".

The system is based on a ATtiny84A controller running on a 4915200Hz crystal.
TCNT0 and TCNT1 are used with a /8 prescaler, overflowing at 2400Hz providing the
timing for the SW UART TD and RD respectively.
TCNT0 is furthermore used for the sec/min/hour counter chain.

The pins of the ATtinu84A is used as follows:
pin 1   VCC         3.3V power
pin 2   Xtal        4915200Hz crystal
pin 3   Xtal        4915200Hz crystal
pin 4   RES         used by ISP
pin 5   PB2/INT0    SW UART RD, startbit low
pin 6   PA7         SW UART TD, startbit low
pin 7   MOSI        used by ISP
pin 8   MISO        used by ISP
pin 9   SCL         used by ISP
pin 10  PA3         red led, active low
pin 11  PA2         solar-battery switch, active high
pin 12  PA1         battery-load switch, active high
pin 13  ADC0        battery voltage sense, 68.1k/5.11k divider
pin 14  GND

 */

#include <avr/io.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#define ONE_BIT 255			// 2400 baud
#define ONE_HALF_BIT 127
#define BUF_LEN 32

volatile signed int bat_volt = 0, temp = 0;
volatile unsigned int tick = 0;
volatile unsigned char sec = 0, old_sec = 0;
volatile unsigned char min = 0, a_min = 0, s_min = 0;
volatile unsigned char hour = 0, a_hour = 20, s_hour = 23;;
volatile unsigned char SwUartTXState = 0;
unsigned char rd[BUF_LEN];	// data received from µBIT-X controller
volatile unsigned int rd_rd_pnt = 0;
volatile unsigned int rd_wt_pnt = 0;
unsigned char wt[BUF_LEN];	// data send to µBIT-X controller
volatile unsigned int wt_rd_pnt = 0;
volatile unsigned int wt_wt_pnt = 0;
char cmd_buf[BUF_LEN];
volatile unsigned int cmd_buf_pnt = 0;
volatile unsigned char cmd_flag = 0;
volatile unsigned char SwUartRXBitCount = 0;
volatile unsigned char SwUartRXData = 0;
volatile unsigned char SwUartTXData = 0;
volatile unsigned char SwUartTXBitCount = 0;
volatile unsigned char tmp_char;
char tmp_buf[16];
const char hello[] PROGMEM = "Hello from controller !\r\n\0";
const char error[] PROGMEM = "Command error\r\n\0";
const char kill[] PROGMEM = "Bye bye\r\n\0";
const char boot[] PROGMEM = "Rebooting\r\n\0";
signed int i, j;
volatile unsigned char low_flag = 0, time_set_flag = 0, ten_sec_flag = 0;
volatile unsigned char blink_flag = 1, boot_flag = 0, kill_flag = 0;
volatile signed char boot_cnt = 3;

// **********************************************************************************************
// a few low level functions
// ********************************************************
void pgm_out_str(const char *strng)
	{
	tmp_char = pgm_read_byte(strng);
	while (tmp_char)
		{
		wt[wt_wt_pnt] = tmp_char;
		wt_wt_pnt++;
		if(wt_wt_pnt == BUF_LEN) wt_wt_pnt = 0;
		strng++;
		tmp_char = pgm_read_byte(strng);
		}
	}

void out_str(const char *strng)
	{
	unsigned int i = 0;
	while (tmp_buf[i])
		{
		wt[wt_wt_pnt] = tmp_buf[i];
		wt_wt_pnt++;
		if(wt_wt_pnt == BUF_LEN) wt_wt_pnt = 0;
		i++;
		}
	}

// **********************************************************************************************
// SW UART transmitter & time keeping service routine
// ********************************************************
ISR (TIM0_OVF_vect)
	{
	// time keeping, adjust the time
	tick++;
	if (tick == 2400)
        {
        tick = 0;
        sec++;
        if (sec == 60)
            {
            sec = 0;
            min++;
            if (min == 60)
                {
                min = 0;
                hour++;
                if (hour == 24)
                    {
                    hour = 0;
                    }
                }
            }
        }

	// SW UART transmit
	if (!SwUartTXState)
		{
		if(wt_rd_pnt != wt_wt_pnt)
			{
			PORTA &= ~(1<<7);  			// send the start bit
			SwUartTXState = 1;
 			SwUartTXBitCount = 0;
			SwUartTXData = wt[wt_rd_pnt] ;
			wt_rd_pnt++;
			if (wt_rd_pnt == BUF_LEN) wt_rd_pnt = 0;
 			}
		}
	else
	    {
		if( SwUartTXBitCount < 8 )
			{
      		if( SwUartTXData & 0x01 )
				{         		// if the LSB of the TX buffer is 1:
        		PORTA |= (1<<7);        // Send a logic 1 on the TX_PIN.
      			}
      		else
	  			{
        		PORTA &= ~(1<<7);        // Send a logic 0 on the TX_PIN.
      			}
      		SwUartTXData = SwUartTXData >> 1;  // bitshift the TX buffer and
      		SwUartTXBitCount++;                // increment TX bit counter.
			}
		else
			{
			PORTA |= (1<<7);        // send the stop bit
			SwUartTXState = 0;
			}
		}
	}

// SW UART receiver service routines
// *************************************
ISR (TIM1_COMPA_vect)
	{
    if( SwUartRXBitCount < 9 ) //Receiving, LSB first.
		{
        SwUartRXBitCount++;
        SwUartRXData = (SwUartRXData>>1);   // prepare for the next bit .
        if((PINB & (1 << 2)) != 0 )
			{
            SwUartRXData |= 0x80;       // if a 1 is read, then write one
		   	}
		OCR1A = ONE_BIT;
    	}
    else
		{								// one byte received
		rd[rd_wt_pnt] = SwUartRXData;
		rd_wt_pnt++;				// store in buffer
		if (rd_wt_pnt == BUF_LEN) rd_wt_pnt = 0;
        TIMSK1 &= ~( 1<< OCIE1A );      // disable the interrupt.
        GIFR |= (1 << INTF0);           // reset flag, not to enter the ISR one extra time.
        GIMSK |= ( 1<< INT0 );     	    // enable interrupt to receive next bytes.
    	}
	}

ISR (EXT_INT0_vect)  		// start bit detection service routine
	{
	GIMSK &= ~(1<< INT0 );
    SwUartRXBitCount = 0;
	TCNT1 = 0;
	OCR1A = ONE_HALF_BIT;
	TIFR1 |= (1 << OCF1A);
	TIMSK1 |= ( 1<< OCIE1A );
	}


// **********************************************************************************************
// main program
// **********************************************************************************************

int main(void)
    {
	// code at start up
	// ******************************************************************************************

	// configuration of the ports
	DDRA = (1<<1) | (1<<2) | (1<<3) | (1<<7);
	PORTA = (1<<3) | (1<<7);

	// configure the analog to digital converter,
	ADMUX = 0x80;	// use ADC0 as input & use internal 1.1V reference
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1); // set the clock to 77kHz, a conversion in 170µs
	DIDR0 = (1<<ADC0D); // disable digital port at PA0

	// configure external interrupt
	MCUCR = (1<<ISC01);  // falling edge
	// configure the timer/counter 0 prescaler /8, no interrupts for now,
	TCCR0B = (1<<CS01); // to be used for the SW-UART transmit
	// configure the timer/counter 1 prescaler /8, no interrupts for now, clear timer on compare
	TCCR1B = (1<<WGM12) | (1<<CS11); // to be used for the  SW-UART receive
	OCR1A = ONE_BIT;

	// start the whole thing
	TIMSK0 |= ( 1<< TOIE0 ); // enable timer interrupt for the SW-UART transmit
 	GIMSK |= ( 1<< INT0 );    // enable external interrupt to receive on SW-UART
	sei();

	// wait a bit before switching the load
	PORTA &= ~(1<<1);	// load off
	PORTA &= ~(1<<2);	// solar off
	while(sec != 5); 	// wait for 5s
	PORTA |= (1<<1);	// set load on at first

	// *****************************************
	// recurrent part of the program
	// *****************************************
	while(1)		// endless loop
        {

		if (old_sec != sec)
			{
			old_sec = sec;

			// blinking the led with a 2s period and performing the AD conversions
			if (blink_flag)
				{
				// check the battery voltage with the internal 10bit ADC,
				// each bit is 15.39mV
				ADCSRA |= (1<<ADSC);  // start a conversion
				while (!(ADCSRA & (1<<ADIF)));  // wait for the end of the conversion, takes about 177µs
				bat_volt = (int)((float)ADC * 15.14);
				ADMUX = 0x80;	// use ADC8 as next input for the chip temperature
				if (!time_set_flag) PORTA &= ~(1<<3);  // led on
				blink_flag = 0;
				}
			else {
				// check the temperature with the internal sensor
				// each bit is 1°C
				ADCSRA |= (1<<ADSC);  // start a conversion
				while (!(ADCSRA & (1<<ADIF)));  // wait for the end of the conversion, takes about 177µs
				temp = ADC - 264;
				ADMUX = 0xa2;	// use ADC0 as next input for the battery voltage
				PORTA |= (1<<3);  // led off
				blink_flag = 1;
				}

			// spiel with starting & ending times
			if ((sec == 0) && (min == a_min) && (hour == a_hour) && !low_flag) {
				PORTA |= (1<<1);
				time_set_flag = 0;
				boot_cnt = 3;
				}
			if ((sec == 0) && (min == s_min) && (hour == s_hour)) PORTA &= ~(1<<1);

			// spiel with the battery
			// a Li-Ion battery is considered full when 4.1V per cell is reached
			// likewise a battery is considered empty when 3.1V per cell is reached
			// switching the solar panel on and off with some hysteresis
			if (bat_volt > 12300) PORTA &= ~(1<<2);
			if (bat_volt < 12200) PORTA |= (1<<2);
			// inhibiting the load to switch on when battery is empty
			// for some reason the first battery measurements are incorrect, so
			// we inhibit the check until 10s after a reset
			if(sec == 10) ten_sec_flag = 1;  // set the then sec flag
			if ((bat_volt < 9300) && ten_sec_flag) low_flag = 1;
			if (bat_volt > 9600) low_flag = 0;
			if((PORTA & (1<<1)) && low_flag) PORTA &= ~(1<<1);

			// spiel with kill & reboot
			if (kill_flag > 0)
				{
				kill_flag--;
				if (kill_flag == 0) PORTA &= ~(1<<1);
				} // a kill command was given
			if (boot_flag > 0)
				{ // a boot command was given
				boot_flag--;
				if (boot_flag == 5) PORTA &= ~(1<<1);
				if (boot_flag == 0) PORTA |= (1<<1);
				}
			if((sec == 59) && !time_set_flag)
				{  // no time update within 1 min, hence reboot
				boot_cnt--;
				if (boot_cnt > 0) boot_flag = 10;
				else if (boot_cnt == 0) kill_flag = 5;  // number of reboots is exhausted
				else boot_cnt = 0;
				}
			}
		// check if a command was received
		while(rd_rd_pnt != rd_wt_pnt)
			{
 			// read from rd buffer0
			cmd_buf[cmd_buf_pnt] = rd[rd_rd_pnt];
			if(rd[rd_rd_pnt] == '\r') cmd_flag = 1; //testing on CR
			rd_rd_pnt++;
			if(rd_rd_pnt == BUF_LEN) rd_rd_pnt = 0;
			cmd_buf_pnt++;
			if (cmd_buf_pnt == BUF_LEN) cmd_buf_pnt = BUF_LEN - 1;
			}

		if(cmd_flag)		// a command was received
			{
			switch(cmd_buf[0])
				{
				case 'H':		// say hello
				case 'h':

				switch(cmd_buf_pnt)
					{
					case 3:
					i = 0;
					pgm_out_str(hello);
					break;

					default:
					pgm_out_str(error);
					break;
					}
				break;

				case 'B':		// request battery and temperature
				case 'b':

				switch(cmd_buf_pnt)
					{
					case 3:
					// battery voltage in mV
					itoa(bat_volt, tmp_buf, 10);
					j = strlen(tmp_buf);
					for(i = j - 1; i >= 0; i--) tmp_buf[i + 2] = tmp_buf[i];
					tmp_buf[0] = 'B';
					tmp_buf[1] = '=';
					tmp_buf[j + 2] = ' ';
					tmp_buf[j + 3] = '\0';
					out_str(tmp_buf);
					// temperature in °C
				    itoa(temp, tmp_buf, 10);
					j = strlen(tmp_buf);
					for(i = j - 1; i >= 0; i--) tmp_buf[i + 2] = tmp_buf[i];
					tmp_buf[0] = 'T';
					tmp_buf[1] = '=';
					tmp_buf[j + 2] = ' ';
					tmp_buf[j + 3] = '\0';
					out_str(tmp_buf);
					// status info of both power switches
					tmp_buf[0] = 'S';
					tmp_buf[1] = '=';
					if (PORTA & (1<<2)) tmp_buf[2] = '1';
					else tmp_buf[2] = '0';
					tmp_buf[3] = ' ';
					tmp_buf[4] = 'L';
					tmp_buf[5] = '=';
					if (PORTA & (1<<1)) tmp_buf[6] = '1';
					else tmp_buf[6] = '0';
					tmp_buf[7] = '\r';
					tmp_buf[8] = '\n';
					tmp_buf[9] = '\0';
					out_str(tmp_buf);
					break;

					default:
					pgm_out_str(error);
					break;
					}
				break;

				case 'T':		// set and tell current time
				case 't':

				switch(cmd_buf_pnt)
					{
					case 6:
					cmd_buf[5] = '\0';
					min = atoi(&cmd_buf[3]);
					if (min > 59) min = 59;
					cmd_buf[3] = '\0';
					hour = atoi(&cmd_buf[1]);
					if (hour > 23) hour = 23;
					time_set_flag = 1;  // time has been set, no more blinking

					case 3:
					    itoa(hour, tmp_buf, 10);
						j = strlen(tmp_buf);
						if (j == 1)
							{

							tmp_buf[1] = tmp_buf[0];
							tmp_buf[0] = '0';
							}
						tmp_buf[2] = ':';
					    itoa(min, &tmp_buf[3], 10);
						j = strlen(tmp_buf);
						if (j == 4)
							{
							tmp_buf[4] = tmp_buf[3];
							tmp_buf[3] = '0';
							}
						tmp_buf[5] = '\0';
						for(i = 5; i >= 0; i--) tmp_buf[i + 2] = tmp_buf[i];
						tmp_buf[0] = 'T';
						tmp_buf[1] = '=';
						tmp_buf[7] = '\r';
						tmp_buf[8] = '\n';
						tmp_buf[9] = '\0';
						out_str(tmp_buf);
						break;

					default:
					pgm_out_str(error);
					break;
					}
				break;

				case 'A':		// set and tell starting time
				case 'a':

				switch(cmd_buf_pnt)
					{
					case 6:
					cmd_buf[5] = '\0';
					a_min = atoi(&cmd_buf[3]);
					if (a_min > 59) a_min = 59;
					cmd_buf[3] = '\0';
					a_hour = atoi(&cmd_buf[1]);
					if (a_hour > 23) a_hour = 23;

					case 3:
					    itoa(a_hour, tmp_buf, 10);
						j = strlen(tmp_buf);
						if (j == 1)
							{

							tmp_buf[1] = tmp_buf[0];
							tmp_buf[0] = '0';
							}
						tmp_buf[2] = ':';
					    itoa(a_min, &tmp_buf[3], 10);
						j = strlen(tmp_buf);
						if (j == 4)
							{
							tmp_buf[4] = tmp_buf[3];
							tmp_buf[3] = '0';
							}
						tmp_buf[5] = '\0';
						for(i = 5; i >= 0; i--) tmp_buf[i + 2] = tmp_buf[i];
						tmp_buf[0] = 'A';
						tmp_buf[1] = '=';
						tmp_buf[7] = '\r';
						tmp_buf[8] = '\n';
						tmp_buf[9] = '\0';
						out_str(tmp_buf);
						break;

					default:
					pgm_out_str(error);
					break;
					}
				break;

				case 'S':		// set and tell ending time
				case 's':

				switch(cmd_buf_pnt)
					{
					case 6:
					cmd_buf[5] = '\0';
					s_min = atoi(&cmd_buf[3]);
					if (s_min > 59) s_min = 59;
					cmd_buf[3] = '\0';
					s_hour = atoi(&cmd_buf[1]);
					if (s_hour > 23) s_hour = 23;

					case 3:
					    itoa(s_hour, tmp_buf, 10);
						j = strlen(tmp_buf);
						if (j == 1)
							{

							tmp_buf[1] = tmp_buf[0];
							tmp_buf[0] = '0';
							}
						tmp_buf[2] = ':';
					    itoa(s_min, &tmp_buf[3], 10);
						j = strlen(tmp_buf);
						if (j == 4)
							{
							tmp_buf[4] = tmp_buf[3];
							tmp_buf[3] = '0';
							}
						tmp_buf[5] = '\0';
						for(i = 5; i >= 0; i--) tmp_buf[i + 2] = tmp_buf[i];
						tmp_buf[0] = 'S';
						tmp_buf[1] = '=';
						tmp_buf[7] = '\r';
						tmp_buf[8] = '\n';
						tmp_buf[9] = '\0';
						out_str(tmp_buf);
						break;

					default:
					pgm_out_str(error);
					break;
					}
				break;

				case 'K':		// kill now
				case 'k':

				switch(cmd_buf_pnt)
					{
					case 3:
					i = 0;
					pgm_out_str(kill);
					kill_flag = 5; // shut down the subsystem in 5s
					break;

					default:
					pgm_out_str(error);
					break;
					}
				break;
				case 'R':		// kill now
				case 'r':

				switch(cmd_buf_pnt)
					{
					case 3:
					i = 0;
					pgm_out_str(boot);
					boot_flag = 10; // reboot the subsystem in 10s
					break;

					default:
					pgm_out_str(error);
					break;
					}
				break;

			default:
				pgm_out_str(error);
				break;
				}

			cmd_buf_pnt = 0;		// ready for next command
			cmd_flag = 0;
			rd_rd_pnt = rd_wt_pnt;  // flush input buffer
			} // end command handling

        }	// end endless loop
    return 0;
    }



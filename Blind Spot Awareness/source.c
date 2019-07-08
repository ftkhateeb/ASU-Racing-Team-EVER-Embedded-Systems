/*
 * BlindSpotAwarness.c
 *
 * Created: 7/8/2019 10:41:08 PM
 * Author : Ali
 */ 
#define F_CPU 1000000      

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>

#define TIMER1_ICP1 0 

static volatile uint8_t pulse_length = 0; //Length of pulse that came back from ECHO in the ultrasonic sensor. 

int main(void)
{
	sei(); //Enable global interrupts
    /* Replace with your application code */
    while (1) 
    {
    }
}


/*
* 8-bit timer
* Init timer in Normal Mode
*  
*/
void icu_init(void)
{
	DDRB &= ~(1 << TIMER1_ICP1); //Set PB0 (ICP1) as I/P
	TCCR1B = 0;
	TCCR1B = (1 << ICES1) | (1 << CS10);
	/*
	* WGM13 = 0, WGM12 = 0.  Normal Mode
	* ICNC1 = 0. Disable Input Capture Noise Filter
	* ICES1 = 1. Initially set the input capture at rising edge (will be toggled at each capture)
	* CS12 CS11 CS10 (0 0 1), set clock to CPU_CLOCK with no prescale.
	*/
	
	TIMSK1 |= (1 << ICIE1); //Enable interrupts on Input Capture for timer1
	
}


ISR(TIMER1_CAPT_vect)
{
	
}

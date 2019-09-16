/*
 * BlindSpotAwarness.c
 *
 * Created: 7/8/2019 10:41:08 PM
 * Author : Ali/Khateeb
 */ 
#define F_CPU 1000000      

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include "lcd.h"
#include <math.h>

#define TIMER1_ICP1 0 

enum edge {RISING, FALLING};

static volatile uint16_t pulse_length_g = 0; //Length of pulse that came back from ECHO in the ultrasonic sensor.
static volatile uint16_t t1_g = 0; //Time of occurrence of rising edge 
static volatile uint16_t t2_g = 0; //Time of occurrence of falling edge
static volatile enum edge current_edge_g = RISING;

void icu_init(void);

int main(void)
{
	sei(); //Enable global interrupts
    /* Replace with your application code */
	LCD_init(); /* initialize LCD */
	LCD_displayString("Period is = ");
	icu_init();
    while (1) 
    {
	    if(t2_g > t1_g)
	    {
		pulse_length_g = t2_g - t1_g;
		LCD_goToRowColumn(1, 0);
		LCD_intgerToString(pulse_length_g);
    
	    }
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
	* CS12 CS11 CS10 (0 0 1), set clock to CPU_CLOCK.
	*/
	TIFR1 |= (1<<ICF1); //Clear input capture flag
	TIMSK1 |= (1 << ICIE1); //Enable interrupts on Input Capture for timer1
	
}


ISR(TIMER1_CAPT_vect)
{
	if(current_edge_g == RISING)
	{
		TCCR1B &= ~(1 << ICES1); //Set input capture at falling edge
		t1_g = ICR1;
		current_edge_g = FALLING;
	}
	else if(current_edge_g == FALLING)
	{
		TCCR1B |= (1 << ICES1); //Set input capture at rising edge
		t2_g = ICR1;
		current_edge_g = RISING;
		TCNT1 = 0; //Reset timer
	}
}

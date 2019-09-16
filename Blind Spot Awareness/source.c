  
/*
 * BlindSpotAwarness.c
 *
 * Created: 7/8/2019 10:41:08 PM
 * Author : Ali
 */ 
#define F_CPU 1000000UL      

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
//#include "lcd.h"
#include <math.h>

#define TIMER1_ICP1 0
#define DISTANCE(TIME) ((17150 * (float)TIME))  //Speed of sound * time = distance, but the sound goes and comes (so it goes the distance two times) so divide by 2

enum edge {RISING, FALLING};


static volatile uint16_t t1_g = 0; //Time of occurrence of rising edge 
static volatile uint16_t t2_g = 0; //Time of occurrence of falling edge
static volatile enum edge current_edge_g = RISING;

void icu_init(void);

int main(void)
{
	float time = 0;
	float distance = 0;
	uint32_t pulse_ticks_g = 0; //Number of clock ticks that passed during "high" part of the echo signal.
	sei(); //Enable global interrupts
	//LCD_init(); /* initialize LCD */
	//LCD_displayString("Distance = ");
	icu_init();
	
	DDRB |= (1 << PORTB1); //Trigger pin, O/P
	DDRB |= (1 << PORTB4);
    while (1) 
    {
		/* Sensor is triggered by 10 us high signal, give low first to ensure clean high */
		PORTB &= ~(1 << PORTB1);
		_delay_us(5);
		PORTB |= (1 << PORTB1);
		_delay_us(15);
		PORTB &= ~(1 << PORTB1);
		if(t2_g > t1_g)
			pulse_ticks_g = t2_g - t1_g;
		else
			pulse_ticks_g = (65535 - (uint32_t)t1_g + t2_g);
		time = (float)pulse_ticks_g * 1024 / F_CPU;
		distance = DISTANCE(time);
		
		if(distance < 2500) //2.5 meters
			PORTB |= (1 << PORTB4);
		else
			PORTB &= ~(1 << PORTB4);
		
		
	//	LCD_goToRowColumn(1, 0);
	//	LCD_floatToString(DISTANCE(time));
		_delay_ms(100);
    }
}


/*
* 16-bit timer
* Init timer in Input Capture Mode
*  
*/
void icu_init(void)
{
	DDRB &= ~(1 << TIMER1_ICP1); //Set PB0 (ICP1) as I/P
	TCCR1B = 0;
	TCCR1B = (1 << ICES1) | (1 << CS12) | (1 << CS10);
	/*
	* WGM13 = 0, WGM12 = 0.  Normal Mode
	* ICNC1 = 0. Disable Input Capture Noise Filter
	* ICES1 = 1. Initially set the input capture at rising edge (will be toggled at each capture)
	* CS12 CS11 CS10 (1 0 1), set clock to CPU_CLOCK/1024.
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

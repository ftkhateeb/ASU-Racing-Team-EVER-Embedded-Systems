//#include "lcd.h"
#include "common_macros.h"

void ICUInit(void);
void PCINT0Init(void);
void INT0Init(void);
void Timer0OVFInit(void);

#define LEFTSIGNAL_IN PORTD0
#define RIGHTSIGNAL_IN PORTD1
#define WAITINGSIGNAL_IN PORTD2
#define LEFTSIGNAL_IN PORTB0
#define RIGHTSIGNAL_IN PORTB1
#define WAITINGSIGNAL_IN PORTB2
#define BRAKES_IN PORTD2 // It can only be PORTD2(INT0) or PORTD3(INT1)

#define LEFTSIGNAL_OUT PORTD3
#define RIGHTSIGNAL_OUT PORTD4
#define LEFTSIGNAL_OUT PORTD5
#define RIGHTSIGNAL_OUT PORTD6
#define BRAKES_OUT PORTD0


enum SignalDirection {Forward, Waiting, Left, Right};

volatile enum SignalDirection signal_state = Forward;
volatile uint8_t portbhistory_signals = 0xFF;     // default is high because the pull-up
volatile uint8_t brakes_state = 0; // 0 is low, 1 is high

enum SignalDirection signal_state = Forward;

int main(void)
{
	DDRD &= ~(1 << LEFTSIGNAL_IN | 1 << RIGHTSIGNAL_IN | 1 << WAITINGSIGNAL_IN);  // Set them as input
	DDRD |= (1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT); // Set them as output
	DDRB |= (1 << PORTB3);
	PORTB &= ~(1 << PORTB3);
	DDRB &= ~(1 << PORTB4);
	ICUInit();
	Timer0OVFInit();
	sei();
	DDRD |= (1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT | 1 << BRAKES_OUT); // Set them as output
	PCINT0Init();
	INT0Init();
	Timer0OVFInit();
    /* Replace with your application code */
    while (1) 
    {
		if(BIT_IS_SET(PORTD, PORTB4))
		{
			PORTB |= (1 << PORTB3);
		}	
		///////////////////// Turn Signals & Waiting ////////////////////////////////
		/*if(BIT_IS_SET(PORTD, WAITINGSIGNAL_IN))
		{
			if((BIT_IS_SET(PORTD, LEFTSIGNAL_OUT) && BIT_IS_SET(PORTD, RIGHTSIGNAL_OUT)) || 
			   (BIT_IS_CLEAR(PORTD, LEFTSIGNAL_OUT) && BIT_IS_CLEAR(PORTD, RIGHTSIGNAL_OUT)) )
			   {
				   PORTD &= ~(1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT); //Make sure both are the same state to avoid desync between the right & left signals.
			   }
			_delay_ms(25); // Debounce time
			if(BIT_IS_SET(PORTD, WAITINGSIGNAL_IN)) { signal_state = Waiting; }
		}
		else if(BIT_IS_SET(PORTD, LEFTSIGNAL_IN))
		{
			_delay_ms(25); // Debounce time
			if(BIT_IS_SET(PORTD, LEFTSIGNAL_IN)) { signal_state = Left; }
		}
		else if(BIT_IS_SET(PORTD, RIGHTSIGNAL_IN))
		{
			_delay_ms(25); // Debounce time
			if(BIT_IS_SET(PORTD, RIGHTSIGNAL_IN)) { signal_state = Right; }
		}
		else
		{
			signal_state = Forward;
		}*/
		///////////////////////////////////////////////////////////////////////////


		/////////////////////////// Headlights ////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////



		/////////////////////////// Brakes lights ///////////////////////////////////

		////////////////////////////////////////////////////////////////////////////
    }
}

void ICUInit(void)

void PCINT0Init(void)
{
	DDRB &= ~(1 << LEFTSIGNAL_IN | 1 << RIGHTSIGNAL_IN | 1 << WAITINGSIGNAL_IN);  // Set them as input
	PORTB |= (1 << LEFTSIGNAL_IN | 1 << RIGHTSIGNAL_IN | 1 << WAITINGSIGNAL_IN); // Turn internal pull up resistors on
	PCICR |= (1 << PCIE0); // Enable Pin Change scan on PCINT[7:0]
	PCMSK0 |= (1 << PCINT0 | 1 << PCINT1 | 1 << PCINT2); // Enable pin change interrupt for the individual pins (they all call the same ISR)
	// PCINT0 --> LEFTSIGNAL_IN
	// PINCT1 --> RIGHTSIGNAL_IN
	// PCINT2 --> WAITING_IN

}

void INT0Init(void)
{
	DDRB &= ~(1 << PORTB0); //Set PB0 (ICP1) as I/P
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
	DDRD &= ~(1 << PORTD2);  // Set it as I/P
	PORTD |= (1 << PORTD2);  // Turn on pull up resistor
	EICRA |= (1 << ISC00);   // Any logical change on INT0 generates an interrupt request.
	EICRA &= ~(1 << ISC01);
	EIMSK |= (1 << INT0);
}

void Timer0OVFInit(void)
@@ -117,58 +86,82 @@ ISR(TIMER0_OVF_vect)
{
	if(signal_state == Waiting)
	{
		if(!(((BIT_IS_SET(PIND, LEFTSIGNAL_OUT) && BIT_IS_SET(PIND, RIGHTSIGNAL_OUT))) ||
		(BIT_IS_CLEAR(PIND, LEFTSIGNAL_OUT) && BIT_IS_CLEAR(PIND, RIGHTSIGNAL_OUT))))
		{
			PORTD &= ~(1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT); // Make sure both are the same state to avoid desync between the right & left signals.
		}
		PORTD ^= (1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT);
	}
	else if(signal_state == Left)
	{
		PORTB |= (1 << PORTB3);
		CLEAR_BIT(PORTD, RIGHTSIGNAL_OUT);
		TOGGLE_BIT(PORTD, LEFTSIGNAL_OUT);
	}
	else if(signal_state == Right)
	{
		//PORTB |= (1 << PORTB3);
		CLEAR_BIT(PORTD, LEFTSIGNAL_OUT);
		TOGGLE_BIT(PORTD, RIGHTSIGNAL_OUT);
	}
	else
	{
		//PORTB |= (1 << PORTB3);
		//PORTD &= ~(1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT); // Turn off the signals
		//TIMSK0 &= ~(1 << TOIE0); // Disable timer interrupt so the ISR that toggles the turn signals doesn't get called
		//TCNT0 = 0; // Clear timer counter
		PORTD &= ~(1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT); // Turn off the signals
		TIMSK0 &= ~(1 << TOIE0); // Disable timer interrupt so the ISR that toggles the turn signals doesn't get called
		TCNT0 = 0; // Clear timer counter
	}

}

ISR(TIMER1_CAPT_vect)
ISR(PCINT0_vect)
{
	if(BIT_IS_SET(PORTD, WAITINGSIGNAL_IN))
	uint8_t changedbits;

	changedbits = PINB ^ portbhistory_signals;
	portbhistory_signals = PINB;
	if(changedbits & (1 << WAITINGSIGNAL_IN))
	{
		if((BIT_IS_SET(PORTD, LEFTSIGNAL_OUT) && BIT_IS_SET(PORTD, RIGHTSIGNAL_OUT)) ||
		(BIT_IS_CLEAR(PORTD, LEFTSIGNAL_OUT) && BIT_IS_CLEAR(PORTD, RIGHTSIGNAL_OUT)) )
		{
			PORTD &= ~(1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT); //Make sure both are the same state to avoid desync between the right & left signals.
		}
		_delay_ms(25); // Debounce time
		if(BIT_IS_SET(PORTD, WAITINGSIGNAL_IN)) { signal_state = Waiting; }
		if(BIT_IS_CLEAR(PINB, WAITINGSIGNAL_IN))
			signal_state = Waiting;
		else
			signal_state = Forward;
	}
	else if(BIT_IS_SET(PORTD, LEFTSIGNAL_IN))
	else if(changedbits & (1 << LEFTSIGNAL_IN))
	{
		_delay_ms(25); // Debounce time
		if(BIT_IS_SET(PORTD, LEFTSIGNAL_IN)) { signal_state = Left; }
		if(BIT_IS_CLEAR(PINB, LEFTSIGNAL_IN))
			signal_state = Left;
		else
			signal_state = Forward;
	}
	else if(BIT_IS_SET(PORTD, RIGHTSIGNAL_IN))
	else if(changedbits & (1 << RIGHTSIGNAL_IN))
	{
		_delay_ms(25); // Debounce time
		if(BIT_IS_SET(PORTD, RIGHTSIGNAL_IN)) { signal_state = Right; }
		if(BIT_IS_CLEAR(PINB, RIGHTSIGNAL_IN))
			signal_state = Right;
		else
			signal_state = Forward;
	}
	else
	{
		signal_state = Forward;
	}
	// Enable the timer0 OVF interrupt which would control the signals.
	TIMSK0 |= (1 << TOIE0);
	// Call the timer0 which controls the turning signals for the first time so the driver would see immediate change (not neccessary)
	//TIMER0_OVF_vect();
}

ISR (INT0_vect)
{
	if(brakes_state == 0)
	{
		PORTD |= (1 << BRAKES_OUT);
		brakes_state = 1;
	}
	else
	{
		PORTD &= ~(1 << BRAKES_OUT);
		brakes_state = 0;
	}
}

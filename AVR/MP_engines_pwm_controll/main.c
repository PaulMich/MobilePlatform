/*
 * MP_engines_pwm_controll.c
 *
 * Created: 10/20/2019 12:15:40 PM
 * Author : Pawel
 */ 

/********************************************************************************************/
/* TABELA PRAWDY DLA STEROWNIKA TB9051FTG													*/
/*																							*/
/*  WEJSCIE						|WYJSCIE				|									*/
/*  EN	|ENB	|PWM1(A)|PWM2(B)|OUT1		|OUT2		|TRYB PRACY							*/
/*	PWM	|0		|1		|0		|PWM(H/Z)	|PWM(L/Z)	|Obroty do przodu/swobodny bieg		*/
/*	PWM	|0		|0		|1		|PWM(L/Z)	|PWM(H/Z)	|Obroty do tylu/swobodny bieg		*/
/*	0	|X		|X		|X		|Z			|Z			|Swobodny bieg, wyjscia odlaczone	*/
/********************************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "I2CSlave.h"

#define I2C_ADDR 0x08

#define CTRL_LED	PB0
#define PORT_CTRL_LED PORTB	

//#define ENB_FR	PC0
#define EN_FR	PD6	//!< (PWM -> OCR0A)
#define A_FR	PC3
#define B_FR	PC2

#define PORT_EN_FR PORTD
#define PORT_AB_FR PORTC

//#define ENB_FL	PC0
#define EN_FL	PD5	//!< (PWM -> OCR0B)
#define A_FL	PC0
#define B_FL	PC1

#define PORT_EN_FL PORTD
#define PORT_AB_FL PORTC

//#define ENB_RR	PC0
#define EN_RR	PB1	//!< (PWM -> OCR1A)
#define A_RR	PD4
#define B_RR	PD7

#define PORT_EN_RR PORTB
#define PORT_AB_RR PORTD

//#define ENB_RL	PC0
#define EN_RL	PB2	//!< (PWM -> OCR1B)
#define A_RL	PD0
#define B_RL	PD1

#define PORT_EN_RL PORTB
#define PORT_AB_RL PORTD

#define NONE 0
#define FRONT_LEFT 1
#define FRONT_RIGHT 2
#define REAR_LEFT 3
#define REAR_RIGHT 4

#define STOP 0
#define RIGHT 1
#define LEFT 2

#define I2C_CMD_START 0xFF

#define CMD_FL 118
#define CMD_FR 119
#define CMD_RL 120
#define CMD_RR 121

#define CMD_STOP 115
#define CMD_RIGHT 114
#define CMD_LEFT 108

double dutyCycle_FL = 0;
double dutyCycle_FR = 0;
double dutyCycle_RL = 0;
double dutyCycle_RR = 0;

double FL_DutyCorrection = 1;


uint8_t digitalWrite(volatile uint8_t *port, uint8_t pin, uint8_t state)
{
	if(state == 1) (*port) |= _BV(pin);
	else (*port) &= ~_BV(pin);
	
	return state;
}

void driveForward(uint8_t duty)
{
	digitalWrite(&PORT_AB_FL, A_FL, 1);
	digitalWrite(&PORT_AB_FL, B_FL, 0);
	
	digitalWrite(&PORT_AB_FR, A_FR, 0);
	digitalWrite(&PORT_AB_FR, B_FR, 1);
	
	digitalWrite(&PORT_AB_RL, A_RL, 1);
	digitalWrite(&PORT_AB_RL, B_RL, 0);
	
	digitalWrite(&PORT_AB_RR, A_RR, 0);
	digitalWrite(&PORT_AB_RR, B_RR, 1);
	
	dutyCycle_FL = duty*FL_DutyCorrection;
	dutyCycle_FR = duty;
	dutyCycle_RL = duty;
	dutyCycle_RR = duty;
}

void driveBackward(uint8_t duty)
{
	digitalWrite(&PORT_AB_FL, A_FL, 0);
	digitalWrite(&PORT_AB_FL, B_FL, 1);
	
	digitalWrite(&PORT_AB_FR, A_FR, 1);
	digitalWrite(&PORT_AB_FR, B_FR, 0);
	
	digitalWrite(&PORT_AB_RL, A_RL, 0);
	digitalWrite(&PORT_AB_RL, B_RL, 1);
	
	digitalWrite(&PORT_AB_RR, A_RR, 1);
	digitalWrite(&PORT_AB_RR, B_RR, 0);
	
	dutyCycle_FL = duty*FL_DutyCorrection;
	dutyCycle_FR = duty;
	dutyCycle_RL = duty;
	dutyCycle_RR = duty;
}

void driveSidewardRight(uint8_t duty)
{
	digitalWrite(&PORT_AB_FL, A_FL, 1);
	digitalWrite(&PORT_AB_FL, B_FL, 0);
	
	digitalWrite(&PORT_AB_FR, A_FR, 1);
	digitalWrite(&PORT_AB_FR, B_FR, 0);
	
	digitalWrite(&PORT_AB_RL, A_RL, 0);
	digitalWrite(&PORT_AB_RL, B_RL, 1);
	
	digitalWrite(&PORT_AB_RR, A_RR, 0);
	digitalWrite(&PORT_AB_RR, B_RR, 1);
	
	dutyCycle_FL = duty*FL_DutyCorrection;
	dutyCycle_FR = duty;
	dutyCycle_RL = duty;
	dutyCycle_RR = duty;
}

void driveSidewardLeft(uint8_t duty)
{
	digitalWrite(&PORT_AB_FL, A_FL, 0);
	digitalWrite(&PORT_AB_FL, B_FL, 1);
	
	digitalWrite(&PORT_AB_FR, A_FR, 0);
	digitalWrite(&PORT_AB_FR, B_FR, 1);
	
	digitalWrite(&PORT_AB_RL, A_RL, 1);
	digitalWrite(&PORT_AB_RL, B_RL, 0);
	
	digitalWrite(&PORT_AB_RR, A_RR, 1);
	digitalWrite(&PORT_AB_RR, B_RR, 0);
	
	dutyCycle_FL = duty*FL_DutyCorrection;
	dutyCycle_FR = duty;
	dutyCycle_RL = duty;
	dutyCycle_RR = duty;
}

void driveForwardRight_45(uint8_t duty)
{
	digitalWrite(&PORT_AB_FL, A_FL, 1);
	digitalWrite(&PORT_AB_FL, B_FL, 0);
	
	digitalWrite(&PORT_AB_FR, A_FR, 0);
	digitalWrite(&PORT_AB_FR, B_FR, 0);
	
	digitalWrite(&PORT_AB_RL, A_RL, 0);
	digitalWrite(&PORT_AB_RL, B_RL, 0);
	
	digitalWrite(&PORT_AB_RR, A_RR, 0);
	digitalWrite(&PORT_AB_RR, B_RR, 1);
	
	dutyCycle_FL = duty*FL_DutyCorrection;
	dutyCycle_FR = 100;
	dutyCycle_RL = 100;
	dutyCycle_RR = duty;
}

void driveBackwardLeft_45(uint8_t duty)
{
	digitalWrite(&PORT_AB_FL, A_FL, 0);
	digitalWrite(&PORT_AB_FL, B_FL, 1);
	
	digitalWrite(&PORT_AB_FR, A_FR, 0);
	digitalWrite(&PORT_AB_FR, B_FR, 0);
	
	digitalWrite(&PORT_AB_RL, A_RL, 0);
	digitalWrite(&PORT_AB_RL, B_RL, 0);
	
	digitalWrite(&PORT_AB_RR, A_RR, 1);
	digitalWrite(&PORT_AB_RR, B_RR, 0);
	
	dutyCycle_FL = duty*FL_DutyCorrection;
	dutyCycle_FR = 100;
	dutyCycle_RL = 100;
	dutyCycle_RR = duty;
}

void driveForwardLeft_45(uint8_t duty)
{
	digitalWrite(&PORT_AB_FL, A_FL, 0);
	digitalWrite(&PORT_AB_FL, B_FL, 0);
	
	digitalWrite(&PORT_AB_FR, A_FR, 0);
	digitalWrite(&PORT_AB_FR, B_FR, 1);
	
	digitalWrite(&PORT_AB_RL, A_RL, 1);
	digitalWrite(&PORT_AB_RL, B_RL, 0);
	
	digitalWrite(&PORT_AB_RR, A_RR, 0);
	digitalWrite(&PORT_AB_RR, B_RR, 0);
	
	dutyCycle_FL = 100;
	dutyCycle_FR = duty;
	dutyCycle_RL = duty;
	dutyCycle_RR = 100;
}

void driveBackwardRight_45(uint8_t duty)
{
	digitalWrite(&PORT_AB_FL, A_FL, 0);
	digitalWrite(&PORT_AB_FL, B_FL, 0);
	
	digitalWrite(&PORT_AB_FR, A_FR, 1);
	digitalWrite(&PORT_AB_FR, B_FR, 0);
	
	digitalWrite(&PORT_AB_RL, A_RL, 0);
	digitalWrite(&PORT_AB_RL, B_RL, 1);
	
	digitalWrite(&PORT_AB_RR, A_RR, 0);
	digitalWrite(&PORT_AB_RR, B_RR, 0);
	
	dutyCycle_FL = 100;
	dutyCycle_FR = duty;
	dutyCycle_RL = duty;
	dutyCycle_RR = 100;
}

void rotateRight(uint8_t duty)
{
	digitalWrite(&PORT_AB_FL, A_FL, 1);
	digitalWrite(&PORT_AB_FL, B_FL, 0);
	
	digitalWrite(&PORT_AB_FR, A_FR, 1);
	digitalWrite(&PORT_AB_FR, B_FR, 0);
	
	digitalWrite(&PORT_AB_RL, A_RL, 1);
	digitalWrite(&PORT_AB_RL, B_RL, 0);
	
	digitalWrite(&PORT_AB_RR, A_RR, 1);
	digitalWrite(&PORT_AB_RR, B_RR, 0);
	
	dutyCycle_FL = duty*FL_DutyCorrection;
	dutyCycle_FR = duty;
	dutyCycle_RL = duty;
	dutyCycle_RR = duty;
}

void rotateLeft(uint8_t duty)
{
	digitalWrite(&PORT_AB_FL, A_FL, 0);
	digitalWrite(&PORT_AB_FL, B_FL, 1);
	
	digitalWrite(&PORT_AB_FR, A_FR, 0);
	digitalWrite(&PORT_AB_FR, B_FR, 1);
	
	digitalWrite(&PORT_AB_RL, A_RL, 0);
	digitalWrite(&PORT_AB_RL, B_RL, 1);
	
	digitalWrite(&PORT_AB_RR, A_RR, 0);
	digitalWrite(&PORT_AB_RR, B_RR, 1);
	
	dutyCycle_FL = duty*FL_DutyCorrection;
	dutyCycle_FR = duty;
	dutyCycle_RL = duty;
	dutyCycle_RR = duty;
}

void stop()
{
	digitalWrite(&PORT_AB_FL, A_FL, 0);
	digitalWrite(&PORT_AB_FL, B_FL, 0);
	
	digitalWrite(&PORT_AB_FR, A_FR, 0);
	digitalWrite(&PORT_AB_FR, B_FR, 0);
	
	digitalWrite(&PORT_AB_RL, A_RL, 0);
	digitalWrite(&PORT_AB_RL, B_RL, 0);
	
	digitalWrite(&PORT_AB_RR, A_RR, 0);
	digitalWrite(&PORT_AB_RR, B_RR, 0);
	
	dutyCycle_FL = 100;
	dutyCycle_FR = 100;
	dutyCycle_RL = 100;
	dutyCycle_RR = 100;
}


volatile uint8_t i2c_data = 0;
uint8_t received_velocity = 0;
uint8_t f_which_engine = NONE;
uint8_t f_engine_rotation = STOP;
uint8_t en_a = 0;
uint8_t en_b = 0;

uint8_t engines_cmd_counter = 0;	//licznik przes?anych danych
uint8_t f_new_cmd = 0;				
uint8_t direction_mask = 0x0F;		//maska bitowa zawieraj?ca informacje o kierunku obrotów silników

void I2C_received(uint8_t data)
{
	if(data == I2C_CMD_START)	//je?eli nowa komenda
	{
		engines_cmd_counter = 0;	
		f_new_cmd = 1;			//rozpocznij zapis danych
	}
	
	if(f_new_cmd)
	{
		if(engines_cmd_counter == 1) dutyCycle_FL = data;
		else if(engines_cmd_counter == 2) dutyCycle_FR = data;		
		else if(engines_cmd_counter == 3) dutyCycle_RL = data;		
		else if(engines_cmd_counter == 4) dutyCycle_RR = data;	
		else if(engines_cmd_counter == 5) {direction_mask = data; f_new_cmd = 0;}	//zako?cz zapis danych
	}
	engines_cmd_counter++;
}
void I2C_requested()
{
	//I2C_transmitByte(i2c_data);
}

int main(void)
{	
	//! Inicjalizacja I2C w trybie slave z adresem 0x08
	I2C_setCallbacks(I2C_received, I2C_requested);
	I2C_init(I2C_ADDR);
	
	//! Zapalenie diody kontrolnej
	DDRB |= _BV(CTRL_LED);
	digitalWrite(&PORT_CTRL_LED, CTRL_LED, 1);
	
    //! Konfiguracja portow sterownika 1. silnika (FR)
	DDRD |= _BV(EN_FR); 
	DDRC |= _BV(A_FR);
	DDRC |= _BV(B_FR); 
	
	//! Konfiguracja portow sterownika 2. silnika (FL)
	DDRD |= _BV(EN_FL);
	DDRC |= _BV(A_FL);
	DDRC |= _BV(B_FL);
	
	//! Konfiguracja portow sterownika 3. silnika (RR)
	DDRB |= _BV(EN_RR);
	DDRD |= _BV(A_RR);
	DDRD |= _BV(B_RR);
	
	//! Konfiguracja portow sterownika 4. silnika (RL)
	DDRB |= _BV(EN_RL);
	DDRD |= _BV(A_RL);
	DDRD |= _BV(B_RL);	
	
	//! Konfiguracja timera dla PWM 1. i 2. silnika (FR, FL) - non-inverting Fast PWM
	TCCR0A = (_BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00));
	//! Konfiguracja przerwan timera dla PWM 1. i 2. silnika (FR, FL) - timer overflow
	TIMSK0 = _BV(TOIE0);
	
	OCR0A = (dutyCycle_FR/100)*255;
	OCR0B = (dutyCycle_FL/100)*255;
	
	//! Konfiguracja timera dla PWM 3. silnika (RR) - 8-bit non-inverting Fast PWM
	TCCR1A = (_BV(COM1A1) | _BV(COM1B1) | _BV(WGM10));
	//! Konfiguracja przerwan timera dla PWM 3. i 4. silnika (RR, RL) - timer overflow
	TIMSK1 = _BV(TOIE1);

	OCR1A = (dutyCycle_RR/100)*255;
	OCR1B = (dutyCycle_RL/100)*255;
	
	GTCCR = _BV(PSRSYNC);
	
	//! Konfiguracja prescalera dla PWM 1. i 2. silnika (FR, FL) - clk_IO/1
	TCCR0B = _BV(CS10);

	//! Konfiguracja prescalera dla PWM 3. i 4. silnika (RR) - clk_IO/1
	TCCR1B = (_BV(CS10) | _BV(WGM12));
	
	sei();
	
	///////////////////////////
	
	dutyCycle_FR = 0;
	dutyCycle_FL = 0;
	dutyCycle_RL = 0;
	dutyCycle_RR = 0;
    while (1) 
    {
		if(dutyCycle_FL == 0)
		{
			digitalWrite(&PORT_AB_FL, A_FL, 0);
			digitalWrite(&PORT_AB_FL, B_FL, 0);
		}
		else
		{
			if(direction_mask & 0x01)
			{
				digitalWrite(&PORT_AB_FL, A_FL, 0);
				digitalWrite(&PORT_AB_FL, B_FL, 1);
			}
			else
			{
				digitalWrite(&PORT_AB_FL, A_FL, 1);
				digitalWrite(&PORT_AB_FL, B_FL, 0);
			}
			
		}
		
		if(dutyCycle_FR == 0)
		{
			digitalWrite(&PORT_AB_FR, A_FR, 0);
			digitalWrite(&PORT_AB_FR, B_FR, 0);
		}
		else
		{
			if(direction_mask & 0x02)
			{
				digitalWrite(&PORT_AB_FR, A_FR, 1);
				digitalWrite(&PORT_AB_FR, B_FR, 0);
			}
			else
			{
				digitalWrite(&PORT_AB_FR, A_FR, 0);
				digitalWrite(&PORT_AB_FR, B_FR, 1);
			}
			
		}
		
		if(dutyCycle_RL == 0)
		{
			digitalWrite(&PORT_AB_RL, A_RL, 0);
			digitalWrite(&PORT_AB_RL, B_RL, 0);
		}
		else
		{
			if(direction_mask & 0x04)
			{
				digitalWrite(&PORT_AB_RL, A_RL, 0);
				digitalWrite(&PORT_AB_RL, B_RL, 1);
			}
			else
			{
				digitalWrite(&PORT_AB_RL, A_RL, 1);
				digitalWrite(&PORT_AB_RL, B_RL, 0);
			}
			
		}
		
		if(dutyCycle_RR == 0)
		{
			digitalWrite(&PORT_AB_RR, A_RR, 0);
			digitalWrite(&PORT_AB_RR, B_RR, 0);
		}
		else
		{
			if(direction_mask & 0x08)
			{
				digitalWrite(&PORT_AB_RR, A_RR, 1);
				digitalWrite(&PORT_AB_RR, B_RR, 0);
			}
			else
			{
				digitalWrite(&PORT_AB_RR, A_RR, 0);
				digitalWrite(&PORT_AB_RR, B_RR, 1);
			}
		}
    }
}

//! 
ISR(TIMER0_OVF_vect)
{
	OCR0A = (dutyCycle_FR/100)*255;
	OCR0B = (dutyCycle_FL/100)*255;
}

ISR(TIMER1_OVF_vect)
{
	OCR1A = (dutyCycle_RR/100)*255;
	OCR1B = (dutyCycle_RL/100)*255;
}

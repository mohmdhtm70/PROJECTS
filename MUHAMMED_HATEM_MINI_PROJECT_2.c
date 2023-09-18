/*
 * MUHAMMED_HATEM_MINI_PROJECT_2.c
 *
 *  Created on: Sep 17, 2023
 *      Author: ELBOSTAN
 */
#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>

unsigned char sec=0;
unsigned char min=0;
unsigned char hrs=0;

unsigned char flag=0;

ISR(TIMER1_COMPA_vect)
{
flag=1;
}

void TIMER1_CTC_COUNT_TIME(void)
{SREG  |= (1<<7);

	TCNT1=0;
	OCR1A=500;
	TIMSK|=(1<<OCIE1A);
	/*STEPS FOR CONFIGURATION TIMER1

	-->HERE WE USE CTC MODE SO THERE ARE NO NEED FOR PWM
	SO [FOC1A=1,FOC1B=1]

	-->THERE ARE NO NEED FOR OC1A & OC1B
	SO [COM1A0/COM1B0=0,COM1A1,COM1B1=0]

	 -->HERE WE USE CTC MODE AND WE PUT THE COMPARE MATCH VALUE IN OCR1A
	 SO FROM DATASHEET WE FIND THAT IS THE 4TH MODE  [WGM10=0 , WGM11=0 , WGM12=1 , WGM13=0]
     AND YOU SHOULD PUT 1 AT WGM12 AT TCCR1B
	 */
	TCCR1A=(1<<FOC1A)|(1<<FOC1B);

	//CLOCK SELECT BIT --> Clk/1024 SO [CS10=1,CS11=0,CS12=1]
	TCCR1B =(1<<WGM12) | (1<<CS12) | (1<<CS10);

}

ISR(INT0_vect)
{
	//reset the stop watch

	sec=0;
	min=0;
	hrs=0;

}
ISR(INT1_vect) {
	/*
	 TO PAUSE THE TIMER(STOP COUNTING) WE SHOULD PUT[CS12=0,CS11=0,CS10=0]
	*/
	TCCR1B &= 0xF8;
}
ISR(INT2_vect) {
	 //TO RESUME THE COUNTING WE SHOULD PUT[CS12=1,CS11=0,CS10=1] TO USE PRESCALER 1024 AS IT WAS
	TCCR1B |= (1 << CS10) | (1 << CS12);

}


void INT0_FALLINGEDGE(void)
 {
	/*INT0 configuration
	 -->for INT0 FALLING EDGE [ISC01=1 , ISC00=0]

	 -->WE SHOULD SET EXTERNAL INTERRUPT REQUIST 0 ENABLE

    -->DEFINE PD2 AS INPUT FOR INT0
	 */
MCUCR &= ~(1<<ISC00);
MCUCR |=(1<<ISC01);
GICR |=(1<<INT0);
DDRD&=~(1<<PD2);
PORTD|=(1<<PD2);
 }
void INT1_RISINGEDGE(void)
{/*INT1 configuration
  -->for INT1 RISING EDGE [ISC10=1 , ISC11=1]

  -->WE SHOULD SET EXTERNAL INTERRUPT REQUIST 1 ENABLE

  -->DEFINE PD3 AS INPUT FOR INT1

*/
MCUCR |= (1 << ISC10) | (1 << ISC11);
GICR |= (1 << INT1);
DDRD &= ~(1 << PD3);

}
void INT2_FALLINGEDGE(void)
{/*  INT2 configuration

	 -->for INT2 FALLING EDGE WE SHOULD PUT ISC2=0 IN MCU Control and
         Status Register (MCUCSR)

	 -->WE SHOULD SET EXTERNAL INTERRUPT REQUIST 2 ENABLE

    -->DEFINE PB2 AS INPUT FOR INT0
	 */
	MCUCSR &=~(1<<ISC2);
	GICR |= (1 << INT2);
	DDRB &= ~(1 << PB2);
	PORTB |= (1 << PB2);



}

int main(void)
{
DDRC|=0x0F;
PORTC &=0xF0;
//FIRST 6PINS IN PORTA (ENABLE/DISABLE)
DDRA  |= 0x3F;
PORTA |=0x3F;
SREG  |= (1<<7);

TIMER1_CTC_COUNT_TIME();
INT0_FALLINGEDGE();
INT1_RISINGEDGE();
INT2_FALLINGEDGE();

while(1)
{
	if(1==flag)
	{ sec++;
	if(60==sec)
	  {
		min++;
		sec=0;
	  }
	if(60==min)
	 {
		hrs++;
		min=0;
	 }
	if(24==hrs)
	{

	hrs=0;
	}
	flag=0;

	}
	else{


	//1ST 7_SEGMENT FOR 1ST DIGIT IN  SECONDS
	PORTA=(PORTA&0xC0) | 0X01;
  //VALUE OF 1ST DIGIT (IN SEC1) IN 1ST 4PINS IN PORTC
	PORTC=(PORTC &0xF0) |((sec%10));
	_delay_ms(3);

	//2ND 7_SEGMENT FOR 2ND DIGIT IN SECONDS
	PORTA=(PORTA & 0xC0)|0X02;
    //VALUE OF 2ND DIGIT (IN SEC2) IN 1ST 4PINS IN PORTC
	PORTC=(PORTC &0xF0) |((sec/10));
	_delay_ms(1);

	//3RD 7_SEGMENT FOR 3RD DIGIT IN  MINUTES
	PORTA = (PORTA & 0xC0)|0x04;
    //VALUE OF 3RD DIGIT (IN min1) IN 1ST 4PINS IN PORTC
	PORTC=(PORTC &0xF0) |((min%10));
	_delay_ms(1);

	//4th 7_SEGMENT FOR 2ND DIGIT IN minutes
	PORTA = (PORTA & 0xC0)|0x08;
    //VALUE OF 4TH DIGIT (IN min2) IN 1ST 4PINS IN PORTC
	PORTC = (PORTC & 0xF0) | ((min / 10) );
	_delay_ms(1);

	//5th 7_SEGMENT FOR 1ST DIGIT IN HOURES
	PORTA = (PORTA & 0xC0)|0x10;
	 //VALUE OF 5TH DIGIT (IN hrs1) IN 1ST 4PINS IN PORTC
	PORTC = (PORTC & 0xF0) | ((hrs % 10) );
	_delay_ms(1);

	//6th 7_SEGMENT FOR 2ND DIGIT IN houres
    PORTA = (PORTA & 0xC0)|0x20;
	 //VALUE OF 6TH DIGIT (IN hrs2) IN 1ST 4PINS IN PORTC
	PORTC = (PORTC & 0xF0) | ((hrs / 10) );
	_delay_ms(1);
	}














}






}

/*
 * Author : Alin
 * Edited by: Botond Bencsik
 * Updated via GitHub
 * Polished with GitHub Copilot
 */ 

 
#define F_CPU 16000000UL

#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "usart.h"


volatile unsigned char dutyCycle = 0; //value = 0-255; duty cycle formula: dutyCycle/255=percentage, has to be declared outside main for the ISR to access it
volatile float dutyHelper = 0;

unsigned char motor_run = 0;

volatile char rx_end = 0;
volatile char rx_in;
volatile unsigned char rx_strike = 0;
volatile unsigned char rx_error_strike = 0;
volatile unsigned char rx_val = 0;
volatile unsigned char rx_page = 0;
volatile unsigned char rx_error = 0;

volatile unsigned int time_temp = 0;
volatile double time = 0.0;
volatile unsigned char overflow_flag = 0;
volatile double timer = 0.0;

ISR(USART_RX_vect)
{
    rx_in = UDR0;
    if(rx_in == 0x71 && rx_val == 0 && rx_error == 0) //First bit; incoming datatype flag
        rx_val = 1;
    else if(rx_in == 0x1A && rx_val == 0 && rx_error == 0)
        rx_error = 1;
    else if(rx_in == 0xFF && rx_error == 1)
    {
        rx_error_strike++;
        if(rx_error_strike == 3)
        {
            rx_error = 0;
            rx_error_strike = 0;
        }
    }
    else if(rx_in == 0xFF && rx_val == 1 && rx_error == 0)
    {
        rx_strike++;
        if(rx_strike == 3)
        {
            rx_val = 0;
            rx_strike = 0;
        }
    }
    else if(rx_val == 1 && rx_error == 0 && rx_in <= 100)
    {
        dutyCycle = 255 * (rx_in/100.0);
        OCR0A = dutyCycle;
        OCR0B = OCR0A;
    }
    if(rx_val == 1 && rx_error == 0)
    {
        switch (rx_in)
        {
            case 101: //increase duty cycle by 10%
                if((dutyCycle + 25) <= 255)
                {
                    dutyHelper = dutyHelper + 25.5;
                    dutyCycle = dutyHelper;
                    OCR0A = dutyCycle;
                    OCR0B = OCR0A;
                }
                break;
            case 102: //decrease duty cycle by 10%
                if((dutyCycle - 25) >= 0)
                {
                    dutyHelper = dutyHelper - 25.5;
                    dutyCycle = dutyHelper;
                    OCR0A = dutyCycle;
                    OCR0B = OCR0A;
                }
                break;
            case 103: //start motor
                motor_run = 1;
                rx_end = 1;
                break;
            case 104: //stop motor 
                motor_run = 0;
                rx_end = 1;
                break;
            case 105: //change page
                rx_page = 1;
                break;
            case 106: //change page
                rx_page = 0;
                break;
        }
    }
}


ISR(TIMER1_OVF_vect) //Timer/Counter1 Overflow Interrupt to keep track of time even when the TCNT1 register reset without a detection
{

    if(overflow_flag == 1)
    {
        time_temp += 65535;
    }
    overflow_flag = 1;

}


void screen_reset(void);
unsigned int read_adc(void);


int main(void) {    

    uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the communication

    //Reset to make everything nice and zero if the arduino is rebooted
    screen_reset();

    //Constants
    double DETAIL_OF_ENCODER = 10; //how many cutouts there are on the encoder wheel
    double WHEEL_DIAM = 66.0;//in mm
    double SLIP = 1; //can modify the value of the dist_const to better reflect reality
    double DIST_CONST = ((((WHEEL_DIAM/10)*3.141)*((360/DETAIL_OF_ENCODER)/360)))*SLIP; //in cm

    //Battery voltage
    double volts;

    //Speed and distance
    double dist = 0, speed = 0;

    //Setup timer/counter 1 as pure ticks counter, with 1024 prescaling ((16*10^6)/prescaler)=number of CPU cycles per clock tick
    TCCR1A = 0x00; //0b00000000
    TCCR1B = 0xC5; //Input capture on positive edge ICP1 pin (PB0). Filtered. 1024 prescaler
    DDRB &= ~0x01;//PINB0 as input for ICP1 use​
    PORTB |= 0x01;//Enable pullup

    //PWM, motor driver, runs in the backround
    DDRD |= (1 << PORTD5) | (1 << PORTD6); //PD6-5 to output // 0x60
    TCCR0A |= 0b00000011; //original value 0b10100011
    //TCCR0A &= ~((1 << COM0A1) | (1 << COM0B1));
    TCCR0B |= (0 << WGM02 ) |(1 << CS02) | (0 << CS01) | (1 << CS00);
    OCR0A |= dutyCycle; //When the compare match occurs OC0A is cleared (output pins cleared)
    OCR0B = OCR0A;

    //ADC
    ADMUX = 0b01000000; //ADC0 selected, PC0, A0 on arduino
    ADCSRA = 0b11100111; //Bit 4 is set when conversion is complete; 0xE7
    ADCSRB = 0x00;
    //Need to read ADCL first otherwise it won't update, then add ADCH BUT LEFTSHIFTED 8 places

    //Interrupts
    SREG |= 0x80; //Enabling global interrupts
    UCSR0A |= (1 << RXC0);
    UCSR0B |= (1 << RXCIE0);
    //UDR0; //recieved data will be in this register
    PORTD |= 0x03;
    
    //printf("%c%c%c", 255, 255, 255);

    while(1)
    {

        while(!(TIFR1 & (1<<ICF1)))
        {

            if(rx_page == 0)
            {

                volts = 5.00*(read_adc()/1023.00)*100;
                printf("voltval.val=%d%c%c%c", (int)volts, 255 , 255, 255); //Voltage converted and displayed for debugging/testing/demonstration
                
                if(rx_end == 1)
                {
                    if(motor_run == 0)
                    {   
                        TCCR0A &= ~((1 << COM0A1) | (1 << COM0B1));
                        //printf("TCCR0A=%d%c%c%c", TCCR0A, 255, 255, 255); //Debug command
                    }
                    else if(motor_run == 1)
                    {
                        TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
                        //printf("TCCR0A=%d%c%c%c", TCCR0A, 255, 255, 255); //Debug command
                    }
                    rx_end = 0;
                }
            }

        }
        while(TIFR1 & (1<<ICF1))
        {

            time = ((ICR1 + time_temp) / (16000000.0/1024.0));
            TCNT1 = 0; //Clock reset
            TIFR1 = TIFR1 | 0b00100000; //TIFR1 = (1<<ICF1); these are doing the same thing: reset the input capture flag
            time_temp = 0;
            overflow_flag = 0;
            
            speed = DIST_CONST / time;
            dist = dist + DIST_CONST;
            timer += time;

            if(rx_page == 0)
            {
                //Display instructions, occasional multiplication to adjust the values for the screen to be displayed properly
                printf("speedval.val=%d%c%c%c", (int)speed, 255, 255, 255);
                printf("distval.val=%d%c%c%c", (int)dist, 255 , 255, 255);
                printf("timeval.val=%d%c%c%c", (int)(timer * 10), 255, 255, 255);
            }

        }

    }

    return 0;

}

//Analog to digital voltage converter, ADCL, ADCH contain the conversion results
unsigned int read_adc(void) {
    unsigned int adc_low = ADCL; //ADCL should be read first
    return adc_low + ((ADCH & 0x03) << 8); //<< 8 is there so ADCH will act like the 9th and 10th bit when added to ADCL
}   //0b11111111 + 0b00000011 is not what we want, we want 0b1100000000 + 0b11111111 = 0b1111111111 (arbitrary values)

void screen_reset(void)
{
    printf("page 0%c%c%c", 255, 255, 255);
    printf("speedval.val=0%c%c%c", 255, 255, 255);
    printf("distval.val=0%c%c%c", 255, 255, 255);
    printf("voltval.val=0%c%c%c", 255, 255, 255);
    printf("timeval.val=0%c%c%c", 255, 255, 255);
    printf("page 1%c%c%c", 255, 255, 255);
    printf("n0.val=0%c%c%c", 255, 255, 255);
    printf("h0.val=0%c%c%c", 255, 255, 255);
    printf("page 0%c%c%c", 255, 255, 255);
}
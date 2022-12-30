/*
 * Made by: Botond Bencsik
 * Updated via GitHub
 */ 

 
#define F_CPU 16000000UL

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "usart.h"


typedef struct {
    float dist;
    float time;
} section_t;

volatile section_t create_section(float, float);

//Global variables

volatile section_t rx_sections[5]; //array of sections for now max is 10
volatile unsigned char rx_section_index = 0; //index of current section
volatile unsigned char rx_section_count = 1; //number of sections
volatile unsigned char rx_section_current = 0; //current section

//Speed and distance
double c_dist = 0, p_dist = 0, dist_left = 0, c_speed = 0, prev_speed = 0, v_change = 0, d_dist = 0;
double speed_change_tol = 0.07; //speed change tolerance
bool first_run = 0;
double dist_change_tol = 2.0; //distance change tolerance, in cm

volatile float dutyCycle = 150; //value = 0-255; duty cycle formula: dutyCycle/255=percentage, has to be declared outside main for the ISR to access it
volatile bool motor_run = 0;

//Battery voltage
double volts;
float VREF = 5.05; //measured with multimetre, PC: 4.79, LiPO: 5.03

volatile unsigned char rx_in;
volatile bool rx_val = 0;
volatile unsigned char rx_page = 0;
volatile bool rx_end = 0;

//Timer variables
double time = 0.0;
volatile unsigned char overflow_count = 0;

volatile float timer0 = 0.0;
volatile float timer1 = 0.0;
volatile bool timer_running = 0;

double time_left = 0.0;

//Constants
const double DETAIL_OF_ENCODER = 20; //how many cutouts there are on the encoder wheel
const double WHEEL_DIAM = 66.0;//in mm
const double SLIP = 1.02; //can modify the value of the dist_const to potentially better reflect reality
const double PRESCALED_CPU = 15625.0; //prescaled CPU frequency
const double OVF16_SEC = 4.19424; //seconds per overflow of 16 bit timer


ISR(USART_RX_vect)
{
    rx_in = UDR0;
    if(rx_in == 0xFF) //End of transmission flag 
        return;
    if(rx_in == 0x71 && rx_val == 0) //First bit; incoming datatype flag
    {
        rx_val = 1;
        return;
    }
    if(rx_val == 1)
    {
        switch(rx_page)
        {
            case 1:
                switch(rx_in)
                {
                    case 103: //start motor
                        motor_run = 1;
                        rx_end = 1;
                        break;
                    case 104: //stop motor 
                        motor_run = 0;
                        rx_end = 1;
                        break;
                    case 105: //change page 
                        rx_page = 0;
                        break;
                }
                break;
            case 0:
                switch(rx_in)
                {
                    case 108: //change page
                        rx_page = 1;
                        break;
                    case 109: //add section
                        rx_section_index++;
                        rx_section_count++;
                        break;
                    case 110: //distance increase
                        rx_sections[rx_section_index].dist += 10.0;
                        break;
                    case 111: //distance decrease
                        rx_sections[rx_section_index].dist -= 10.0;
                        break;
                    case 112: //time increase
                        rx_sections[rx_section_index].time += 2.5;
                        break;
                    case 113: //time decrease
                        rx_sections[rx_section_index].time -= 2.5;
                        break;
                    case 114: //delete section
                        rx_sections[rx_section_index].dist = 0;
                        rx_sections[rx_section_index].time = 0;
                        rx_section_index--;
                        rx_section_count--;
                        break;
                    case 115: //change section+
                        rx_section_index++;
                        break;
                    case 116: //change section-
                        rx_section_index--;
                        break;
                }
                break;
        }
        rx_val = 0;
    }
}


ISR(TIMER1_OVF_vect) //Timer/Counter1 Overflow Interrupt to keep track of time even when the TCNT1 register reset without a detection
{
    if(!(timer_running == 1))
        return;
    overflow_count++;
}


void screen_reset(void);
unsigned int read_adc(void);
void motor_control(int);
void vchange_print(void);
void section_print(void);
void section_dash(void);
void compare(void);
void voltage_print(void);
void current_variables(double);
void distance_left(void);


int main(void) {    

    uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the communication

    //Reset to make everything nice and zero if the arduino is rebooted
    screen_reset();

    //Section array pre-initialization with 0 values
    for(int i = 0; i < 5; i++)
    {
        rx_sections[i].dist = 0.0;
        rx_sections[i].time = 0.0;
    }

    //Distance constant calculation, per detection
    double DIST_CONST = ((((WHEEL_DIAM/10)*3.141)*((360/DETAIL_OF_ENCODER)/360)))*SLIP; //in cm

    //Setup for Timer/Counter1
    TCCR1A = 0x00; //0b00000000
    TCCR1B = 0xC5; //Positive edge ICP1 pin (PB0). Noise filter enabled. 1024 prescaler -> 16MHz/1024 = 15625Hz is the frequency of the timer
    DDRB &= ~0x01;//PB0 as input for ICP1 use​
    PORTB |= 0x01;//Enable pullup resistor on PB0
    TIMSK1 |= 0x01; //Enable overflow interrupt

    //PWM, motor driver, runs in the backround
    DDRD |= (1 << PORTD5) | (1 << PORTD6); //PD6-5 to output // 0x60
    TCCR0A |= 0b00000011; //value given in the PPT 0b10100011; alternative way of setting the bits: TCCR0A &= ~(1 << COM0A1);
    TCCR0B |= (0 << WGM02 ) |(1 << CS02) | (0 << CS01) | (1 << CS00);
    //OCR0A |= dutyCycle; //When the compare match occurs OC0A is cleared (output pins cleared)

    //PWM, motor driver, runs in the backround
    DDRD |= (1 << PORTD5) | (1 << PORTD6); //PD6-5 to output
    TCCR0A |= 0b00000011; //Disconnecting OC0A and OC0B from the output pins, so motor does not start without a command
    TCCR0B |= (0 << WGM02 ) |(1 << CS02) | (0 << CS01) | (1 << CS00); //Prescaler 1024
    //OCR0A |= dutyCycle; //When the compare match occurs OC0A is cleared (output pins cleared)

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
    PORTD |= 0x03; //Enable pullup on PD0-1

    while(1)
    {

        _delay_ms(200); //Delay to make sure conversion result is ready

        while(!(TIFR1 & (1<<ICF1)))
        {
            //printf("dutycycle%d%c%c%c", OCR0A, 255, 255, 255); //Useful debug info
            //printf("motorrun%d%c%c%c", motor_run, 255, 255, 255); //Useful debug info
            //printf("overflow%d%c%c%c", overflow_count, 255, 255, 255); //Useful debug info
            //printf("page%d%c%c%c", rx_page, 255, 255, 255); //Useful debug info
            //printf("section%d%c%c%c", rx_section_index, 255, 255, 255); //Useful debug info
            //printf("dist%f%c%c%c", rx_sections[rx_section_index].dist, 255, 255, 255); //Useful debug info
            //printf("time%f%c%c%c", rx_sections[rx_section_index].time, 255, 255, 255); //Useful debug info
            //printf("mode%d%c%c%c", mode, 255, 255, 255); //Useful debug info

            if(rx_page == 0)
            {
                section_print();
            }
            else if(rx_page == 1)
            {
                voltage_print();

                if(rx_end == 1)
                {
                    motor_control(2);
                    rx_section_index = 0;
                    rx_end = 0;
                }
            }
        }

        while(TIFR1 & (1<<ICF1))
        {    

            time = (ICR1 / PRESCALED_CPU) + (overflow_count * OVF16_SEC);
            TCNT1 = 0; //Clock reset
            TIFR1 = TIFR1 | 0b00100000; //TIFR1 = (1<<ICF1); these are doing the same thing: reset the input capture flag
            overflow_count = 0;
            
            voltage_print();

            if(timer_running == 1)
            {
                current_variables(DIST_CONST);

                timer1 += time;
                time_left = rx_sections[rx_section_index].time - timer1;
                p_dist = c_dist + (c_speed * time_left);
                d_dist = p_dist - (rx_sections[rx_section_index].dist);
                compare();

                distance_left();
                section_dash();

                if(c_dist >= (rx_sections[rx_section_index].dist))
                {
                    if((rx_section_index + 1) <= rx_section_count)
                    {
                        rx_section_index++;
                        rx_section_current++;
                        timer1 = 0;
                        c_dist = 0;
                        timer1 = 0;
                    }
                    if(rx_section_index == rx_section_count)
                    {
                        motor_control(0);
                    }
                }

                if(first_run == 1) //This is to prevent the first run from displaying a value
                {
                    v_change = prev_speed - c_speed;

                    if(rx_page == 2 || rx_page == 1)
                    {
                        vchange_print();
                    }
                } 
                
                first_run = 1; 
                prev_speed = c_speed;
            }
        }
    }
    return 0;
}

inline unsigned int read_adc(void) //Analog to digital voltage converter, ADCL, ADCH contain the conversion results
{
    unsigned int adc_low = ADCL; //ADCL should be read first
    return adc_low + ((ADCH & 0x03) << 8); //<< 8 is there so ADCH will act like the 9th and 10th bit when added to ADCL
}   //0b11111111 + 0b00000011 is not what we want, we want 0b11-0000-0000 + 0b1111-1111 = 0b11-1111-1111 (arbitrary values)

inline void voltage_print(void)
{
    volts = VREF*2.0*(read_adc()/1024.00);
    printf("voltval.val=%d%c%c%c", (int)(volts * 100.0), 255 , 255, 255); //Voltage converted and displayed for debugging/testing/demonstration
    if(volts < 6.8)
    {
        printf("page 2%c%c%c", 255, 255, 255);
        while(1);
    }
}

inline void current_variables(double DIST_CONST)
{
    c_speed = DIST_CONST / time;
    c_dist = c_dist + DIST_CONST;
}

void screen_reset(void)
{
    printf("page 1%c%c%c", 255, 255, 255);
    printf("timeleft.val=0%c%c%c", 255, 255, 255);
    printf("timebar.val=0%c%c%c", 255, 255, 255);
    printf("distbar.val=0%c%c%c", 255, 255, 255);
    printf("cdist.val=0%c%c%c", 255, 255, 255);
    printf("distleft.val=0%c%c%c", 255, 255, 255);
    printf("vchange.txt=\" \"%c%c%c", 255, 255, 255);
    printf("page 0%c%c%c", 255, 255, 255);
    printf("sectionsnum.val=1%c%c%c", 255, 255, 255);
    printf("sectnr.val=0%c%c%c", 255, 255, 255);
    printf("sectdist.val=0%c%c%c", 255, 255, 255);
    printf("secttime.val=0%c%c%c", 255, 255, 255);
}

volatile section_t create_section(float distance, float time)
{
    section_t section;
    section.dist = distance;
    section.time = time;
    return section;
}

void motor_control(int run)
{
    if(motor_run == 0 || run == 0)
    {   
        TCCR0A &= ~(1 << COM0A1);
        timer_running = 0; 
    }
    else if(motor_run == 1 || run == 1)
    {
        TCCR0A |= (1 << COM0A1);
        timer_running = 1;
        OCR0A = dutyCycle;
        overflow_count = 0;
        TCNT1 = 0;
    }
}

inline void vchange_print(void)
{
    if(v_change > speed_change_tol)
        printf("vchange.txt=\"Accelerating\"%c%c%c",255,255,255);

    if(v_change < -(speed_change_tol))
        printf("vchange.txt=\"Deaccelerating\"%c%c%c",255,255,255);
                        
    if((v_change < speed_change_tol ) && (v_change > -(speed_change_tol) ))
        printf("vchange.txt=\" \"%c%c%c",255,255,255);
}

inline void section_print(void)
{
    printf("sectdist.val=%d%c%c%c", (int)(rx_sections[rx_section_index].dist / 10), 255, 255, 255);
    printf("secttime.val=%d%c%c%c", (int)(rx_sections[rx_section_index].time * 10), 255, 255, 255);
    printf("sectionsnum.val=%d%c%c%c", (int)rx_section_count, 255, 255, 255);
    printf("sectnr.val=%d%c%c%c", (int)(rx_section_index + 1), 255, 255, 255);
}

inline void section_dash(void)
{
    printf("timeleft.val=%d%c%c%c", (int)(time_left * 10), 255, 255, 255);
    printf("cdist.val=%d%c%c%c", (int)(c_dist), 255, 255, 255);
    printf("distleft.val=%d%c%c%c", (int)(dist_left), 255, 255, 255);
    printf("csect.val=%d%c%c%c", (int)(rx_section_current + 1), 255, 255, 255);
    printf("timebar.val=%d%c%c%c", (int)((timer1 / rx_sections[rx_section_index].time) * 100.0), 255, 255, 255);
    printf("distbar.val=%d%c%c%c", (int)((c_dist / rx_sections[rx_section_index].dist) * 100.0), 255, 255, 255);
}

inline void compare(void)
{
    if((d_dist >= dist_change_tol) && ((dutyCycle - 5.0) >= 0))
    {
        dutyCycle -= 5.0;
    }
    else if((d_dist < dist_change_tol) && ((dutyCycle + 5.0) <= 255))
    {
        dutyCycle += 5.0;
    }
    OCR0A = dutyCycle;
}

inline void distance_left(void)
{
    if((rx_sections[rx_section_index].dist - c_dist) >= 0)
        dist_left = (rx_sections[rx_section_index].dist) - c_dist;
    else
        dist_left = 0;
}

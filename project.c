/********************************************
 *
 *  Name: Padmanabha Kavasseri
 *  Email: kavasser@usc.edu
 *  Section: Friday 12:30
 *  Assignment: Project 
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h> 
#include <stdlib.h>
#include "lcd.h"
#include "adc.h"
#include "ds18b20.h"
#include <avr/eeprom.h>

#define FOSC 16000000           
#define BAUD 9600               
#define MYUBRR (FOSC/16/BAUD-1)

volatile char buff[6];
volatile int i = 0;
volatile int flag = 0;
volatile int stringstarting = 0;
volatile int stringready = 0;
volatile int numcharacters = 0;
volatile unsigned char changed = 0;  // Flag for state change
volatile int hightemp = 0;
volatile int lowtemp = 0;
volatile unsigned char new_state, old_state;
volatile int changingHigh = 0;


void serial_init(unsigned short ubrr_value)
{
    // Set up USART0 registers
    // Enable tri-state
    //Initialization of the serial port (USART0)
    UBRR0 = ubrr_value;                       
    UCSR0C = (3 << UCSZ00);               
                                          
    UCSR0B |= (1 << TXEN0 | 1 << RXEN0);  
    DDRD |= (1 << PD3);                  
    PORTD &= ~(1 << PD3);               
}

void serial_txchar(char ch)
{
    
    while ((UCSR0A & (1<<UDRE0)) == 0){};
    UDR0 = ch;
}

void serial_stringout(char *s)
{
    
    int i=0;
    for (i = 0; i < 6; i++)
    {
       serial_txchar(s[i]); 
    }
}
void discard () {
	int j;
	for (j = 0; j< 5; j++){
    		buff[j] = '\0';
    }
    stringstarting = 0;
    stringready = 0;
    numcharacters = 0;

}

ISR(USART_RX_vect)
{

    char a;
    a = UDR0;

    if (a == '@'){
    	stringstarting = 1;

    }
    else if (stringstarting == 1){
    	if (numcharacters == 0 && a != '+' && a != '-'){
    		discard();
    		return;
    	}
    	if ((numcharacters < 4) && (numcharacters > 0)  && ((a - '0') < 0 || (a - '0') > 9 )){
    		discard();
    		return;
    	}
    	if (a == '$'){
    		if (numcharacters == 0){
    			discard();
    			return;
    		} 
    		stringstarting = 0;
    		stringready =1;
    		return;
    	}	
       buff[i] = a;
       i++;
       numcharacters++;

    }	

}

ISR(PCINT1_vect)
{



	unsigned char a, b;
	// Read the A and B inputs to determine the intial state
    // Warning: Do NOT read A and B separately.  You should read BOTH inputs
    // at the same time, then determine the A and B values from that value.    
    unsigned char read = PINC;
    b = read & (1 << 4); //note to self: change read to PINC if necessary
    a = read & (1 << 1);

    if (!b && !a)
	new_state = 0;
    else if (!b && a)	
	new_state = 1;
    else if (b && !a)
	new_state = 2;
    else
	new_state = 3;

    // Loop forever
	// Read the input bits and determine A and B
    read = PINC;
    b = read & (1 << 4); 
    a = read & (1 << 1);

	// For each state, examine the two input bits to see if state
	// has changed, and if so set "new_state" to the new state,
	// and adjust the count value.

    int * varChanging;
    if(changingHigh){
    	varChanging = &hightemp;
    }
    else{
    	varChanging = &lowtemp;
    }

	if (old_state == 0) {
		if (a) {
			new_state = 1;
			(*varChanging)++;
		}
		else if (b) {
			new_state = 2;
			(*varChanging)--;
		}
		}
	    // Handle A and B inputs for state 0
	else if (old_state == 1) {
		if (!a) {
			new_state = 0;
			(*varChanging)--;
		}
		else if (b) {
			new_state = 3;
			(*varChanging)++;
		}
		}
	    // Handle A and B inputs for state 1

	else if (old_state == 2) {
		if (!b) {
			new_state = 0;
			(*varChanging)++;
		}
		else if (a) {
			new_state = 3;
			(*varChanging)--;
			
		}
		}
	    // Handle A and B inputs for state 2

	else {   // old_state = 3
		if (!a) { 
			(*varChanging)++;
			new_state = 2;
		}
		else if (!b) {
			new_state = 1;
			(*varChanging)--;
		}
	    // Handle A and B inputs for state 3
	}

	if (new_state != old_state) {
	    changed = 1;
	    old_state = new_state;
	    eeprom_update_byte ((void *)200, hightemp);
	    eeprom_update_byte ((void *)201, lowtemp);

	}
}



int main () {
    // Enable interrupts
    PORTC |= ((1 << 1) | (1 << 4)); //enable the pull-up resistors for Port C, bits 1 and 5
    DDRB |= (1 << 4); //set Port B, bit 4 as output // REMEMBER TO CHANGE THE PORTS
    
    PCICR |= (1 << PCIE1);
    PCMSK1 |= ((1 << PCINT9) | (1 << PCINT12));
    UCSR0B |= (1 << RXCIE0); 

    PORTB |= (1 << 5) | (1 << 4);

    sei(); 

	lcd_init();
	serial_init(MYUBRR);

	lcd_writecommand(1);
	//serial_stringout("@ab$");
	//_delay_ms(1000);
	short temperature = 0; 
	
	lcd_moveto(1,0);
	lcd_stringout(buff);
	char str[17];

	sprintf(str, "High: %2d Low: %2d",  hightemp, lowtemp);
	//serial_stringout("@+123$");


	ds_init();

	unsigned char tdata [2] = {'a','b'};
	
	hightemp= (int)eeprom_read_byte((void *)200);
	lowtemp= (int)eeprom_read_byte((void *)201);


	while (1) {
		ds_temp(tdata);


	
	temperature |= (tdata[1] << 8);
	temperature |= (tdata[0])	;
	/*if(tdata[1] < 0){
		temperature |= 0xFFFF0000; 

	}*/			
	temperature = (((temperature * 9) / (5*16)) + 32);
	
	char bufff[5];
	sprintf(str, "Temp:  %d %d",   temperature, bufff);
	lcd_moveto(1,0);
	lcd_stringout(str);

	
	snprintf(bufff, 5, "@+%d$" , temperature);
	serial_stringout(bufff);



	if ((PINB & (1<<5)) == 0){
		changingHigh = 1;                                   

	}
	else if ((PINB & (1<<4)) == 0){
		changingHigh = 0;
	}

	if (stringready == 1)
        {
            //display output on second line of arduino LCD
            //lcd_writecommand(1);
            lcd_moveto(1,14);
            char strr[8];
            sprintf(strr, "%c", bufff);
            lcd_stringout(strr);
            stringready=0;

        }
        if (changed) { 
	    	changed = 0;
	    	lcd_moveto(0,0);
			char str[16];
			sprintf(str, "High: %2d Low: %2d",  hightemp, lowtemp);
			lcd_stringout(str);

        }
	}
return 0;
}



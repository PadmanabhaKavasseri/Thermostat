#include <avr/io.h>

#include "adc.h"


void adc_init(unsigned char channel)
{
	// Initialize the ADC

	// reference vcc

    ADMUX &= ~(1 << REFS1);
    ADMUX |= (1 << REFS0);

    //ADMUX &= ~(0x0F);
    char mask = 0x0F;
    ADMUX &= ~mask;
    ADMUX |= (channel & mask);
    //ADMUX |= channel; // select channel

    ADCSRA |= (1 << ADPS2);
    ADCSRA |= (1 << ADPS1);
    ADCSRA |= (1 << ADPS0); 

    // 8BIT PRECISION 
    ADMUX |= (1 << ADLAR);

    // enable adc
    ADCSRA |= (1 << ADEN);

    


}

unsigned char adc_sample()
{
	ADCSRA |= (1 << ADSC);
	while ((ADCSRA & (1 << ADSC)) != 0)
	{}
	unsigned char result = ADCH;
	return result;
    // Convert an analog input and return the 8-bit result

}

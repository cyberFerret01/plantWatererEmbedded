/*
 * plantWaterer.c
 *
 * Created: 26/11/2020 22:33:50
 * Author : bear
 */ 
#define D0 eS_PORTB4
#define D1 eS_PORTB3
#define D2 eS_PORTB2
#define D3 eS_PORTC1
#define D4 eS_PORTC2
#define D5 eS_PORTC3
#define D6 eS_PORTC4
#define D7 eS_PORTC5
#define RS eS_PORTD4
#define EN eS_PORTB5
#define F_CPU 16000000UL
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU/(BAUDRATE*16UL)))-1)

#include "lcd.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

//drip tray full
ISR(INT0_vect){
	Lcd4_Clear();
	drainWater(150);
}

//water bucket empty
ISR(INT1_vect){
	Lcd4_Clear();
	Lcd4_Write_String("Refill bucket");
	_delay_ms(10);
}

void USART_init(void){
	UBRR0H=(uint8_t)(BAUD_PRESCALLER>>8); //set baud rate
	UBRR0L=(uint8_t)(BAUD_PRESCALLER);
	UCSR0B=(1<<RXEN0)|(1<<TXEN0);//enable transmit
	UCSR0C=(3<<UCSZ00);//set 8-bit (default)
	
	PORTB |= (1 << 5);
}

void USART_send(unsigned char data){
	while(!(UCSR0A&(1<<UDRE0)));  // check if data is sent
	UDR0= data;
	}

	unsigned char USART_receive(void){
		while(!(UCSR0A&(1<<RXC0)));  // Wait to receive data
		return UDR0;// Read data from UDR
	}

void USART_putstring(char* fullstring){
	//keep sending until the next char has value 0 (it's null)
	while (*fullstring != 0x00)
	{
		USART_send(*fullstring ++);
	}
}

void customDelay(int amount){
	for (int i = 0; i < amount; i++)
	{
		_delay_ms(1000);
	}
	
}

int sendMoisture(int level) {
	
	char s[10001];
			
	dtostrf(level,3,2,s);
			
	USART_putstring(s);
	USART_send('\n');
	_delay_ms(100);
	
	return level;
	}

int pumpWater(int length) {
	Lcd4_Clear();
	Lcd4_Write_String("Pumping...");
	
	PORTD |= (1 << 6);
	customDelay(length);
	PORTD &= ~(1 << 6);
	
	return 2;
}

int drainWater(int Dlength) {
	//2.50 measured time
	
	Lcd4_Write_String("Draining...");
	
	PORTD &= ~(1 << 6);	
	PORTD |= (1 << 7);
	customDelay(Dlength);
	PORTD &= ~(1 << 7);

	return 3;
}


int main(void)
{

	//PD3/2 = floatSwitch1/2 (input)
	//motor PWM on PD5 (out)
	//motor forwward/backwards 7/6 (out)
	
	//setup IO pins
	DDRC = 0b1111110;
	DDRD = 0b00;
	DDRD = 0b11110000;
	DDRB= 0b111110;
	
	//PWM initialization
	DDRD|=(1<<5);
	TCCR0A=0b00100011;
	OCR0B = 255;	
	//PWM on
	TCCR0B=0b101;

	//ADC/moisture sensor initialization 	
	ADMUX|=(1<<REFS0); //set reference voltage
	ADCSRA|=(1<<ADPS1)|(1<<ADPS0);//ADC clock prescaler/ 8
	ADCSRA|=(1<<ADEN);//enables the ADC
		
	int moistureSensor = 0;
	int waterTh = 950;
	int waterAmount = 80;

	//Serial communication initialization
    USART_init();
	char serial = '0';
	
	//Interrupt(s) initialization
	EIMSK = 0b11;
	sei();
	
	//LCD initialization
	Lcd4_Init();
	
	Lcd4_Write_String("->");
	int menu = 0;
	char s[10001]; //for reading (int) to char array
	
	//areas of code put into functions to reduce how much is going on in main loop (while (1){}) for ease of reading
	void userInterface(){
		//LCD UI thousands are main menus, hundreds and sub menus and onces are from confirmed selection
		//indenting here represents the depth of the selection (calibrating -> watering early -> updating... is depth 3 so 3 indents)
		switch(menu)
		{
			
			case 1000:
			Lcd4_Write_String("Calibrate ->");
			_delay_ms(10);
			Lcd4_Clear();
			break;
			
			case 1001:
			Lcd4_Write_String("Watering early");
			_delay_ms(10);
			Lcd4_Clear();
			break;
			
			case 1002:
			waterTh = waterTh + 1;
			Lcd4_Write_String("Updated");
			_delay_ms(1000);
			Lcd4_Clear();
			menu = 1001;
			break;

			case 1101:
			Lcd4_Write_String("Watering late");
			_delay_ms(10);
			Lcd4_Clear();
			break;
			
			case 1102:
			waterTh = moistureSensor-1;
			Lcd4_Write_String("Updated");
			_delay_ms(1000);
			Lcd4_Clear();
			menu = 1101;
			break;
			
			case 1201:
			Lcd4_Write_String("Add 100ml");
			_delay_ms(10);
			Lcd4_Clear();
			break;
			
			
			case 1202:
			waterAmount += 10;
			Lcd4_Write_String("Updated");
			_delay_ms(1000);
			Lcd4_Clear();
			menu = 1201;
			break;
			
			case 1301:
			Lcd4_Write_String("Remove 100ml");
			_delay_ms(10);
			Lcd4_Clear();
			break;
			
			case 1302:
			waterAmount += -10;
			if (waterAmount == 0) waterAmount = 1;
			Lcd4_Write_String("Updated");
			_delay_ms(1000);
			Lcd4_Clear();
			menu = 1301;
			break;
			
			case 2000:
			Lcd4_Write_String("Test ->");
			_delay_ms(10);
			Lcd4_Clear();
			break;
			
			case 2001:
			Lcd4_Write_String("Pump");
			_delay_ms(10);
			Lcd4_Clear();
			break;
			
			case 2002:
			pumpWater(waterAmount/10);
			menu = 2001;
			break;
			
			case 2101:
			Lcd4_Write_String("Drain");
			_delay_ms(10);
			Lcd4_Clear();
			break;
			
			case 2102:
			drainWater(waterAmount/10);
			menu = 2101;
			break;
			
			case 2201:
			Lcd4_Write_String("Sensor Val:");
			_delay_ms(10);
			Lcd4_Clear();
			break;
			
			case 2202:
			dtostrf(moistureSensor,3,2,s);
			Lcd4_Write_String(s);
			_delay_ms(10);
			Lcd4_Clear();
			break;
			
			default:
			Lcd4_Write_String("Welcome!! ->");
			_delay_ms(10);
			Lcd4_Clear();
			menu = 0;
			break;
		}
		Lcd4_Clear();
	}
	void serialInterface(){
		
		//if the serial flag is risen then read it (if there's anything coming from th serial bus)
		if ((UCSR0A&(1<<RXC0))) serial = USART_receive();
		
		//Serial Debugging
		switch(serial)
		{
			case '1':
			sendMoisture(moistureSensor);
			break;
					
			case '2':
			pumpWater(1);
			break;
					
			case '3':
			drainWater(1);
			break;
					
			case '4':
			Lcd4_Write_String("Testing....");
			_delay_ms(3000);
			Lcd4_Clear();
			serial = 'q';
			break;
		}
		
	}
	void readButtons(){
		for (int i = 0; i <2; i++){

			if(PINB & (1<<i)){
				_delay_ms(10);
				//double check to see if switch was actually pressed
				if(PINB & (1<<i)){
					
					if(i == 0) menu+= 1; //confirm selection (move one menu deeper)
					
					//move across depth (between different options)
					if(i == 1){
						if (menu%1000 ==0){menu += 1000;} //main menus
						else{menu+=100;} //sub menus
					}
					//extra denounce
					_delay_ms(1000);
				}
			}
		}
	}	
	
	
    while (1){
		
		userInterface();
		readButtons();
		serialInterface();

		//read in mositure sensor value
		ADCSRA|=(1<<ADSC);//start ADC conversion
		while((ADCSRA&(1<<ADSC))){} // wait till the ADC is done
		uint8_t theLowADC=ADCL; //read lower bits
		moistureSensor=ADCH << 8|theLowADC; //read upper bits
		
		//check if the sensor is reading a value above the threshold if it is then water the soil 
		if (moistureSensor > waterTh)
		{
			pumpWater(waterAmount);
		}
	}
}
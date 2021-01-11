Automatic plant water
This project monitors a moisture sensor and applies a set amount of water once the threshold is past 
This project also has an auto drain and an auto alert facility if the drip tray is full or the reservoir is empty 
The LCD display allows the user to calibrate the sensor or water dosing amount 
The LCD and serial interface allow the user to test the sensor and pump
The serial interface allows data logging through the 0 and 1 pins

Readbuttion(): Reads the two input buttons
userInterface(): performs an action depending on the combination of buttons pressed
serialInterface(): performs an action depending on the input 
drainWater(int Dlength): drains water from the drip tray for a given length
pumpWater(int Plength): waters soil for a given time
sendMoisture(): sends the moisture sensor value to the serial port
customDelay(): creates a delay for a given time in seconds
USART_init(): initializes the serial interface
USART_send(): sends char to serial interface
USART_recieve(): receives a char array from serial interface



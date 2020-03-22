/*
 * Filename: bump_bot_ece111.c
 * Created: 3/21/2020 4:07:49 PM
 * Author : Tristan Luther
 * Purpose: To display the Bump Bot action for ECE111 
 *          (rewritten in embedded C March 2020)
 * Web: http://tristanluther.com
 */ 

/**************** Macros/Globals ***********/
#ifndef F_CPU
#define F_CPU 16000000UL //Set clock speed to 16MHz
#endif

unsigned char command = 'h'; //The current command from the USART communication/default 'h' for halt

/*************** Libraries ****************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/************** Functions *****************/

//Function for setting up the USART communication for the ATmega168
void USART_init(uint16_t baud){
	//Set the baud rate
	UBRR0H = (uint16_t) (baud>>8);
	UBRR0L = (uint16_t)baud;
	//Enable the receiver with interrupts
	UCSR0B = (1<<RXEN0) | (1<<RXCIE0);
	//Set-up the frame format for the USART communication (8-bits 1 stop bit Parity Disabled)
	UCSR0C = (0<<USBS0) | (3<<UCSZ00);
	return; //Go back to previous location
}

//Function to empty the USART Receiver and Transmit Buffer
void USART_flush(){
	unsigned char dummy; //Dummy variable to clear the buffer with
	//Check if the RXC0 flag is cleared
	while(UCSR0A & (1<<RXC0)){
		dummy = UDR0; //Flush the FIFO buffer
	}
	return; //Go back to previous location
}

//Function to read the UDR0 Buffer from the receive
unsigned char USART_recieve(){
	//Wait for data to be received
	while(!(UCSR0A & (1<<RXC0)));
	//Return received data from the buffer
	return UDR0; //Go back to previous location
}

//Function to move forward
void Move_Fwd(){
	PORTD &= ~((1<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7)); //Clear the output bits on port D
	PORTD |= (0<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7); //Or the PORTD with forward bits
	PORTB &= ~((1<<PB0) | (1<<PB1)); //Clear the output bits on port B
	PORTB |= (0<<PB0) | (1<<PB1); //Or the PORTB with the backward bits
	return; //Go back to previous location
}

//Function to move backward
void Move_Bck(){
	PORTD &= ~((1<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7)); //Clear the output bits on port D
	PORTD |= (1<<PD4) | (1<<PD5) | (1<<PD6) | (0<<PD7); //Or the PORTD with the backward bits
	PORTB &= ~((1<<PB0) | (1<<PB1)); //Clear the output bits on port B
	PORTB |= (1<<PB0) | (0<<PB1); //Or the PORTB with the backward bits
	return; //Go back to previous location
}

//Function to move left
void Move_Lft(){
	PORTD &= ~((1<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7)); //Clear the output bits on port D
	PORTD |= (0<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7); //Or the PORTD with the left bits
	PORTB &= ~((1<<PB0) | (1<<PB1)); //Clear the output bits on port B
	PORTB |= (1<<PB0) | (0<<PB1); //Or the PORTB with the left bits
	return; //Go back to previous location
}

//Function to move right
void Move_Rgt(){
	PORTD &= ~((1<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7)); //Clear the output bits on port D
	PORTD |= (1<<PD4) | (1<<PD5) | (1<<PD6) | (0<<PD7); //Or the PORTD with the right bits
	PORTB &= ~((1<<PB0) | (1<<PB1)); //Clear the output bits on port B
	PORTB |= (0<<PB0) | (1<<PB1); //Or the PORTB with the right bits
	return; //Go back to previous location
}

//Function to halt the bot
void Halt(){
	PORTD &= ~((1<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7)); //Clear the output bits on port D
	PORTD |= (0<<PD4) | (0<<PD5) | (0<<PD6) | (0<<PD7); //Or the PORTD with the halt bits
	PORTB &= ~((1<<PB0) | (1<<PB1)); //Clear the output bits on port B
	PORTB |= (0<<PB0) | (0<<PB1); //Or the PORTB with the halt bits
	return; //Go back to previous location
}

/******* Interrupt Service Routines *******/

//Interrupt Service Routine for the left bumper external interrupt (INT0)
ISR(INT0_vect){
	//Left Bump Action
	//Have the bot back up for one second
	Move_Bck();
	_delay_ms(1000);
	//Have the bot turn to the right
	Move_Rgt();
	_delay_ms(500);
	//Clear any queued up interrupts by clearing the EIFR Flag
	EIFR = (1<<INTF1) | (1<<INTF0);
	//Bot will return to while(1) and resume the previous command
}

//Interrupt Service Routine for the right bumper external interrupt (INT1)
ISR(INT1_vect){
	//Right Bump Action
	//Have the bot back up for one second
	Move_Bck();
	_delay_ms(1000);
	//Have the bot turn to the left
	Move_Lft();
	_delay_ms(500);
	//Clear any queued up interrupts by clearing the EIFR Flag
	EIFR = (1<<INTF1) | (1<<INTF0);
	//Bot will return to while(1) and resume the previous command
}

//Interrupt Service Routine for the receive interrupt (RXCIE0)
ISR(USART_RX_vect){
	//Receive Command Action
	command = USART_recieve();
	//Flush the buffer
	USART_flush();
}

/************** Main **********************/
int main(void){
    //Set-up the Data Direction for Port D
	/*
		Pin Descriptions:
		PD0: Used for USART TX: Output (1)
		PD1: Used for USART RX: Input (0)
		PD2: Used for Left Bump external interrupt: Input (0)
		PD3: Used for Right Bump external interrupt: Input (0)
		PD4: Used for Motor Direction 1A: Output (1)
		PD5: Used for Motor Control enable 1,2: Output (1)
		PD6: Used for Motor Control enable 3,4: Output (1)
		PD7: Used for Motor Direction 2A: Output (1)
	*/
	DDRD |= (1<<PD0) | (0<<PD1) | (0<<PD2) | (0<<PD3) | (1<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7);
	//Set the default values for outputs to zero and inputs to have pull-up resistors
	PORTD |= (0<<PD0) | (1<<PD1) | (1<<PD2) | (1<<PD3) | (0<<PD4) | (0<<PD5) | (0<<PD6) | (0<<PD7);
	
	//Set-up the Data Direction for Port B
	/*
		Pin Descriptions:
		PB0: Used for Motor Direction 3A: Output (1)
		PB1: Used for Motor Direction 4A: Output (1)
	*/
	DDRB |= (1<<PB0) | (1<<PB1);
	//Set the default values for outputs to zero
	PORTB |= (0<<PB0) | (0<<PB1);
	
	//Set-up the External Interrupts for the bump buttons (PD2 -> INT0 & PD3 -> INT1)
	//Set-up the interrupt to trigger on the falling edge of the signal
	EICRA = (1<<ISC11) | (0<<ISC10) | (1<<ISC01) | (0<<ISC00);
	//The interrupt mask to enable specific external interrupts
	EIMSK = (1<<INT1) | (1<<INT0);
	
	//Set up the USART communication
	uint16_t baud = 103; //Baud rate of 9600bps without double data rate
	USART_init(baud);
	
	//Set-up the Global External Interrupt
	sei();
	
	//End of initialization/following code will run forever
    while(1){
		/*
			State machine for bot actions (change of command variable):
			'h': Halt (Motor controller is disabled. PD4 = 0/PD5 = 0/PD6 = 0/PD7 = 0/PB0 = 0/PB1 = 0)
			'f': Forward (Forward Movement. PD4 = 1/PD5 = 1/PD6 = 1/PD7 = 0/PB0 = 1/PB1 = 0)
			'b': Backward (Backward Movement. PD4 = 0/PD5 = 1/PD6 = 1/PD7 = 1/PB0 = 0/PB1 = 1)
			'l': Left (Left Movement. PD4 = 0/PD5 = 1/PD6 = 1/PD7 = 1/PB0 = 1/PB1 = 0)
			'r': Right (Right Movement. PD4 = 1/PD5 = 0/PD6 = 1/PD7 = 1/PB0 = 0/PB1 = 1)
		*/
		switch(command){
			case 'h':
				Halt();
				break;
			case 'f':
				Move_Fwd();
				break;
			case 'b':
				Move_Bck();
				break;
			case 'l':
				Move_Lft();
				break;
			case 'r':
				Move_Rgt();
				break;
			default:
				Halt();
				break;
		}
    }
}


/******************************************************************************
 * Project Name: EE 128 Mini Project - Door Lock
 * File Name: main.c
 * Author: J. Fababier, M. Henson
 * Submission Date: [December 06, 2024]
 * Description:
 *   This project simulates a simple digital door lock's behavior using the following components:
 *   - 1x 4-by-4 membrane keypad for user input,
 *   - 1x passive buzzer for audio feedback,
 *   - 1x 2-phase bipolar stepper motor to simulate a locking mechanism.
 *
 * Notes:
 *   [Any additional notes, assumptions, or limitations regarding the program.]
 *
 *****************************************************************************/

#include "fsl_device_registers.h"

// Macro definition for the number of tasks in the system
#define NUM_TASKS 3

// Task structure definition
typedef struct _task {
	signed char state;              // Task's current state
	unsigned long period;           // Task period (in ms)
	unsigned long elapsedTime;      // Time elapsed since last task tick (in ms)
	int (*TickFct)(int);            // Pointer to the task's state machine function
} task;

// Array to hold the tasks in the system
task tasks[NUM_TASKS];

// Task periods (in ms). Adjust based on desired behavior.
const unsigned long GCD_PERIOD = 5;	// Common divisor period
const unsigned long SM1_PERIOD = 250;	// State machine 1 period
const unsigned long SM2_PERIOD = 1000;	// State machine 2 period
const unsigned long SM3_PERIOD = 15;	// State machine 3 period

// PIT0 Interrupt Service Routine (ISR)
void PIT0_IRQHandler(void) {
	// Clear the interrupt flag for PIT channel 0
	PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;

	// Iterate over all tasks and handle execution
	for (unsigned char i = 0; i < NUM_TASKS; i++) {
		if (tasks[i].elapsedTime >= tasks[i].period) {
			// Execute the task's tick function and update its state
			tasks[i].state = tasks[i].TickFct(tasks[i].state);
			// Reset elapsed time for the task
			tasks[i].elapsedTime = 0;
		}
		// Increment elapsed time by the GCD_PERIOD
		tasks[i].elapsedTime += GCD_PERIOD;
	}
}

// Function to configure the PIT timer for the desired period
void TimerSet(unsigned long period) {
	// Enable clock for PIT module
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;

	// Enable the PIT module
	PIT->MCR &= ~PIT_MCR_MDIS_MASK;

	// Calculate the timer load value for a 1 ms interrupt
	uint32_t timerLoadValue = (period * (SystemCoreClock / 1000)) - 1;
	PIT->CHANNEL[0].LDVAL = PIT_LDVAL_TSV(timerLoadValue);

	// Enable interrupts for PIT channel 0
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;
}

// Function to start the PIT timer
void TimerOn() {
	// Start the PIT timer on channel 0
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
	// Enable the PIT interrupt in the NVIC
	NVIC_EnableIRQ(PIT0_IRQn);
}

// Global variables
unsigned char password[] = {'1', '1', '1', '1'};	// Password as characters
unsigned char user_input[4] = {'\0'};			// Initialize all elements to 0
unsigned char input_index = 0;				// Index for the user input
unsigned char locked = 1;				// Locked state: 0 = unlocked, 1 = locked
unsigned char phases[4] = {0x36, 0x35, 0x39, 0x3A};	// Stepper motor phases

// Keypad mapping
static char keypad[4][4] = {
		{'1', '2', '3', 'A'},
		{'4', '5', '6', 'B'},
		{'7', '8', '9', 'C'},
		{'*', '0', '#', 'D'}
};

// Keypad input capture function
char get_key(void) {
	// Define the column pin mapping: indices correspond to physical pins PC4, PC5, PC7, PC8
	unsigned char column_pins[] = {4, 5, 7, 8};

	// Loop through each row
	for (unsigned char row = 0; row < 4; row++) {
		// Set the current row to LOW (active), others HIGH (inactive)
		GPIOC_PDOR = ~(1 << row) & 0x0F;

		// Check each column
		for (unsigned char col = 0; col < 4; col++) {
			if ((GPIOC_PDIR & (1 << column_pins[col])) == 0) {  // Column is LOW (key pressed)

				// Return the detected key from the keypad map
				return keypad[row][col];
			}
		}
	}

	// No key pressed, return null character
	return '\0';
}

// Lock system
void lock_system(void) {
	locked = 1;		// Set system to locked state
	input_index = 0;	// Reinitialize input_index to 0
	memset(user_input, '\0', sizeof(user_input));  // Reset user_input to all '\0'
}

// Unlock system
void unlock_system(void) {
	locked = 0;			// Set system to unlocked state
	input_index = 0;		// Reinitialize input index to 0
	GPIOB_PDOR &= ~(1 << 2);	// Turn off red LED
	GPIOB_PDOR |= (1 << 3);		// Turn on green LED
}

// Compare user-input to password
void compare_password(void) {
	unsigned char i;

	// Check if user_input matches the password
	for (i = 0; i < 4; i++) {
		if (user_input[i] != password[i]) {
			// If any character doesn't match, keep the system locked and indicate error
			lock_system();	// Reset lock
			return;		// Exit the function immediately
		}
	}

	// If all characters match, unlock the system
	unlock_system();
}

// Passive buzzer off, PWM mode disabled
void buzzer_off(void) {
	FTM3_MODE = 0x5;		// Enable FTM3
	FTM3_MOD = 10499;		// period
	FTM3_C6SC = 0x00;		// PWM active low
	FTM3_C6V = 5250;		// pulse width
	FTM3_SC = (1 << 3) | 1; 	// system clock; prescale 2
}

// Passive buzzer on, PWM mode enabled
void buzzer_on(void) {
	FTM3_MODE = 0x5;		// Enable FTM3
	FTM3_MOD = 10499;		// period
	FTM3_C6SC = 0x28;		// PWM active high
	FTM3_C6V = 5250;		// pulse width
	FTM3_SC = (1 << 3) | 1; 	// system clock; prescale 2
}

// Read the value of the specified GPIOD pin
void sync_motor_bit_to_led(unsigned int PORTD_PIN) {
	// Read the value of the specified GPIOD pin
	unsigned int bit_value = (GPIOD_PDOR & (1 << PORTD_PIN)) >> PORTD_PIN;

	// Control the LED on PB11 based on the bit value
	GPIOB_PDOR = bit_value ? (GPIOB_PDOR | (1 << 11)) : (GPIOB_PDOR & ~(1 << 11));
}

// State machine enumerations and declarations
enum SM1_Tick { SM1_INIT, SM1_IDLE, SM1_LOG_KEY, SM1_LOCK_SYSTEM, SM1_DEBOUNCE_KEY };				// States for state machine 1
enum SM2_Tick { SM2_INIT, SM2_IDLE, SM2_LOCK_SYSTEM, SM2_DEBOUNCE_KEY };					// States for state machine 2
enum SM3_Tick { SM3_INIT, SM3_MOTOR_LOCKED, SM3_MOTOR_UNLOCKED, SM3_TURN_MOTOR_CW, SM3_TURN_MOTOR_CCW };	// States for state machine 3
int SM1_Tick(int state);	// Tick function for state machine 1
int SM2_Tick(int state);	// Tick function for state machine 2
int SM3_Tick(int state);	// Tick function for state machine 3

// State machine 1 tick function - Manage password logging
int SM1_Tick(int state) {
	// State transitions
	switch (state) {
	case SM1_INIT:
		state = SM1_IDLE;
		break;

	case SM1_IDLE:
		if (get_key() == '\0') {
			state = SM1_IDLE;
		}
		else if (get_key() == '#') {
			state = SM1_LOCK_SYSTEM;
		}
		else {
			state = SM1_LOG_KEY;
		}
		break;

	case SM1_LOG_KEY:
		buzzer_on();
		state = SM1_DEBOUNCE_KEY;
		break;

	case SM1_LOCK_SYSTEM:
		buzzer_on();
		lock_system();
		state = SM1_DEBOUNCE_KEY;
		break;

	case SM1_DEBOUNCE_KEY:
		buzzer_off();
		if (get_key() != '\0') {
			state = SM1_DEBOUNCE_KEY;
			break;
		}
		state = SM1_IDLE;
		break;

	default:
		break;
	}

	// State actions
	switch (state) {
	case SM1_INIT:
		break;

	case SM1_IDLE:
		GPIOB_PDOR &= ~(1 << 2);	// Turn off red LED
		GPIOB_PDOR &= ~(1 << 10);	// Turn off blue LED
		break;

	case SM1_LOG_KEY:
		user_input[input_index] = get_key();
		input_index++;
		break;

	case SM1_LOCK_SYSTEM:
		GPIOB_PDOR |= (1 << 2);
		break;

	case SM1_DEBOUNCE_KEY:
		GPIOB_PDOR |= (1 << 10);	// Turn on blue LED
		break;

	default:
		break;
	}

	return state;
}

// State machine 2 tick function - Handle password comparison, LED interfacing
int SM2_Tick(int state) {
	// State transitions
	switch (state) {
	case SM2_INIT:
		state = SM2_IDLE;
		break;

	case SM2_IDLE:
		state = SM2_IDLE;
		break;

	default:
		break;
	}

	// State actions
	switch (state) {
	case SM2_INIT:
		break;

	case SM2_IDLE:
		if (user_input[3] != '\0') {
			compare_password();
		}
		GPIOB_PDOR = (!locked) ? (GPIOB_PDOR | (1 << 3)) : (GPIOB_PDOR & ~(1 << 3)); // Handle green LED
		break;

	default:
		break;
	}

	return state;
}

// State machine 3 tick function - Stepper motor operation
int SM3_Tick(int state) {
	static unsigned char i = 0;		// Variable to track steps
	static unsigned char current_step = 0;  // Current step index

	// State transitions
	switch (state) {
	case SM3_INIT:
		state = SM3_MOTOR_LOCKED;
		break;

	case SM3_MOTOR_LOCKED:
		if (locked) {
			state = SM3_MOTOR_LOCKED;
			break;
		}
		if (!locked) {
			state = SM3_TURN_MOTOR_CW;
		}
		break;

	case SM3_MOTOR_UNLOCKED:
		if (!locked) {
			state = SM3_MOTOR_UNLOCKED;
		}
		if (locked) {
			state = SM3_TURN_MOTOR_CCW;
		}
		break;

	case SM3_TURN_MOTOR_CW:
		if (i == 32) {
			i = 0;
			state = SM3_MOTOR_UNLOCKED;
		}
		else {
			state = SM3_TURN_MOTOR_CW;
		}
		break;

	case SM3_TURN_MOTOR_CCW:
		if (i == 32) {
			i = 0;
			state = SM3_MOTOR_LOCKED;
		}
		else {
			state = SM3_TURN_MOTOR_CCW;
		}
		break;

	default:
		break;
	}

	// State actions
	switch (state) {
	case SM3_INIT:
		break;

	case SM3_MOTOR_LOCKED:
		break;

	case SM3_MOTOR_UNLOCKED:
		break;

	case SM3_TURN_MOTOR_CW:
		current_step = (current_step + 1) % 4;	// Increment step in circular manner
		GPIOD_PDOR = phases[current_step];
		sync_motor_bit_to_led(0);		// Toggle white LED
		i++;
		break;

	case SM3_TURN_MOTOR_CCW:
		current_step = (current_step - 1 + 4) % 4;	// Decrement step in circular manner
		GPIOD_PDOR = phases[current_step];
		sync_motor_bit_to_led(0);			// Toggle white LED
		i++;
		break;

	default:
		break;
	}

	return state;
}

// Main function
int main(void) {
	// Refer to "EE 128 Mini Project Pinout" for pinout details
	// Enable clock for PORTB, PORTC, PORTD
	SIM_SCGC5 |= (SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK);	// Enable clock for PORTB, PORTC, PORTD

	// Configure PORTB GPIO pins
	PORTB_GPCLR = 0x0C0C0100;	// Configure PB2, PB3, PB10, PB11 as GPIO
	GPIOB_PDDR |= 0x0C0C;		// Set PB2, PB3, PB10, PB11 as outputs
	GPIOB_PDOR |= 0x0000;		// Set PB2-PB3 as LOW

	// Configure PORTC GPIO pins
	PORTC_GPCLR = 0x01BF0100;	// Configure PC0-PC3, PC4, PC5, PC7, PC8 as GPIO
	GPIOC_PDDR |= 0x000F;		// Set PC0-PC3 as outputs (rows)
	GPIOC_PDDR &= ~0x01B0;		// Set PC4, PC5, PC7, PC8 as inputs (columns)
	GPIOC_PDOR |= 0x0000;		// Set outputs to LOW
	PORTC_PCR4 |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;	// Enable pull-up on PC4
	PORTC_PCR5 |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;	// Enable pull-up on PC5
	PORTC_PCR7 |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;	// Enable pull-up on PC7
	PORTC_PCR8 |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;	// Enable pull-up on PC8

	// Configure PC10 for PWM output (FTM3_CH6)
	// PC10 generates 1 kHz PWM signal
	// Refer to: - buzzer_off(), buzzer_on()
	SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;	// Enable clock for FTM3
	PORTC_PCR10 = 0x300;			// Port C Pin 10 as FTM3_CH6 (ALT3)

	// Configure PORTD GPIO pins
	PORTD_GPCLR = 0x003F0100;	// Configure pins PD0-PD5
	GPIOD_PDDR |= 0x003F;		// Set pins PD0-PD5 as outputs
	GPIOD_PDOR &= ~0x003F;		// Set outputs to low

	// Configure Task 1
	unsigned char i = 0;
	tasks[i].state = SM1_INIT;      // Set initial state
	tasks[i].period = SM1_PERIOD;   // Set task period
	tasks[i].elapsedTime = tasks[i].period; // Initialize elapsed time
	tasks[i].TickFct = &SM1_Tick;   // Set tick function

	// Configure Task 2
	i++;
	tasks[i].state = SM2_INIT;      // Set initial state
	tasks[i].period = SM2_PERIOD;   // Set task period
	tasks[i].elapsedTime = tasks[i].period; // Initialize elapsed time
	tasks[i].TickFct = &SM2_Tick;   // Set tick function

	// Configure Task 3
	i++;
	tasks[i].state = SM3_INIT;      // Set initial state
	tasks[i].period = SM3_PERIOD;   // Set task period
	tasks[i].elapsedTime = tasks[i].period; // Initialize elapsed time
	tasks[i].TickFct = &SM3_Tick;   // Set tick function

	// Configure and start the timer
	TimerSet(GCD_PERIOD);           // Set timer period
	TimerOn();                      // Start timer

	// Run program indefinitely
	while (1) {
		// Infinite loop
	}

	// Program should never reach here
	return 0;
}

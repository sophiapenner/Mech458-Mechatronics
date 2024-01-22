/* 	
	Course		: UVic Mechatronics 458
	Milestone	: 5
	Title		: Final Project - Classification
	Name 1:	Emiko Hourston				Student ID: V00914008
	Name 2:	Sophia Penner				Student ID: V00915678
	
	Description: This program controls a mechatronics inspection system that sorts four materials based on reflectivity.
				 A DC motor powers a conveyor belt, which transports materials through a reflectivity sensor to classify
				 them and add them into a linked list. An optical sensor marks the end of the conveyor, and when triggered
				 the part is dequeued from the linked list and the material is sorted into a stepper-powered bucket.

				 Two switches are used to (1) pause and resume the system, and (2) rampdown the system. The pause switch 
				 stops the conveyor and displays the count of each sorted material and the count of the unsorted materials.
				 A second press of the button resumes the conveyor and clears the LCD. The rampdown switch sorts the remaining 
				 materials on the conveyor then kills the system.
*/

#include <avr/interrupt.h>
#include <avr/io.h>
#include "lcd.h"
#include "LinkedQueue.h"
#include <stdlib.h>

// declare constants ======================================================
#define CCW 0b1110
#define CW 0b1101
#define BRAKE 0b1111    					    // brake HIGH
#define NUM_RUNS 10     					    // 48, make smaller for testing
#define step1 0b00101101						// 0b00000011
#define step2 0b00101011						// 0b00011000
#define step3 0b00011011						// 0b00000101
#define step4 0b00011101						// 0b00101000
#define CODE 0                                  // Material code index, used to classify materials
#define LOW 1                                   // Material low bound index
#define HIGH 2                                  // Material high bound index

// declare variables NOT to be changed once the system is running =========
unsigned int BLACK[3] = {0, 943, 1023};			// material format {CODE, LOW, HIGH}
unsigned int STEEL[3] = {1, 151, 750};			// LOW & HIGH updated after calibration
unsigned int WHITE[3] = {2, 751, 942};
unsigned int ALUMINUM[3] = {3, 0, 150};
unsigned int ACCEL90[50] = {15, 13, 9, 7, 7,	// Acceleration for turning bucket 90 degrees
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 9, 13, 15};
unsigned int ACCEL180[100] = {15, 13, 9, 7, 7,	// Acceleration for turning bucket 180 degrees
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 7, 7, 7,
							7, 7, 9, 13, 15};

// declare global variables ===============================================
volatile char STATE;
volatile unsigned int SYSTEMRUNNING = 1;		// flag used on pause/play
volatile unsigned int SORTED[4] = {0, 0, 0, 0};	// keeps track of how many of each material have been processed
volatile unsigned int ADC_result;				// store ADC value for each new conversion
volatile unsigned int MIN;					    // store minimum conversion value for one sensor pass
volatile unsigned int COUNT = 0;				// how many parts have been processed in OR sensor
volatile unsigned int CUR_pos;					// store current position of bucket (CW & CCW)
volatile unsigned int BUCKET_pos;				// store current position of bucket (stepper)
volatile unsigned int DESIRED_pos;				// store desired position of bucket (stepper)
volatile unsigned int RAMPDOWN = 0;				// set to 1 to trigger rampdown

// setup linked list ======================================================
link *head;			/* The ptr to the head of the queue */
link *tail;			/* The ptr to the tail of the queue */
link *newLink;		/* A ptr to a link aggregate data type (struct) */
link *rtnLink;		/* same as the above */
element eTest;		/* A variable to hold the aggregate data type known as element */

// declare functions ======================================================
void mTimer(int count);
void rTimer(int count);
void LCDsetup();
void INTsetup();
void PWMsetup();
void ADCsetup();
void clockwise(int steps);
void counterClockwise(int steps);
void homing(int steps);
void showResults();

int main(int argc, char *argv[]){
	
	CLKPR = 0x80;
	CLKPR = 0x01;							// sets system clock to 8MHz

    // set the PORT as inputs or outputs ==================================
	DDRA = 0xFF;							// stepper
	DDRB = 0xFF;		                    // Set PORTB to output
	DDRF = 0x00;                            // ADC INPUT
	DDRD = 0x00;                            // set interrupt to input
	DDRL = 0x00;                            // set port D to input for homing sensor

	// disable all of the interrupts ======================================
	cli(); 

	// setup timer ========================================================
	TCCR1B |= _BV(CS11); 

    // set up LCD, interrupts, PWM, ADC ===================================
    LCDsetup();
    INTsetup();
	PWMsetup();
    ADCsetup();

	// sets the Global Enable for all interrupts ==========================
	sei();	

	// set up linked list =================================================
    rtnLink = NULL;
	newLink = NULL;

    setup(&head, &tail);

    // initialize variables ===============================================
	MIN = 1023;
    STATE = 0;
	CUR_pos = 0;
	
	// home stepper =======================================================
	while ((PINL & 0x01) == 0x01){
		homing(1);
	}
	homing(8);			// go extra steps to center
	
	PORTB = CCW;		// start conveyor
	
	BUCKET_pos = 0;		// set bucket position to black
	
	goto POLLING_STAGE;

    /**********************************************************************
     * STAGE: Polling
     * DESC: Checks if rampdown switch has been pressed and all items sorted
     */

	POLLING_STAGE:

	// Sort rest of items on conveyor then shut down
	if ((RAMPDOWN == 1) && (isEmpty(&head))){ //size(&head, &tail) == 0) {
		mTimer(1000);
		PORTB = BRAKE;		// stop motor
		mTimer(10);
		PORTB = 0x00;		// kill motor
		showResults();
		while (1) {}		// stop program				
	}

	// state machine
	switch(STATE){
		case (0) :
		goto POLLING_STAGE;
		break;	//not needed but syntax is correct
		case (3) :
		goto BUCKET_STAGE;
		break;
		default :
		goto POLLING_STAGE;
	}//switch STATE
	
    
    /**********************************************************************
     * STAGE: Bucket
     * DESC: Dequeue item from linked list, rotate bucket to sort item
     */

	BUCKET_STAGE:

	DESIRED_pos = head->e.itemCode;		// need to get value of head before dequeueing
    dequeue(&head, &tail, &rtnLink);	// Remove the item at the head of the list
    free(rtnLink);						// Free up memory    

	PORTB = BRAKE;              		// stop conveyor

	// find difference between current position and desired position
	
	// order CW, rotate CCW for next: 
	// Black - 0
	// Steel - 1
	// White - 2
	// Aluminum - 3
	
	switch (BUCKET_pos - DESIRED_pos) {
		case(0):						// already at desired position
		break;
		case(1):
		case(-3):	
		mTimer(30);						// wait for previous part to drop
		clockwise(50);					// 50 steps away from desired position
		mTimer(20);
		break;
		case(-1):
		case(3):
		mTimer(30);						// wait for previous part to drop
		counterClockwise(50);			// 50 steps away from desired position
		mTimer(20);
		break;		
		case(2):
		case(-2):
		mTimer(30);						// wait for previous part to drop
		clockwise(100);					// 100 steps away from desired position
		mTimer(20);
		break;
	}
	
	BUCKET_pos = DESIRED_pos;			// set current bucket position
	SORTED[DESIRED_pos] += 1;			// increment the SORTED count
	PORTB = CCW;						// start conveyor and return to polling

	//Reset the state variable
	STATE = 0;
	goto POLLING_STAGE;

}

/**************************************************************************************/
/************************** INTERRUPT SERVICE ROUTINES ********************************/
/**************************************************************************************/


/**************************************************************************************

 * DESC: This interrupt fires when the rampdown button is pressed
*/
 
ISR(INT0_vect) {

	mTimer(20);								// 20ms debounce

	if ((PIND & 0x01) != 0x00) {
		rTimer(2500);						// start rampdown timer
	}

	while((PIND & 0x01) == 0x00){};         // wait for button to be released
	mTimer(20);								// 20ms debounce
}

/**************************************************************************************

 * DESC: This interrupt fires when the pause/play button is pressed
*/

ISR(INT1_vect) {
	
	if ((PIND & 0x02) == 0x00) {	
		mTimer(20);							// 20ms debounce
		
		switch(SYSTEMRUNNING) {             // Check the current status
			case (1):                       // conveyor is running
			PORTB = BRAKE;                  // stop motor
			SYSTEMRUNNING = 0;				// indicates system is paused
			showResults();					// LCD
			break;
			case(0):						// conveyor is not running
			PORTB = CCW;             		// resume motor
			SYSTEMRUNNING = 1;				// indicates system is running
			LCDClear();						// clear LCD results
			break;
			default:
			break;
		}		
	}

	while((PIND & 0x02) == 0x00){};         // wait for button to be released
	mTimer(20);								// 20ms debounce
}

/**************************************************************************************
 * DESC: This interrupt fires when reflectivity sensor is activated
*/

ISR(INT2_vect) {

	if((PIND & 0x04) == 0x04){				// Check that part is still in sensor (reduce noise)
   		MIN = 1023;							// Reset MIN for next part
		initLink(&newLink);					// Create new link
		enqueue(&head, &tail, &newLink);	// Add part to linked list
		ADCSRA |= _BV(ADSC);            	// Restart ADC
	}
}

/**************************************************************************************
 * DESC: This interrupt fires when end of travel sensor is activated
*/

ISR(INT3_vect) {

	if((PIND & 0x08) == 0x08){				// Check that part is still in sensor (reduce noise)
		STATE = 3;                          // Bucket state, remove material from fifo
    }
}

/**************************************************************************************
 * DESC: This interrupt fires when the ADC is done
*/

ISR(ADC_vect) {
	
	ADC_result = ADC;						// Store ADC value
	if (ADC_result < MIN) {         		// update MIN
		MIN = ADC_result;
	}
	
	if ((PIND & 0x04) == 0x04) {			// Check that part is still in sensor (reduce noise)
		ADCSRA |= _BV(ADSC);            	// Restart ADC		
	} else {
		COUNT++;                 			// iterate to next part
		if ((MIN >= WHITE[LOW]) && MIN <= WHITE[HIGH]) {		// classify item using material CODE
			newLink->e.itemCode = WHITE[CODE];
		} else if ((MIN >= BLACK[LOW]) && MIN <= BLACK[HIGH]) {
			newLink->e.itemCode = BLACK[CODE];
		} else if ((MIN >= ALUMINUM[LOW]) && MIN <= ALUMINUM[HIGH]) {
			newLink->e.itemCode = ALUMINUM[CODE];
		} else if ((MIN >= STEEL[LOW]) && MIN <= STEEL[HIGH]) {
			newLink->e.itemCode = STEEL[CODE];
		} 
	}
}

/**************************************************************************************
 * DESC: This interrupt fires once rTimer is done
*/

ISR(TIMER3_COMPB_vect) {
	RAMPDOWN = 1;

}

/**************************************************************************************
 * DESC: This interrupt is a catch-all ISR
*/

ISR(BADISR_vect) {

    LCDClear();
    LCDWriteStringXY(0, 0, "BAD ISR")
    mTimer(1000);

}

/**************************************************************************************/
/***************************** SUBROUTINES ********************************************/
/**************************************************************************************/


/**************************************************************************************
* DESC: Sets up the LCD
*/

void LCDsetup(){

	InitLCD(LS_BLINK|LS_ULINE);					// Initialize LCD module
	LCDClear(); 								// Clear LCD screen
	LCDWriteStringXY(0, 0, "initializing...");	// Provide feedback that LCD is working in main
	mTimer(1000);								// Delay to show message 
	LCDClear();
}

/**************************************************************************************
* DESC: Sets up the interrupts with rising or falling edge
*/

void INTsetup(){

	EIMSK |= (_BV(INT0) | _BV(INT1) | _BV(INT2) | _BV(INT3));       	// enable INT0, INT1, INT2 & INT3
    EICRA |= (_BV(ISC01) | _BV(ISC00));         // rising edge interrupt INT0 --> rampdown
    EICRA |= (_BV(ISC11));         				// falling edge interrupt INT1 --> pause and restart motor
	EICRA |= (_BV(ISC21) | _BV(ISC20)); 		// rising edge interrupt INT2 --> reflectivity sensor
	EICRA |= (_BV(ISC31) | _BV(ISC30));         // rising edge interrupt INT3 --> end of travel sensor
    EIFR |= _BV(INTF2);                 		// clear the interrupt flag

}

/**************************************************************************************
* DESC: Sets up the PWM to control the amount of voltage applied to a DC motor
*/

void PWMsetup(){

    // Set the timer to Fast PWM mode
	TCCR0A |= _BV(WGM00) | _BV(WGM01);          

    // In the Timer/Counter Control Register A, set the compare match output mode to clear 
    // (change to 0) on a compare match and set the output compare A (change to 1) when the 
    // timer reaches TOP. ? COM */
    TCCR0A |= _BV(COM0A1);
    
    //Set the prescale in TCCR0B to a reasonable value for the period of PWM signal 
    TCCR0B |= _BV(CS01);

    // set duty cycle
    OCR0A = 0x90;        
}

/**************************************************************************************
* DESC: Sets up the ADC
*/

void ADCsetup(){

	// by default, the ADC input (analog input is set to be ADC0 / PORTF0
	ADCSRA |= _BV(ADEN); 						// enable ADC
	ADCSRA |= _BV(ADIE); 						// enable interrupt of ADC
	ADMUX |= _BV(REFS0); 						// Voltage reference selection for ADC is AVCC with 
												// external capacitor at AREF pin
	ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);		// prescaler
}

/**************************************************************************************
* DESC: Stall the program
* INPUT: the number of millisecond wait
*/

void mTimer(int count){
	/* The system clock is 8 MHz. You can actually see the crystal oscillator (16MHz) which is a silver looking can on the board.
	You can use a pre-scaler on system clock to lower the speed. The timer runs on the CPU Clock which is a function of the system clock.
	You can also use a pre-scaler on the Timer, by 1, 8, 64, 256, or 1024 to lower the speed.
	The system clock has been pre-scaled by 2. This means it's running at half speed, 8MHz.
	See Technical manual for ATmega2560 (ie. full manual) and look up "16-bit Timer/Counter1". */	
	
	/* Variable declarations */
	
	int i;							// Keeps track of loop number
	i=0;							// initializes loop counter
	
	/* Set the Waveform Generation mode bit description to Clear Timer on Compare Math mode (CTC) only */
	
	TCCR1B |= _BV(WGM12);			// set WGM (spread over two register) bits to 0100, see page 145
	OCR1A = 0x03E8;					// Set Output Compare Register for 1000 cycles = 1ms
	TCNT1 = 0x0000;					// Sets initial value of Timer counter to 0x0000

    // Timer Interrupt Flag Register
	TIFR1 |=_BV(OCF1A);			    // Resets the timer by writing a 1, telling the hardware to set timer to 0
	
	/* Poll the timer to determine when the timer has reached 0x03E8 */
	
	while(i<count){
		if((TIFR1 & 0x02) == 0x02){
			TIFR1 |= _BV(OCF1A);	// Clear interrupt flag by writing a ONE to the bit
			i++;					// Increment loop number
		}							// End if
	}								// End while
	return;							// End mTimer
}									// Close mTimer

/**************************************************************************************
* DESC: Wait for parts on conveyor to be processed on rampdown
* INPUT: the number of millisecond wait
*/

void rTimer(int count){
	//int i = 0;
	/* Set the Waveform Generation mode bit description to Clear Timer on Compare Math mode (CTC) only */
	
	TCCR3B |= _BV(WGM32);			// CTC mode, set WGM (spread over two register) bits to 0100, see page 145
	OCR3B = 0x6400;					// Set Output Compare Register for 1000 cycles
	TCNT3 = 0x0000;					// Sets initial value of Timer counter to 0x0000
	TIMSK3 = TIMSK3 | 0b00000100;
	TIMSK3 |= _BV(OCIE3B);			// Enable the output compare interrupt enable
	TCCR3B |= _BV(CS32) | _BV(CS30);			// Starting timer and clock prescaler(256)

	return;

}	
/**************************************************************************************
* DESC: initializes the linked queue to 'NULL' status
* INPUT: the head and tail pointers by reference
*/

void setup(link **h,link **t){
	*h = NULL;		/* Point the head to NOTHING (NULL) */
	*t = NULL;		/* Point the tail to NOTHING (NULL) */
	return;
}/*setup*/

/**************************************************************************************
* DESC: This initializes a link and returns the pointer to the new link or NULL if error 
* INPUT: the head and tail pointers by reference
*/
void initLink(link **newLink){
	//link *l;
	*newLink = malloc(sizeof(link));
	(*newLink)->next = NULL;
	return;
}/*initLink*/

/****************************************************************************************
*  DESC: Accepts as input a new link by reference, and assigns the head and tail		
*  of the queue accordingly				
*  INPUT: the head and tail pointers, and a pointer to the new link that was created 
*/
/* will put an item at the tail of the queue */
void enqueue(link **h, link **t, link **nL){

	if (*t != NULL){
		/* Not an empty queue */
		(*t)->next = *nL;
		*t = *nL; //(*t)->next;
	}/*if*/
	else{
		/* It's an empty Queue */
		//(*h)->next = *nL;
		//should be this
		*h = *nL;
		*t = *nL;
	}/* else */
	return;
}/*enqueue*/

/**************************************************************************************
* DESC: Removes the link from the head of the list and assigns it to deQueuedLink
* INPUT: The head and tail pointers, and a ptr 'deQueuedLink' 
* 		 which the removed link will be assigned to
*/
/* This will remove the link and element within the link from the head of the queue */
void dequeue(link **h, link **t, link **deQueuedLink){
	/* ENTER YOUR CODE HERE */
	*deQueuedLink = *h;	// Will set to NULL if Head points to NULL
	/* Ensure it is not an empty queue */
	if (*h != NULL){
		*h = (*h)->next;
	} 
	
	if (*h == NULL) {
		*t = NULL;
	}
	
	return;
}/*dequeue*/

/**************************************************************************************
* DESC: Peeks at the first element in the list
* INPUT: The head pointer
* RETURNS: The element contained within the queue
*/
/* This simply allows you to peek at the head element of the queue and returns a NULL pointer if empty */
element firstValue(link **h){
	return((*h)->e);
}/*firstValue*/

/**************************************************************************************
* DESC: deallocates (frees) all the memory consumed by the Queue
* INPUT: the pointers to the head and the tail
*/
/* This clears the queue */
void clearQueue(link **h, link **t){

	link *temp;

	while (*h != NULL){
		temp = *h;
		*h=(*h)->next;
		free(temp);
	}/*while*/
	
	/* Last but not least set the tail to NULL */
	*t = NULL;		

	return;
}/*clearQueue*/

/**************************************************************************************
* DESC: Checks to see whether the queue is empty or not
* INPUT: The head pointer
* RETURNS: 1:if the queue is empty, and 0:if the queue is NOT empty
*/
/* Check to see if the queue is empty */
char isEmpty(link **h){
	/* ENTER YOUR CODE HERE */
	return(*h == NULL);
}/*isEmpty*/

/**************************************************************************************
* DESC: Obtains the number of links in the queue
* INPUT: The head and tail pointer
* RETURNS: An integer with the number of links in the queue
*/
/* returns the size of the queue*/
int size(link **h, link **t){

	link 	*temp;			/* will store the link while traversing the queue */
	int 	numElements;

	numElements = 0;

	temp = *h;			/* point to the first item in the list */

	while(temp != NULL){
		numElements++;
		temp = temp->next;
	}/*while*/
	
	return(numElements);
}/*size*/

/**************************************************************************************
* DESC: Commands the clockwise rotation of the motor, based on the number of steps, the CUR_pos global variable, and mTimer.
* INPUT: The number of steps to run 
*/
void clockwise (int steps){
	int i;
	if (steps == 50) {					// 90 degree rotation
		for (i=0; i<steps; i++){
			switch(CUR_pos){
				case (0):
				PORTA = step2;
				CUR_pos = 1;
				mTimer(ACCEL90[i]);
				break;
				case (1):
				PORTA = step3;
				CUR_pos = 2;
				mTimer(ACCEL90[i]);
				break;
				case (2):
				PORTA = step4;
				CUR_pos = 3;
				mTimer(ACCEL90[i]);
				break;
				case (3):
				PORTA = step1;
				CUR_pos = 0;
				mTimer(ACCEL90[i]);
				break;
				default:
				break;
			}
		}
			
	} else if (steps == 100) {			// 180 degree rotation
		for (i=0; i<steps; i++){
			switch(CUR_pos){
				case (0):
				PORTA = step2;
				CUR_pos = 1;
				mTimer(ACCEL180[i]);
				break;
				case (1):
				PORTA = step3;
				CUR_pos = 2;
				mTimer(ACCEL180[i]);
				break;
				case (2):
				PORTA = step4;
				CUR_pos = 3;
				mTimer(ACCEL180[i]);
				break;
				case (3):
				PORTA = step1;
				CUR_pos = 0;
				mTimer(ACCEL180[i]);
				break;
				default:
				break;
			}
		}
	}
}

/**************************************************************************************
* DESC: Commands the counter clockwise rotation of the motor, based on the number of steps, the CUR_pos global variable, and mTimer.
* INPUT: The number of steps to run 
*/
void counterClockwise (int steps){
	int i;
	if (steps == 50) {					// 90 degree rotation
		for (i=0; i<steps; i++){
			switch(CUR_pos){
				case (0):
				PORTA = step4;
				CUR_pos = 3;
				mTimer(ACCEL90[i]);
				break;
				case (1):
				PORTA = step1;
				CUR_pos = 0;
				mTimer(ACCEL90[i]);
				break;
				case (2):
				PORTA = step2;
				CUR_pos = 1;
				mTimer(ACCEL90[i]);
				break;
				case (3):
				PORTA = step3;
				CUR_pos = 2;
				mTimer(ACCEL90[i]);
				break;
				default:
				break;
			}	
		}
	} else if (steps == 100) {			// 180 degree rotation
		for (i=0; i<steps; i++){
			switch(CUR_pos){
				case (0):
				PORTA = step4;
				CUR_pos = 3;
				mTimer(ACCEL180[i]);
				break;
				case (1):
				PORTA = step1;
				CUR_pos = 0;
				mTimer(ACCEL180[i]);
				break;
				case (2):
				PORTA = step2;
				CUR_pos = 1;
				mTimer(ACCEL180[i]);
				break;
				case (3):
				PORTA = step3;
				CUR_pos = 2;
				mTimer(ACCEL180[i]);
				break;
				default:
				break;
			}	
		}
	}
}

/**************************************************************************************
* DESC: Commands the counter clockwise rotation of the motor, based on the number of steps, the CUR_pos global variable, and mTimer.
		Uses a non-optimized speed and single steps (see while loop in main).
* INPUT: The number of steps to run 
*/
void homing(int steps) {
	int delay = 20;
	for (int i=0; i<steps; i++){
		switch(CUR_pos){
			case (0):
			PORTA = step4;
			CUR_pos = 3;
			mTimer(delay);
			break;
			case (1):
			PORTA = step1;
			CUR_pos = 0;
			mTimer(delay);
			break;
			case (2):
			PORTA = step2;
			CUR_pos = 1;
			mTimer(delay);
			break;
			case (3):
			PORTA = step3;
			CUR_pos = 2;
			mTimer(delay);
			break;
			default:
			break;
		}
	
		if (i<6){
			delay = delay-2;
		} else if (i>steps-6){
			delay = delay+2;
		}
		
	}
}
/**************************************************************************************
* DESC: Show current counts of sorted materials and unsorted parts on LCD
*/
void showResults (){
	
	LCDClear();
	LCDWriteStringXY(0, 0, "BK");
	LCDWriteIntXY(0, 1, SORTED[BLACK[CODE]], 2);
	LCDWriteStringXY(3, 0, "ST");
	LCDWriteIntXY(3, 1, SORTED[STEEL[CODE]], 2);
	LCDWriteStringXY(6, 0, "WT");
	LCDWriteIntXY(6, 1, SORTED[WHITE[CODE]], 2);
	LCDWriteStringXY(9, 0, "AL");
	LCDWriteIntXY(9, 1, SORTED[ALUMINUM[CODE]], 2);
	if (RAMPDOWN==0){									// don't show unsorted parts after rampdown
		LCDWriteStringXY(12, 0, "US");
		LCDWriteIntXY(12, 1, size(&head, &tail), 2);	
	}
}
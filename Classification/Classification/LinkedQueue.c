/* Solution Set for the LinkedQueue.c */
/* 	
	Course		: UVic Mechatronics 458
	Milestone	: 3
	Title		: Data structures for MCUs and the Linked Queue Library

	Name 1:	Emiko Hourston				Student ID: V00914008
	Name 2:	Sophia Penner				Student ID: V00915678
	
	Description: This file uses pointers and timers to create a linked list, store polling data from switches,
				 remove the first data set (out of four datasets), and show the remaining three datasets on the
				 LEDs (PORTC).
*/

/* include libraries */
#include <stdlib.h>
#include <avr/io.h>
#include "LinkedQueue.h" 	/* This is the attached header file, which cleans things up */
							/* Make sure you read it!!! */
/* global variables */
/* Avoid using these */

void mTimer(int count);

/* main routine 
   You need to add the mtimer function to this project.    */

int main(){	

	link *head;			/* The ptr to the head of the queue */
	link *tail;			/* The ptr to the tail of the queue */
	link *newLink;		/* A ptr to a link aggregate data type (struct) */
	link *rtnLink;		/* same as the above */
	element eTest;		/* A variable to hold the aggregate data type known as element */

	DDRC = 0xFF; 		/* Used for debugging purposes only LEDs on PORTC */
	//DDRD = 0xFF;
	DDRA = 0x00;		/* Set PORTA to input*/

	CLKPR = 0x80;
	CLKPR = 0x01;		/* Required to set CPU Clock to 8MHz*/		

	TCCR1B |= _BV(CS11);		/* _BV sets the bit to logic 1*/

	rtnLink = NULL;
	newLink = NULL;

	while(1){
		setup(&head, &tail);

		PORTC = 0x00;	/* Sets all pins off */

		for(int i = 0; i < 4; i++){
		/* For loop stores 4 input datasets in linked list */
			
			while((PINA & 0x04) == 0x04){};		// While in the normal operation (have not yet pressed push button)
			mTimer(20);							// Debounce, 20ms software delay

			/* Initialize a new link here */
			initLink(&newLink);
			newLink->e.itemCode = PINA & 0x03;	// itemCode should be the input value
			enqueue(&head, &tail, &newLink);

			while((PINA & 0x04) == 0x00){};		// Button has been released
			mTimer(20);							// Debounce, 20ms software delay			
		}

		/* Remove the first item in queue and free the memory*/
		dequeue(&head, &rtnLink);	// Remove the item at the head of the list
		free(rtnLink);				// Free up memory

		for(int j = 0; j < 3; j++){
		/* For loop dequeues remaining 3 items from linked list and shows output on LEDs (PORTC) */
			dequeue(&head, &rtnLink);	// remove the item at the head of the list
			PORTC |= (rtnLink->e.itemCode << (2*j));		// Shift LEDs
			free(rtnLink);				// Free up memory

			mTimer(2000);				// 2 second delay between outputs
		}

		/* Reset period - either UI reset button or set timer */
		mTimer(5000);

	} // Closes while loop
	

	return(0);
}/* main */


/**************************************************************************************/
/***************************** SUBROUTINES ********************************************/
/**************************************************************************************/







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
void dequeue(link **h, link **deQueuedLink){
	/* ENTER YOUR CODE HERE */
	*deQueuedLink = *h;	// Will set to NULL if Head points to NULL
	/* Ensure it is not an empty queue */
	if (*h != NULL){
		*h = (*h)->next;
	}/*if*/
	
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

/* This is the driver for the timer. */
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
	TIMSK1 = TIMSK1 | 0b00000010;	// Enable the output compare interrupt enable
	TIMSK1 |=_BV(OCF1A);			// Clear the timer interrupt flag and begin new timing
	
	/* Poll the timer to determine when the timer has reached 0x03E8 */
	
	while(i<count){
		if((TIFR1 & 0x02) == 0x02){
			TIFR1 |= _BV(OCF1A);	// Clear interrupt flag by writing a ONE to the bit
			i++;					// Increment loop number
		}							// End if
	}								// End while
	return;							// End mTimer
}									// Close mTimer


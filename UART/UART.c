#include "MKL25Z4.h"                    // Device header

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22 //no need if dont need transmit
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128 //maybe should be 0?

#define Q_SIZE (32)

#define MASK(x) (1 << (x))

/* QUESTIONS 
1. how come when changing the volatile queues use pass by reference (&)
2. should we use unsigned char or uint8_t for the commands?
*/


/* Defining the queues*/
typedef struct{
		unsigned char Data[Q_SIZE];
		unsigned int Head; // points to oldest data element
		unsigned int Tail; // points to next free space
		unsigned int Size; // quantity of elements in queue
} Q_T;

/* Declaring the queues*/
volatile Q_T tx_q, rx_q; //no need transmit?

/* Initialising a queue*/
void Q_Init(volatile Q_T * q) {
	unsigned int i;
	for (i=0; i<Q_SIZE; i++)
		q->Data[i] = 0; // to simplify our lives when debugging
	q->Head = 0;
	q->Tail = 0;
	q->Size = 0;
}

/* Init UART2 */
void initUART2(uint32_t baud_rate)
{
	uint32_t divisor, bus_clock;
	
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	UART2->C2 |= UART_C2_TIE_MASK |
	UART_C2_RIE_MASK;
	
	UART2->C2 |= UART_C2_RIE_MASK;
	
	Q_Init(&tx_q); //no need?
	Q_Init(&rx_q);
}

/* To chevck if queue is empty*/
int Q_Empty(volatile Q_T * q) {
	return q->Size == 0;
}

/* To check if queue is full*/
int Q_Full(volatile Q_T * q) {
	return q->Size == Q_SIZE;
}

/* Delay Routine */
static void delay(volatile uint32_t nof) 
{
	while(nof != 0) {
		__asm("NOP");
		nof--;
	}
}

int Q_Enqueue(volatile Q_T * q, unsigned char d) {
	// What if queue is full?
	if (!Q_Full(q)) {
		q->Data[q->Tail++] = d;
		q->Tail %= Q_SIZE;
		q->Size++;
		return 1; // success
	} else {
		return 0; // failure
	}
}

unsigned char Q_Dequeue(volatile Q_T * q) {
	// Must check to see if queue is empty before dequeueing
	unsigned char t=0;
	if (!Q_Empty(q)) {
		t = q->Data[q->Head];
		q->Data[q->Head++] = 0; // to simplify debugging
		q->Head %= Q_SIZE;
		q->Size--;
	}
	return t;
}

void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2->S1 & UART_S1_TDRE_MASK) {
		// can send another character
		if (!Q_Empty(&tx_q)) {
			UART2->D = Q_Dequeue(&tx_q);
		} else {
			// queue is empty so disable tx
			UART2->C2 &= ~UART_C2_TIE_MASK;
		}
	}
	
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		// received a character
		if (!Q_Full(&rx_q)) {
			Q_Enqueue(&rx_q, UART2->D);
		} else {
			// error -queue full.
			//while (1);
		}
	}
}

/* MAIN function */
int main(void)
{
	uint8_t rx_data = 0x0;
	
	SystemCoreClockUpdate();
	initUART2(BAUD_RATE);
	
	while(1)
	{
		rx_data = Q_Dequeue(&rx_q); //get first data in the queue
		
		/* Parse/decode data received from phone app*/
		switch (rx_data) {
			case 1: break; //connected - play tone; blink led
			case 2: break; //move forward
			case 4: break; //move backward
			case 8: break; //move left
			case 16: break; //move right
			case 32: break; //stop moving
			case 64: break; //end challenge
			default: break; //error
		}
	}
}
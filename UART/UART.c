#include "MKL25Z4.h"                    // Device header

#define BAUD_RATE 9600
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 0 //maybe should be 0?

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1

#define Q_SIZE (32)

#define MASK(x) (1 << (x))

/* QUESTIONS 
1. how come when changing the volatile queues use pass by reference (&)
2. should we use unsigned char or uint8_t for the commands?
*/


 void InitGPIO(void)
{
  // Enable Clock to PORTB and PORTD
  SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
  
  // Configure MUX settings to make all 3 pins GPIO
  PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK; 
  PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1); 
  
  PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
  
  PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
  PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
  
  // Set Data Direction Registers for PortB and PortD
  PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
  PTD->PDDR |= MASK(BLUE_LED);
}

 void LED_FLASH_RED(void) {
  PTB->PCOR = MASK(RED_LED);
}

void LED_FLASH_GREEN(void) {
  PTB->PCOR = MASK(GREEN_LED);
}

void LED_FLASH_BLUE(void) {
  PTD->PCOR = MASK(BLUE_LED);
}

void LED_OFF(int colour) {
  switch (colour) {
    case 1: PTB->PSOR = MASK(RED_LED); break;
    case 2: PTB->PSOR = MASK(GREEN_LED); break;
    case 3: PTD->PSOR = MASK(BLUE_LED); break;
  }
}

/* Defining the queues*/
typedef struct{
		unsigned char Data[Q_SIZE];
		unsigned int Head; // points to oldest data element
		unsigned int Tail; // points to next free space
		unsigned int Size; // quantity of elements in queue
} Q_T;

/* Declaring the queues*/
volatile Q_T rx_q;

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
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	UART2->C2 &= ~(UART_C2_RE_MASK);
	
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	UART2->C2 |= UART_C2_RE_MASK;
	
	NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	UART2->C2 |= UART_C2_RIE_MASK;
	
	Q_Init(&rx_q);
}

/* To check if queue is empty*/
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
	uint8_t rx_data;
	
	SystemCoreClockUpdate();
	initUART2(BAUD_RATE);
  InitGPIO();
  LED_OFF(1); 
  LED_OFF(2);
  LED_OFF(3);
	
	while(1)
	{
		rx_data = Q_Dequeue(&rx_q); //get first data in the queue
		
		/* Parse/decode data received from phone app*/
		switch (rx_data) {
			case 1: break; //connected - play tone; blink led
			case 2: LED_FLASH_RED();
				delay(0x80000);
				LED_OFF(1);
				delay(0x80000); 
				break; //move forward
			case 4: LED_FLASH_GREEN();
				delay(0x80000);
				LED_OFF(2);
				delay(0x80000);break; //move backward
			case 8: LED_FLASH_BLUE();
				delay(0x80000);
				LED_OFF(3);
				delay(0x80000); break; //move left
			case 16: LED_FLASH_RED();
				delay(0x80000);
				LED_OFF(1);
				delay(0x80000); break; //move right
			case 32: LED_FLASH_GREEN();
				delay(0x80000);
				LED_OFF(2);
				delay(0x80000);break; //stop moving
			case 64: LED_FLASH_BLUE();
				delay(0x80000);
				LED_OFF(3);
				delay(0x80000); break; //end challenge
			default: break; //error
		}
	}
}
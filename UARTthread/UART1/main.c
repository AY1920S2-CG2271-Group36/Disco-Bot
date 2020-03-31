/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define BAUD_RATE 9600
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 0 //maybe should be 0?

#define Q_SIZE (32)

#define MASK(x) (1 << (x))

// LED
#define PTE4_Pin 4
#define PTB2_Pin 2
#define PTB3_Pin 3
#define PTE5_Pin 5
#define PTE20_Pin 20
#define PTE21_Pin 21
#define PTE22_Pin 22
#define PTE29_Pin 29
#define PTE30_Pin 30

// BUZZER
#define PTB0_Pin 0
#define Cnote 1046
#define Dnote 1175
#define Enote 1319
#define Fnote 1397
#define Gnote 1568
#define Anote 1760
#define Bbnote 1864

#define Rnote 0
#define CLK_FREQ 48000000
#define PRESCALER 128
#define DUTY_CYCLE 0.5
#define COUNT_THRES 65000

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
volatile Q_T rx_q;

/* Creating threads*/
osThreadId_t green_led_connect_id;
osThreadId_t green_led_moving_id;
osThreadId_t green_led_stationary_id;
osThreadId_t red_led_moving_id;
osThreadId_t red_led_stationary_id;
osThreadId_t generate_melody_id;

void initGPIO(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// LED
	PORTE->PCR[PTE4_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE4_Pin] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[PTE5_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE5_Pin] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[PTE20_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE20_Pin] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[PTE21_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE21_Pin] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[PTE22_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE22_Pin] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[PTE29_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE29_Pin] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[PTE30_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE30_Pin] |= PORT_PCR_MUX(1);
	
	// BUZZER
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	//initial mod pos
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Set Data Direction Registers for PortB
	PTE->PDDR |= (MASK(PTE4_Pin));
	PTB->PDDR |= (MASK(PTB2_Pin));
	PTB->PDDR |= (MASK(PTB3_Pin));
	PTE->PDDR |= (MASK(PTE5_Pin));
	PTE->PDDR |= (MASK(PTE20_Pin));
	PTE->PDDR |= (MASK(PTE21_Pin));
	PTE->PDDR |= (MASK(PTE22_Pin));
	PTE->PDDR |= (MASK(PTE29_Pin));
	PTE->PDDR |= (MASK(PTE30_Pin));
}


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

// LED
void ALL_LED_OFF(void)
{
	PTE->PCOR = MASK(PTE4_Pin);
}

void RED_LED_ON_500(void)
{
	PTE->PSOR = MASK(PTE4_Pin);
	osDelay(500);
}

void RED_LED_OFF_500(void)
{
	PTE->PCOR = MASK(PTE4_Pin);
	osDelay(500);
}

void RED_LED_ON_250(void)
{
	PTE->PSOR = MASK(PTE4_Pin);
	osDelay(250);
}

void RED_LED_OFF_250(void)
{
	PTE->PCOR = MASK(PTE4_Pin);
	osDelay(250);
}

// BUZZER
int calcModValue(int freq_value)
{
  int timer_clock_freq = CLK_FREQ / PRESCALER;
	int mod_value = timer_clock_freq / freq_value - 1;
	return mod_value;
}

void generateSignal(int freq) 
{
	TPM1->MOD = calcModValue(freq);
	TPM1_C0V = (TPM1->MOD) * DUTY_CYCLE;
}

void generateRest(void) 
{
	TPM1_C0V = 0;
}

void generateFullNote(int freq) 
{
	generateSignal(freq);
	osDelay(300);
	generateRest();
	osDelay(300);
}

void generateHalfNote(int freq)
{
	generateSignal(freq);
	osDelay(150);
	generateRest();
	osDelay(150);
}

 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void UART_DECODE(void *argument) {
	 uint8_t rx_data;
	for(;;)
	{
		rx_data = Q_Dequeue(&rx_q); //get first data in the queue
		//ALL_LED_OFF();
		
		/* Parse/decode data received from phone app*/
		switch (rx_data) {
			case 1: 
				break; //connected - play tone; blink led
			case 2: 
				break; //move forward
			case 4: 
				break; //move backward
			case 8: 
				break; //move left
			case 16: 
				break; //move right
			case 32: 
				break; //stop moving
			case 64: 
				break; //end challenge
			default: break; //error
		}
	}
}

void GREEN_LED_CONNECT(void *argument) {
	PTB->PSOR = MASK(PTB2_Pin);
	PTB->PSOR = MASK(PTB3_Pin);
}

void GREEN_LED_Stationary(void *argument)
{
	// ...
	for(;;) {
	PTB->PSOR = MASK(PTB2_Pin);
	PTB->PSOR = MASK(PTB3_Pin);
	PTE->PSOR = MASK(PTE5_Pin);
	PTE->PSOR = MASK(PTE20_Pin);
	PTE->PSOR = MASK(PTE21_Pin);
	PTE->PSOR = MASK(PTE22_Pin);
	PTE->PSOR = MASK(PTE29_Pin);
	PTE->PSOR = MASK(PTE30_Pin);
	}
}

void GREEN_LED_Moving (void *argument) {
 
  // ...
  for (;;) {
		PTB->PSOR = MASK(PTB2_Pin);
		osDelay(500);
		PTB->PCOR = MASK(PTB2_Pin);
		osDelay(500);
		PTB->PSOR = MASK(PTB3_Pin);
		osDelay(500);
		PTB->PCOR = MASK(PTB3_Pin);
		osDelay(500);
		PTE->PSOR = MASK(PTE5_Pin);
		osDelay(500);
		PTE->PCOR = MASK(PTE5_Pin);
		osDelay(500);
		PTE->PSOR = MASK(PTE20_Pin);
		osDelay(500);
		PTE->PCOR = MASK(PTE20_Pin);
		osDelay(500);
		PTE->PSOR = MASK(PTE21_Pin);
		osDelay(500);
		PTE->PCOR = MASK(PTE21_Pin);
		osDelay(500);
		PTE->PSOR = MASK(PTE22_Pin);
		osDelay(500);
		PTE->PCOR = MASK(PTE22_Pin);
		osDelay(500);
		PTE->PSOR = MASK(PTE29_Pin);
		osDelay(500);
		PTE->PCOR = MASK(PTE29_Pin);
		osDelay(500);
		PTE->PSOR = MASK(PTE30_Pin);
		osDelay(500);
		PTE->PCOR = MASK(PTE30_Pin);
		osDelay(500);
	}
}

void RED_LED_Moving (void *argument) {
 
  // ...
  for (;;) {
		RED_LED_ON_500();
		RED_LED_OFF_500();
	}
}

void RED_LED_Stationary (void *argument) {
 
  // ...
  for (;;) {
		RED_LED_ON_250();
		RED_LED_OFF_250();
	}
}

void generate_melody (void *argument) {
			while(1)
			{
				generateFullNote(Cnote);
				generateHalfNote(Cnote);
				generateFullNote(Dnote);
				generateFullNote(Cnote);
				generateFullNote(Fnote);
				generateFullNote(Enote);
				generateFullNote(Cnote);
				generateHalfNote(Cnote);
				generateFullNote(Dnote);
				generateFullNote(Cnote);
				generateFullNote(Gnote);
				generateFullNote(Fnote);
				generateFullNote(Cnote);
				generateHalfNote(Cnote);
				generateFullNote(Anote);
				generateFullNote(Fnote);
				generateFullNote(Enote);
				generateFullNote(Dnote);
				generateFullNote(Bbnote);
				generateHalfNote(Bbnote);
				generateFullNote(Anote);
				generateFullNote(Fnote);
				generateFullNote(Gnote);
				generateFullNote(Fnote);
			}	
}

 
int main (void) {
	
	SystemCoreClockUpdate();
	initGPIO();
	initUART2(BAUD_RATE);
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  green_led_connect_id = osThreadNew(GREEN_LED_CONNECT, NULL, NULL);    // Create application main thread
	green_led_moving_id = osThreadNew(GREEN_LED_Moving, NULL, NULL);
	green_led_stationary_id = osThreadNew(GREEN_LED_Stationary, NULL, NULL);
	red_led_moving_id = osThreadNew(RED_LED_Moving, NULL, NULL);
	red_led_stationary_id = osThreadNew(RED_LED_Stationary, NULL, NULL);
	generate_melody_id = osThreadNew(generate_melody, NULL, NULL);
	osKernelStart();                      // Start thread execution
  for (;;) {}
}

/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h" // Device header

#define BAUD_RATE 9600
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 0 

#define MSG_COUNT 1
#define Q_SIZE (32)
#define MASK(x) (1 << (x))


// LED
#define PTE4_Pin 4 //red led
#define PTB2_Pin 2
#define PTB3_Pin 3
#define PTE5_Pin 5
#define PTE20_Pin 20
#define PTE21_Pin 21
#define PTE22_Pin 22
#define PTE29_Pin 29
#define PTE30_Pin 30

#define GREEN_LED_RUN_DELAY 200

// BUZZER
#define PTB0_Pin 0

#define G0note 784
#define A0note 880
#define Bb0note 932
#define B0note 988

#define Cnote 1046
#define Dnote 1175
#define Enote 1319
#define Fnote 1397
#define Gnote 1568
#define Abnote 1661
#define Anote 1760
#define Bbnote 1864
#define Bnote 1975
#define C1note 2093
#define C2note 4186
#define D2note 4699
#define E2note 5274
#define F2note 5588

#define Rnote 0
#define CLK_FREQ 48000000
#define PRESCALER 128
#define DUTY_CYCLE 0.5
#define COUNT_THRES 65000



//MOTOR START--------------------------------------------------------------
#define PWM_FREQ 50
#define CLK_FREQ 48000000
#define PRESCALER 128
#define FORWARD_DUTY_CYCLE 0.6
#define CURVE_DUTY_CYCLE 0.1

// TPM0_CH0 (Front Left)
#define PTD0_Pin 0 												// AIN1
#define PTC1_Pin 1												// AIN2

// TPM0_CH5 (Front Right)
#define PTD5_Pin 5												// BIN1
#define PTC9_Pin 9												// BIN2

// TPM0_CH2 (Rear Left)
#define PTD2_Pin 2												// AIN1
#define PTC3_Pin 3												// AIN2

// TPM0_CH3 (Rear Right)
#define PTD3_Pin 3												// BIN1
#define PTC4_Pin 4												// BIN2

//MOTOR END --------------------------------------------------------------------------------


volatile int green_led_moving = 0;


/* Creating thread ids*/
osThreadId_t green_led_connect_id;
osThreadId_t green_led_moving_id;
osThreadId_t green_led_stationary_id;
osThreadId_t red_led_moving_id;
osThreadId_t red_led_stationary_id;
osThreadId_t generate_melody_id;
osThreadId_t end_challenge_id;

osThreadId_t UART_id;

/* Creating message queue ids*/
osMessageQueueId_t redMovingMsg;
osMessageQueueId_t redStationaryMsg;
osMessageQueueId_t greenMovingMsg;
osMessageQueueId_t greenStationaryMsg;

osMessageQueueId_t moveMsg;


/* Defining the queues*/
typedef struct{
		unsigned char Data[Q_SIZE];
		unsigned int Head; // points to oldest data element
		unsigned int Tail; // points to next free space
		unsigned int Size; // quantity of elements in queue
} Q_T;

/* Declaring the queues*/
volatile Q_T rx_q;

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

/* intiPWM() */
void initPWM(void) {
	
	// Enable clock to Port C
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	// Enable clock to Port D
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	// Configure the multiplexer values to select the PWM module
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD5_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD5_Pin] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD2_Pin] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD3_Pin] |= PORT_PCR_MUX(4);
	
	// Enable clock and power source to TPM0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); // Prescalar 128
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Edge-aligned PWM mode with high-true pulses
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C5SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C5SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

//--------------------------------------------------UART------------------------------------------------------------------

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
			//set flag for UART thread to run
			osThreadFlagsSet(UART_id, 0x0001); 
		} else {
			// error: queue full
		}
	}
}

// -------------------------------------------------------LED-------------------------------------------------------
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

void ON_ALL_GREEN_LEDS(void)
{
	PTB->PSOR = MASK(PTB2_Pin);
	PTB->PSOR = MASK(PTB3_Pin);
	PTE->PSOR = MASK(PTE5_Pin);
	PTE->PSOR = MASK(PTE20_Pin);
	PTE->PSOR = MASK(PTE21_Pin);
	PTE->PSOR = MASK(PTE22_Pin);
	PTE->PSOR = MASK(PTE29_Pin);
	PTE->PSOR = MASK(PTE30_Pin);
}

void OFF_ALL_GREEN_LEDS(void)
{
	PTB->PCOR = MASK(PTB2_Pin);
	PTB->PCOR = MASK(PTB3_Pin);
	PTE->PCOR = MASK(PTE5_Pin);
	PTE->PCOR = MASK(PTE20_Pin);
	PTE->PCOR = MASK(PTE21_Pin);
	PTE->PCOR = MASK(PTE22_Pin);
	PTE->PCOR = MASK(PTE29_Pin);
	PTE->PCOR = MASK(PTE30_Pin);
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

void generateTwoFullNote(int freq) 
{
  generateSignal(freq);
  osDelay(1900);
	generateRest();
	osDelay(40);
}

void generateFullNote(int freq) 
{
  generateSignal(freq);
  osDelay(900);
	generateRest();
	osDelay(40);
}

void generateHalfNote(int freq)
{
  generateSignal(freq);
  osDelay(450);
	generateRest();
	osDelay(40);
}

void generateQuarterNote(int freq)
{
  generateSignal(freq);
  osDelay(225);
	generateRest();
	osDelay(40);
}

void generateEighthNote(int freq)
{
  generateSignal(freq);
  osDelay(112);
	generateRest();
	osDelay(40);
}

void generateEndFullNote(int freq) 
{
  generateSignal(freq);
  delay(0x40000);
  generateRest();
  delay(0x40000);
}

void generateEndHalfNote(int freq) 
{
  generateSignal(freq);
  delay(0x20000);
  generateRest();
  delay(0x20000);
}

void generateEndQuarterNote(int freq) 
{
  generateSignal(freq);
  delay(0x10000);
  generateRest();
  delay(0x10000);
}

void generateEndTripleNote (int freq)
{
  generateSignal(freq);
  delay(0x120000);
  generateRest();
  delay(0x120000);
}


//MOTOR------------------------------------------------------------------
int startPWM(float freq) {
	int timer_clock_freq = CLK_FREQ / PRESCALER;
	int mod_value = timer_clock_freq / freq - 1;
	TPM0->MOD = mod_value;
	return mod_value;
}

// Spins front left wheel towards 'Front' direction
void forwardFL(float duty_cycle) {
	TPM0_C0V = (TPM0->MOD) * duty_cycle;
	
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(4); // Use the timer
	PTD->PDDR |= MASK(PTD0_Pin);
	
	PORTC->PCR[PTC1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC1_Pin] |= PORT_PCR_MUX(1); // Use GPIO
	PTC->PDDR |= MASK(PTC1_Pin); // Set to Low
	PTC->PCOR = MASK(PTC1_Pin);
}

// Spins front right wheel towards 'Front' direction
void forwardFR(float duty_cycle) {
	TPM0_C5V = (TPM0->MOD) * duty_cycle;
	
	PORTD->PCR[PTD5_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD5_Pin] |= PORT_PCR_MUX(4);
	PTD->PDDR |= MASK(PTD5_Pin);
	
	PORTC->PCR[PTC9_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC9_Pin] |= PORT_PCR_MUX(1);
	PTC->PDDR |= MASK(PTC9_Pin); 
	PTC->PCOR = MASK(PTC9_Pin);
}

// Spins rear left wheel towards 'Front' direction
void forwardRL(float duty_cycle) {
	TPM0_C2V = (TPM0->MOD) * duty_cycle;
	
	PORTD->PCR[PTD2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD2_Pin] |= PORT_PCR_MUX(4);
	PTD->PDDR |= MASK(PTD2_Pin);
	
	PORTC->PCR[PTC3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC3_Pin] |= PORT_PCR_MUX(1);
	PTC->PDDR |= MASK(PTC3_Pin); 
	PTC->PCOR = MASK(PTC3_Pin);
}

// Spins rear right wheel towards 'Front' direction
void forwardRR(float duty_cycle) {
	TPM0_C3V = (TPM0->MOD) * duty_cycle;
	
	PORTD->PCR[PTD3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD3_Pin] |= PORT_PCR_MUX(4);
	PTD->PDDR |= MASK(PTD3_Pin);
	
	PORTC->PCR[PTC4_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC4_Pin] |= PORT_PCR_MUX(1);
	PTC->PDDR |= MASK(PTC4_Pin); 
	PTC->PCOR = MASK(PTC4_Pin);
}

void reverseFL(float duty_cycle) {
	TPM0_C0V = (TPM0->MOD) * duty_cycle;
	
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(1);
	PTD->PDDR |= MASK(PTD0_Pin);
	PTD->PCOR = MASK(PTD0_Pin);
	
	PORTC->PCR[PTC1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC1_Pin] |= PORT_PCR_MUX(4);
	PTC->PDDR |= MASK(PTC1_Pin); 
}

void reverseFR(float duty_cycle) {
	TPM0_C5V = (TPM0->MOD) * duty_cycle;
	
	PORTD->PCR[PTD5_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD5_Pin] |= PORT_PCR_MUX(1);
	PTD->PDDR |= MASK(PTD5_Pin);
	PTD->PCOR = MASK(PTD5_Pin);
	
	PORTC->PCR[PTC9_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC9_Pin] |= PORT_PCR_MUX(3);
	PTC->PDDR |= MASK(PTC9_Pin); 
}

void reverseRL(float duty_cycle) {
	TPM0_C2V = (TPM0->MOD) * duty_cycle;
	
	PORTD->PCR[PTD2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD2_Pin] |= PORT_PCR_MUX(1);
	PTD->PDDR |= MASK(PTD2_Pin);
	PTD->PCOR = MASK(PTD2_Pin);
	
	PORTC->PCR[PTC3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC3_Pin] |= PORT_PCR_MUX(4);
	PTC->PDDR |= MASK(PTC3_Pin); 
}

void reverseRR(float duty_cycle) {
	TPM0_C3V = (TPM0->MOD) * duty_cycle;
	
	PORTD->PCR[PTD3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD3_Pin] |= PORT_PCR_MUX(1);
	PTD->PDDR |= MASK(PTD3_Pin);
	PTD->PCOR = MASK(PTD3_Pin);
	
	PORTC->PCR[PTC4_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC4_Pin] |= PORT_PCR_MUX(4);
	PTC->PDDR |= MASK(PTC4_Pin); 
}

void stopFL() {
	TPM0->MOD = 0;
	TPM0_C0V = 0;
	
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(1);
	PTD->PSOR = MASK(PTD0_Pin);
	
	PORTC->PCR[PTC1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC1_Pin] |= PORT_PCR_MUX(1);
	PTC->PSOR = MASK(PTC1_Pin);
}

void stopFR() {
	TPM0->MOD = 0;
	TPM0_C5V = 0;
	
	PORTD->PCR[PTD5_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD5_Pin] |= PORT_PCR_MUX(1);
	PTD->PSOR = MASK(PTD5_Pin);
	
	PORTC->PCR[PTC9_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC9_Pin] |= PORT_PCR_MUX(1);
	PTC->PSOR = MASK(PTC9_Pin);
}

void stopRL() {
	TPM0->MOD = 0;
	TPM0_C2V = 0;
	
	PORTD->PCR[PTD2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD2_Pin] |= PORT_PCR_MUX(1);
	PTD->PSOR = MASK(PTD2_Pin);
	
	PORTC->PCR[PTC3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC3_Pin] |= PORT_PCR_MUX(1);
	PTC->PSOR = MASK(PTC3_Pin);
}

void stopRR() {
	TPM0->MOD = 0;
	TPM0_C3V = 0;
	
	PORTD->PCR[PTD3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD3_Pin] |= PORT_PCR_MUX(1);
	PTD->PSOR = MASK(PTD3_Pin);
	
	PORTC->PCR[PTC4_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC4_Pin] |= PORT_PCR_MUX(1);
	PTC->PSOR = MASK(PTC4_Pin);
}

void moveForward() {
	startPWM(PWM_FREQ); // Not sure if actually necessary
	forwardFL(FORWARD_DUTY_CYCLE);
	forwardRL(FORWARD_DUTY_CYCLE);
	forwardFR(FORWARD_DUTY_CYCLE);
	forwardRR(FORWARD_DUTY_CYCLE);
}

void moveBackward() {
	startPWM(PWM_FREQ);
	reverseFL(FORWARD_DUTY_CYCLE);
	reverseRL(FORWARD_DUTY_CYCLE);
	reverseFR(FORWARD_DUTY_CYCLE);
	reverseRR(FORWARD_DUTY_CYCLE);
}

void turnLeft() {
	startPWM(PWM_FREQ);
	reverseFL(FORWARD_DUTY_CYCLE);
	forwardFR(FORWARD_DUTY_CYCLE);
	reverseRL(FORWARD_DUTY_CYCLE);
	forwardRR(FORWARD_DUTY_CYCLE);
}

void turnRight() {
	startPWM(PWM_FREQ);
	reverseFR(FORWARD_DUTY_CYCLE);
	forwardFL(FORWARD_DUTY_CYCLE);
	reverseRR(FORWARD_DUTY_CYCLE);
	forwardRL(FORWARD_DUTY_CYCLE);
}

void curveForwardLeft() {
	startPWM(PWM_FREQ);
	forwardFL(CURVE_DUTY_CYCLE);
	forwardRL(CURVE_DUTY_CYCLE);
	forwardFR(FORWARD_DUTY_CYCLE);
	forwardRR(FORWARD_DUTY_CYCLE);
}

void curveForwardRight() {
	startPWM(PWM_FREQ);
	forwardFL(FORWARD_DUTY_CYCLE);
	forwardRL(FORWARD_DUTY_CYCLE);
	forwardFR(CURVE_DUTY_CYCLE);
	forwardRR(CURVE_DUTY_CYCLE);
}

void curveBackwardLeft() {
	startPWM(PWM_FREQ);
	reverseFL(CURVE_DUTY_CYCLE);
	reverseRL(CURVE_DUTY_CYCLE);
	reverseFR(FORWARD_DUTY_CYCLE);
	reverseRR(FORWARD_DUTY_CYCLE);
}

void curveBackwardRight() {
	startPWM(PWM_FREQ);
	reverseFL(FORWARD_DUTY_CYCLE);
	reverseRL(FORWARD_DUTY_CYCLE);
	reverseFR(CURVE_DUTY_CYCLE);
	reverseRR(CURVE_DUTY_CYCLE);
}

void stopMovement() {
	//stopPWM();
	stopFL();
	stopFR();
	stopRL();
	stopRR();
}


/*----------------------------------------------------------------------------
 * Application  threads
 *---------------------------------------------------------------------------*/
void UART_decode(void *argument) {
	uint8_t rx_data;
	int moving, stationary, direction;
	for(;;)
	{
		osThreadFlagsWait(0x0001, osFlagsWaitAll, osWaitForever);
		rx_data = Q_Dequeue(&rx_q); //get first data in the queue
		
		/* Decode data received from phone app*/
		switch (rx_data) {
			//CONNECTED
			case 1: 
				osThreadFlagsSet(green_led_connect_id, 0x0001); //blink led twice
				osThreadFlagsSet(generate_melody_id, 0x0001); //play tone then background music
				break; 
			//FORWARD
			case 2: 
				moving = green_led_moving = 1;
				osMessageQueuePut(redMovingMsg, &moving, NULL, 0);
				osMessageQueuePut(greenMovingMsg, &moving, NULL, 0);
			
				stationary = 0;
				osMessageQueuePut(redStationaryMsg, &stationary, NULL, 0);
				osMessageQueuePut(greenStationaryMsg, &stationary, NULL, 0);
			
				direction = 1;
				osMessageQueuePut(moveMsg, &direction, NULL, 0);
				break;
			//BACKWARD
			case 4: 
				moving = green_led_moving = 1;
				osMessageQueuePut(redMovingMsg, &moving, NULL, 0);
				osMessageQueuePut(greenMovingMsg, &moving, NULL, 0);
			
				stationary = 0;
				osMessageQueuePut(redStationaryMsg, &stationary, NULL, 0);
				osMessageQueuePut(greenStationaryMsg, &stationary, NULL, 0);
			
				direction = 2;
				osMessageQueuePut(moveMsg, &direction, NULL, 0);
				break; 
			//LEFT
			case 8: 
				moving = green_led_moving = 1;
				osMessageQueuePut(redMovingMsg, &moving, NULL, 0);
				osMessageQueuePut(greenMovingMsg, &moving, NULL, 0);
			
				stationary = 0;
				osMessageQueuePut(redStationaryMsg, &stationary, NULL, 0);
				osMessageQueuePut(greenStationaryMsg, &stationary, NULL, 0);
				
				direction = 3;
				osMessageQueuePut(moveMsg, &direction, NULL, 0);
				break; 
			//RIGHT
			case 16: 
				moving = green_led_moving = 1;
				osMessageQueuePut(redMovingMsg, &moving, NULL, 0);
				osMessageQueuePut(greenMovingMsg, &moving, NULL, 0);
				
				stationary = 0;
				osMessageQueuePut(redStationaryMsg, &stationary, NULL, 0);
				osMessageQueuePut(greenStationaryMsg, &stationary, NULL, 0);
				
				direction = 4;
				osMessageQueuePut(moveMsg, &direction, NULL, 0);
				break; 
			//CURVE FORWARD LEFT
			case 20:
				moving = green_led_moving = 1;
				osMessageQueuePut(redMovingMsg, &moving, NULL, 0);
				osMessageQueuePut(greenMovingMsg, &moving, NULL, 0);
				
				stationary = 0;
				osMessageQueuePut(redStationaryMsg, &stationary, NULL, 0);
				osMessageQueuePut(greenStationaryMsg, &stationary, NULL, 0);
				
				direction = 5;
				osMessageQueuePut(moveMsg, &direction, NULL, 0);
				break;
			//CURVE FORWARD RIGHT
			case 21:
				moving = green_led_moving = 1;
				osMessageQueuePut(redMovingMsg, &moving, NULL, 0);
				osMessageQueuePut(greenMovingMsg, &moving, NULL, 0);
				
				stationary = 0;
				osMessageQueuePut(redStationaryMsg, &stationary, NULL, 0);
				osMessageQueuePut(greenStationaryMsg, &stationary, NULL, 0);
				
				direction = 6;
				osMessageQueuePut(moveMsg, &direction, NULL, 0);
				break;
			//CURVE BACKWARD LEFT
			case 22:
				moving = green_led_moving = 1;
				osMessageQueuePut(redMovingMsg, &moving, NULL, 0);
				osMessageQueuePut(greenMovingMsg, &moving, NULL, 0);
			
				stationary = 0;
				osMessageQueuePut(redStationaryMsg, &stationary, NULL, 0);
				osMessageQueuePut(greenStationaryMsg, &stationary, NULL, 0);
				
				direction = 7;
				osMessageQueuePut(moveMsg, &direction, NULL, 0);
				break;
			//CURVE BACKWARD RIGHT
			case 23:
				moving = green_led_moving = 1;
				osMessageQueuePut(redMovingMsg, &moving, NULL, 0);
				osMessageQueuePut(greenMovingMsg, &moving, NULL, 0);
				
			
				stationary = 0;
				osMessageQueuePut(redStationaryMsg, &stationary, NULL, 0);
				osMessageQueuePut(greenStationaryMsg, &stationary, NULL, 0);
				
				direction = 8;
				osMessageQueuePut(moveMsg, &direction, NULL, 0);
				break;
			//STOP
			case 32: 
				moving = green_led_moving = 0;
				osMessageQueuePut(redMovingMsg, &moving, NULL, 0);
				osMessageQueuePut(greenMovingMsg, &moving, NULL, 0);
				
				stationary = 1;
				osMessageQueuePut(redStationaryMsg, &stationary, NULL, 0);
				osMessageQueuePut(greenStationaryMsg, &stationary, NULL, 0);
		
				osMessageQueuePut(moveMsg, &moving, NULL, 0);
				break; 
			//END CHALLENGE
			case 64: 
				osThreadFlagsSet(end_challenge_id, 0x0001);
				break; 
			default: break; //error
		}
	}
}

//------------------------------------------------- led threads --------------------------------------------------------------
void GREEN_LED_CONNECT(void *argument) {
	for (;;) {
		osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
		for(int i = 0; i < 2; i++ ) {
		ON_ALL_GREEN_LEDS();
		osDelay(300);	
		OFF_ALL_GREEN_LEDS();
		osDelay(300);	
	}
	}
}

void GREEN_LED_Stationary(void *argument)
{
	int led_on = 0;
	for(;;) {
		osMessageQueueGet(greenStationaryMsg, &led_on, NULL, 0);
		if (led_on == 1) {
			ON_ALL_GREEN_LEDS();
		} else {
			OFF_ALL_GREEN_LEDS();
			osMessageQueueGet(greenStationaryMsg, &led_on, NULL, osWaitForever);
		}
	}
}

void GREEN_LED_Moving (void *argument) {
	int run = 0; 
  
  for (;;) {
		osMessageQueueGet(greenMovingMsg, &run, NULL, 0);
		if (run == 1) {
			PTB->PSOR = MASK(PTB2_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			PTB->PCOR = MASK(PTB2_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			if (green_led_moving == 0) {
				continue;
			}
			PTB->PSOR = MASK(PTB3_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			PTB->PCOR = MASK(PTB3_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			if (green_led_moving == 0) {
				continue;
			}
			PTE->PSOR = MASK(PTE5_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			PTE->PCOR = MASK(PTE5_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			if (green_led_moving == 0) {
				continue;
			}
			PTE->PSOR = MASK(PTE20_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			PTE->PCOR = MASK(PTE20_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			if (green_led_moving == 0) {
				continue;
			}
			PTE->PSOR = MASK(PTE21_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			PTE->PCOR = MASK(PTE21_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			if (green_led_moving == 0) {
				continue;
			}
			PTE->PSOR = MASK(PTE22_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			PTE->PCOR = MASK(PTE22_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			if (green_led_moving == 0) {
				continue;
			}
			PTE->PSOR = MASK(PTE29_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			PTE->PCOR = MASK(PTE29_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			if (green_led_moving == 0) {
				continue;
			}
			PTE->PSOR = MASK(PTE30_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
			PTE->PCOR = MASK(PTE30_Pin);
			osDelay(GREEN_LED_RUN_DELAY);
		} else {
			osMessageQueueGet(greenMovingMsg, &run, NULL, osWaitForever);
		}
	}
}

void RED_LED_Moving (void *argument) {
  int blink = 0;
  for (;;) {
		osMessageQueueGet(redMovingMsg, &blink, NULL, 0); //try to get message
		if (blink == 1) {
			RED_LED_ON_500();
			RED_LED_OFF_500();
		} else {
			osMessageQueueGet(redMovingMsg, &blink, NULL, osWaitForever);
		}
		
	}
}

void RED_LED_Stationary (void *argument) {
  int blink = 0;
  for (;;) {
		osMessageQueueGet(redStationaryMsg, &blink, NULL, 0); 
		if (blink == 1) {
			RED_LED_ON_250();
			RED_LED_OFF_250();
		} else {
			osMessageQueueGet(redStationaryMsg, &blink, NULL, osWaitForever);
		}
	}
}

void generate_melody (void *argument) {
	osThreadFlagsWait(0x0001, osFlagsWaitAll, osWaitForever);
	
	//connected tone sequence
	generateHalfNote(C2note);
	generateHalfNote(D2note);
	generateHalfNote(E2note);
	generateHalfNote(F2note);
	
	//on stationary leds
	int stationary = 1;
	osMessageQueuePut(redStationaryMsg, &stationary, NULL, 0);
	osMessageQueuePut(greenStationaryMsg, &stationary, NULL, 0);
	
	osDelay(1000);
	
	while(1)
			{
				generateFullNote(Fnote);
        generateQuarterNote(Enote);
        generateQuarterNote(Fnote);
        generateHalfNote(Enote);
        generateFullNote(Cnote);
        generateQuarterNote(A0note);
        generateHalfNote(Dnote);
        generateTwoFullNote(A0note);
        generateFullNote(Fnote);
        generateQuarterNote(Gnote);
        generateQuarterNote(Anote);
        generateHalfNote(C1note);
        generateFullNote(Anote);
        generateEighthNote(Dnote);
        generateEighthNote(Enote);
        generateFullNote(Dnote);
        generateFullNote(Dnote);
        generateHalfNote(Cnote);
        generateQuarterNote(Dnote);
        generateFullNote(Cnote);
        generateFullNote(Bb0note);
        generateFullNote(Bbnote);
        generateHalfNote(Anote);
        generateQuarterNote(Bbnote);
        generateFullNote(Anote);
        generateFullNote(Gnote);
        generateFullNote(Anote);
        generateFullNote(Fnote);
        generateQuarterNote(Bbnote);
        generateFullNote(Anote);
        generateHalfNote(Fnote);
        generateQuarterNote(Bbnote);
        generateFullNote(Abnote);
        generateHalfNote(Fnote);
        generateQuarterNote(Bbnote);
        generateFullNote(Abnote);
			}	
}

void end_challenge(void *argument)
{
	osThreadFlagsWait(0x0001, osFlagsWaitAll, osWaitForever);
	for (;;) {
		OFF_ALL_GREEN_LEDS();
		ALL_LED_OFF();
		delay(0x8000);
		generateEndHalfNote(C1note);
    generateEndHalfNote(C1note);
    generateEndHalfNote(C1note);
    generateEndFullNote(C1note);
    generateEndFullNote(Abnote);
    generateEndFullNote(Bbnote);
    generateEndFullNote(C1note);
    generateEndHalfNote(Bbnote);
    generateEndTripleNote(Cnote);
		
		while(1) {}
	}
}

//---------------------------------------------- motor threads -------------------------------------------------
void motor(void *argument) {
	int move = 0;
  for (;;) {
		osMessageQueueGet(moveMsg, &move, NULL, 0); 
		if (move == 1) {
			moveForward();
		} else if (move == 2) {
			moveBackward();
		} else if (move == 3) {
			turnLeft();
		} else if (move == 4) {
			turnRight();
		} else if (move == 5) {
			curveForwardLeft();
		} else if (move == 6) {
			curveForwardRight();
		} else if (move == 7) {
			curveBackwardLeft();
		} else if (move == 8) {
			curveBackwardRight();
		} else {
			stopMovement();
			osMessageQueueGet(moveMsg, &move, NULL, osWaitForever);
		}
	}
}

int main (void) {
	SystemCoreClockUpdate();	
	initGPIO();
	initUART2(BAUD_RATE);
	initPWM();
	
	//set priority of uart to be higher than normal
	const osThreadAttr_t uart_thread_attr = {
		.priority = osPriorityNormal2
	};
	
	//set priority of end_challenge to be higher than normal
	const osThreadAttr_t end_challenge_thread_attr = {
		.priority = osPriorityNormal2
	};

  // Initialize CMSIS-RTOS
  osKernelInitialize();                 
	
	//create threads
  green_led_connect_id = osThreadNew(GREEN_LED_CONNECT, NULL, NULL);    
	green_led_moving_id = osThreadNew(GREEN_LED_Moving, NULL, NULL);
	green_led_stationary_id = osThreadNew(GREEN_LED_Stationary, NULL, NULL);
	red_led_moving_id = osThreadNew(RED_LED_Moving, NULL, NULL);
	red_led_stationary_id = osThreadNew(RED_LED_Stationary, NULL, NULL);
	generate_melody_id = osThreadNew(generate_melody, NULL, NULL);
	UART_id = osThreadNew(UART_decode, NULL, &uart_thread_attr);
	end_challenge_id = osThreadNew(end_challenge, NULL, &end_challenge_thread_attr);
	osThreadNew(motor, NULL, NULL); 
	
	//create message queues
	redMovingMsg = osMessageQueueNew(MSG_COUNT, sizeof(int), NULL);
	redStationaryMsg = osMessageQueueNew(MSG_COUNT, sizeof(int), NULL);
	greenMovingMsg = osMessageQueueNew(MSG_COUNT, sizeof(int), NULL);
	greenStationaryMsg = osMessageQueueNew(MSG_COUNT, sizeof(int), NULL);
	
	moveMsg = osMessageQueueNew(MSG_COUNT, sizeof(int), NULL);
	
	// Start thread execution
	osKernelStart(); 
  for (;;) {}
	
}

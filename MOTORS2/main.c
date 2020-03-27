/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
/*----------------------------------------------------------------------------
 * PWM Driver
 *---------------------------------------------------------------------------*/

#include "MKL25Z4.h"                    // Device header
//#include "math.h"
#define MASK(x) 			(1 << (x))
#define DELAYCOUNT    100000
#define PERIOD				0.02
#define CORE_PERIOD		(2.67/1000000)  // Prescalar 128

#define PTB0_Pin 0
#define PTB1_Pin 1

// TPM0_CH0 (Front Left)
#define PTD0_Pin 0 												// AIN1
#define PTC1_Pin 1												// AIN2

// TPM0_CH1 (Front Right)
#define PTD1_Pin 1												// BIN1
#define PTC2_Pin 2												// BIN2

// TPM0_CH2 (Rear Left)
#define PTD2_Pin 2												// AIN1
#define PTC3_Pin 3												// AIN2

// TPM0_CH3 (Rear Right)
#define PTD3_Pin 3												// BIN1
#define PTC4_Pin 4												// BIN2

/*
#define PTD0_Pin 6 												// Left Motors
#define PTD2_Pin 8 												// Right Motors

#define PTC12_Pin 1 											// L
#define PTC13_Pin 3
#define PTC16_Pin 5
#define PTC17_Pin 7
*/

#define FORWARD_DUTY_CYCLE 0.8
#define NO_CUTY_CYCLE 0

//bool front = false;

/* intiPWM() */
void initPWM(void) {
	
	// Enable clock to Port C
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	// Enable clock to Port D
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// Configure the multiplexer values to select the PWM module
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(4);
	
	//PORTC->PCR[PTC1_Pin] &= ~PORT_PCR_MUX_MASK;
	//PORTC->PCR[PTC1_Pin] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD1_Pin] |= PORT_PCR_MUX(4);

	//PORTC->PCR[PTC2_Pin] &= ~PORT_PCR_MUX_MASK;
	//PORTC->PCR[PTC2_Pin] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD2_Pin] |= PORT_PCR_MUX(4);
	
	//PORTC->PCR[PTC3_Pin] &= ~PORT_PCR_MUX_MASK;
	//PORTC->PCR[PTC3_Pin] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD3_Pin] |= PORT_PCR_MUX(4);
	
	//PORTC->PCR[PTC4_Pin] &= ~PORT_PCR_MUX_MASK;
	//PORTC->PCR[PTC4_Pin] |= PORT_PCR_MUX(4);
	
	// Enable clock and power source to TPM0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	// Select MCGFLLCLK for TPM counter clock
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); // Prescalar 128
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Edge-aligned PWM mode with high-true pulses
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
}

void forwardFL(float dutyCycle) {
	int numClkCycles = PERIOD / CORE_PERIOD;
	TPM0_C0V = numClkCycles * dutyCycle;
	
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(4);
	
	PORTC->PCR[PTC1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC1_Pin] |= PORT_PCR_MUX(1);
	PTC->PCOR = MASK(PTC1_Pin);
}

void forwardFR(float dutyCycle) {
	int numClkCycles = PERIOD / CORE_PERIOD;
	TPM0_C1V = numClkCycles * dutyCycle;
	
	PORTD->PCR[PTD1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD1_Pin] |= PORT_PCR_MUX(4);
	
	PORTC->PCR[PTC2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC2_Pin] |= PORT_PCR_MUX(1);
	PTC->PCOR = MASK(PTC2_Pin);
}

void forwardRL(float dutyCycle) {
	int numClkCycles = PERIOD / CORE_PERIOD;
	TPM0_C2V = numClkCycles * dutyCycle;
	
	PORTD->PCR[PTD2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD2_Pin] |= PORT_PCR_MUX(4);
	
	PORTC->PCR[PTC3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC3_Pin] |= PORT_PCR_MUX(1);
	PTC->PCOR = MASK(PTC3_Pin);
}

void forwardRR(float dutyCycle) {
	int numClkCycles = PERIOD / CORE_PERIOD;
	TPM0_C3V = numClkCycles * dutyCycle;
	
	PORTD->PCR[PTD3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD3_Pin] |= PORT_PCR_MUX(4);
	
	PORTC->PCR[PTC4_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC4_Pin] |= PORT_PCR_MUX(1);
	PTC->PCOR = MASK(PTC4_Pin);
}

void stopFL() {
	TPM0_C0V = 0;
	
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(1);
	PTD->PSOR = MASK(PTD0_Pin);
	
	PORTC->PCR[PTC1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC1_Pin] |= PORT_PCR_MUX(1);
	PTC->PSOR = MASK(PTC1_Pin);
}

void stopFR() {
	TPM0_C1V = 0;
	
	PORTD->PCR[PTD1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD1_Pin] |= PORT_PCR_MUX(1);
	PTD->PSOR = MASK(PTD1_Pin);
	
	PORTC->PCR[PTC2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC2_Pin] |= PORT_PCR_MUX(1);
	PTC->PSOR = MASK(PTC2_Pin);
}

void stopRL() {
	TPM0_C2V = 0;
	
	PORTD->PCR[PTD2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD2_Pin] |= PORT_PCR_MUX(1);
	PTD->PSOR = MASK(PTD2_Pin);
	
	PORTC->PCR[PTC3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC3_Pin] |= PORT_PCR_MUX(1);
	PTC->PSOR = MASK(PTC3_Pin);
}

void stopRR() {
	TPM0_C3V = 0;
	
	PORTD->PCR[PTD2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD2_Pin] |= PORT_PCR_MUX(1);
	PTD->PSOR = MASK(PTD2_Pin);
	
	PORTC->PCR[PTC3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC3_Pin] |= PORT_PCR_MUX(1);
	PTC->PSOR = MASK(PTC3_Pin);
}

void startPWM() {
	int numClkCycles = PERIOD / CORE_PERIOD;
	TPM0->MOD = numClkCycles - 1;
}

void moveForward() {
	startPWM();
	forwardFL(FORWARD_DUTY_CYCLE);
	forwardRL(FORWARD_DUTY_CYCLE);
	forwardFR(FORWARD_DUTY_CYCLE);
	forwardRR(FORWARD_DUTY_CYCLE);
}

void stopMovement() {
	TPM0->MOD = 0;
	stopFL();
	stopFR();
	stopRL();
	stopRR();
}
 

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main (void *argument) {
 
  // ...
  for (;;) {
		moveForward();
		osDelay(5000);
		stopMovement();
		osDelay(5000);
	}
}
 
int main (void) {
 
	initPWM();
	
  // System Initialization
  SystemCoreClockUpdate();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);    // Create application main thread
  osKernelStart(); 											// Start thread execution
  for (;;) {}
}
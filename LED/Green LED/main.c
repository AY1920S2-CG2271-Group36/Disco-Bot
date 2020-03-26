/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define PTB1_Pin 1
#define PTB2_Pin 2
#define PTB3_Pin 3
#define PTC2_Pin 2
#define PTC1_Pin 1
#define PTE20_Pin 20
#define PTE21_Pin 21
#define PTE29_Pin 29
#define MASK(x) (1 << (x))
 
void initGPIO(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[PTC2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC2_Pin] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[PTC1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC1_Pin] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[PTE20_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE20_Pin] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[PTE21_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE21_Pin] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[PTE29_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE29_Pin] |= PORT_PCR_MUX(1);
	
	// Set Data Direction Registers for PortB
	PTB->PDDR |= (MASK(PTB1_Pin));
	PTB->PDDR |= (MASK(PTB2_Pin));
	PTB->PDDR |= (MASK(PTB3_Pin));
	PTC->PDDR |= (MASK(PTC2_Pin));
	PTC->PDDR |= (MASK(PTC1_Pin));
	PTE->PDDR |= (MASK(PTE20_Pin));
	PTE->PDDR |= (MASK(PTE21_Pin));
	PTE->PDDR |= (MASK(PTE29_Pin));
}

void GREEN_LED_FOREVER(void)
{
	PTB->PSOR = MASK(PTB1_Pin);
	PTB->PSOR = MASK(PTB2_Pin);
	PTB->PSOR = MASK(PTB3_Pin);
	PTC->PSOR = MASK(PTC2_Pin);
	PTC->PSOR = MASK(PTC1_Pin);
	PTE->PSOR = MASK(PTE20_Pin);
	PTE->PSOR = MASK(PTE21_Pin);
	PTE->PSOR = MASK(PTE29_Pin);
}

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void GREEN_LED_Moving (void *argument) {
 
  // ...
  for (;;) {
		PTB->PSOR = MASK(PTB1_Pin);
		osDelay(500);
		PTB->PCOR = MASK(PTB1_Pin);
		osDelay(500);
		PTB->PSOR = MASK(PTB2_Pin);
		osDelay(500);
		PTB->PCOR = MASK(PTB2_Pin);
		osDelay(500);
		PTB->PSOR = MASK(PTB3_Pin);
		osDelay(500);
		PTB->PCOR = MASK(PTB3_Pin);
		osDelay(500);
		PTC->PSOR = MASK(PTC2_Pin);
		osDelay(500);
		PTC->PCOR = MASK(PTC2_Pin);
		osDelay(500);
		PTC->PSOR = MASK(PTC1_Pin);
		osDelay(500);
		PTC->PCOR = MASK(PTC1_Pin);
		osDelay(500);
		PTE->PSOR = MASK(PTE20_Pin);
		osDelay(500);
		PTE->PCOR = MASK(PTE20_Pin);
		osDelay(500);
		PTE->PSOR = MASK(PTE21_Pin);
		osDelay(500);
		PTE->PCOR = MASK(PTE21_Pin);
		osDelay(500);
		PTE->PSOR = MASK(PTE29_Pin);
		osDelay(500);
		PTE->PCOR = MASK(PTE29_Pin);
		osDelay(500);
	}
}

/*void GREEN_LED_Stationary (void *argument) {
	
	// ...
	for (;;) {
		GREEN_LED_FOREVER();
	}
}*/
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	initGPIO();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(GREEN_LED_Moving, NULL, NULL);    // Create application main thread
  //osThreadNew(GREEN_LED_Stationary, NULL, NULL);    // Create application main thread
	osKernelStart();                      // Start thread execution
  for (;;) {}
}

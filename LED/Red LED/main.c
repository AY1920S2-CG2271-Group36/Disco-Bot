/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define PTB0_Pin 0
#define MASK(x) (1 << (x))

void initGPIO(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(1);
	
	// Set Data Direction Registers for PortB
	PTB->PDDR |= (MASK(PTB0_Pin));

}

void RED_LED_ON_500(void)
{
	PTB->PSOR = MASK(PTB0_Pin);
	osDelay(500);
}

void RED_LED_OFF_500(void)
{
	PTB->PCOR = MASK(PTB0_Pin);
	osDelay(500);
}

void RED_LED_ON_250(void)
{
	PTB->PSOR = MASK(PTB0_Pin);
	osDelay(250);
}

void RED_LED_OFF_250(void)
{
	PTB->PCOR = MASK(PTB0_Pin);
	osDelay(250);
}

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void RED_LED_Moving (void *argument) {
 
  // ...
  for (;;) {
		RED_LED_ON_500();
		RED_LED_OFF_500();
	}
}

/*void RED_LED_Stationary (void *argument) {
 
  // ...
  for (;;) {
		RED_LED_ON_250();
		RED_LED_OFF_250();
	}
}*/
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	initGPIO();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(RED_LED_Moving, NULL, NULL); // Create application main thread
	//osThreadNew(RED_LED_Stationary, NULL, NULL); // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}

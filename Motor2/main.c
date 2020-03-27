/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"                    // Device header


#define MASK(x) (1 << (x))

#define PWM_FREQ 50
#define CLK_FREQ 48000000
#define PRESCALER 128
#define DUTY_CYCLE 0.5

// TPM0_CH0 (Front Left)
#define PTD0_Pin 0 // AIN1
#define PTC1_Pin 1 // AIN2

#define PORT_C 3
#define PORT_D 4

int calcModValue(int freq_value)
{
  int timer_clock_freq = CLK_FREQ / PRESCALER;
	int mod_value = timer_clock_freq / freq_value - 1;
	return mod_value;
}

/* initPWM() */
void initPWM(void)
{
	//enable clock to port C and port D
	SIM->SCGC5 |= ((SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTD_MASK));
	
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(4);
	
	PORTC->PCR[PTC1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC1_Pin] |= PORT_PCR_MUX(4);
	
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	//initial mod pos
	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void generateSignal(int freq) 
{
	TPM0->MOD = calcModValue(freq);
	TPM0_C0V = (TPM0->MOD) * DUTY_CYCLE;
}

void make_zero(int pin_num, int port) {
	if (port == PORT_D) { 
		//configure pin as GPIO
		PORTD->PCR[pin_num] &= ~PORT_PCR_MUX_MASK; 
		PORTD->PCR[pin_num] |= PORT_PCR_MUX(1); 
		PTD->PDDR |= MASK(pin_num); //set pin to output
		PTD->PCOR = MASK(pin_num); //output low signal on pin
	} else if (port == PORT_C) { 
		//configure pin as GPIO
		PORTC->PCR[pin_num] &= ~PORT_PCR_MUX_MASK; 
		PORTC->PCR[pin_num] |= PORT_PCR_MUX(1); 
		PTC->PDDR |= MASK(pin_num); //set pin to output
		PTC->PCOR = MASK(pin_num); //output low signal on pin
	}
}

 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main (void *argument) {
 
  for (;;) {
		//move forward
		make_zero(PTD0_Pin, PORT_D);
		generateSignal(PWM_FREQ);
		osDelay(2000);
	}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
  // ...
  initPWM();
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}

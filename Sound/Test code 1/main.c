/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"                    // Device header

#define PTB0_Pin 0
//#define PTB1_Pin 1

#define C 262
#define D 294
#define E 330
#define F 349
#define R 0
#define CLK_FREQ 48000000
#define PRESCALER 128
#define DUTY_CYCLE 0.5
#define COUNT_THRES 65000
//#define MASK(x) (1 << (x))

volatile unsigned int count = 0;

int calcModValue(int freq_value)
{
  int timer_clock_freq = CLK_FREQ / PRESCALER;
	int mod_value = (timer_clock_freq / freq_value) - 1;
	return mod_value;
}

/* initPWM() */
void initPWM(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	
	//PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	//PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	//initial mod pos
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void generateSignal(int freq) 
{
	TPM1->MOD = calcModValue(freq);
	TPM1_C0V = (TPM1->MOD) * DUTY_CYCLE;
}
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main (void *argument) {
 
	int melody [] = {C, R, R, C, R, R, D, R, R, C, R, R, F, R, R, E};
	int MAX_COUNT = sizeof(melody);
		
  // ...
  for (;;) 
	{
		int i = 0;
			while(1)
			{
				count ++;
				if(count > COUNT_THRES) {
					int tone = melody[i++];
					generateSignal(tone);
					count = 0;
				}
				if(i >= MAX_COUNT) {
					i = 0;
				}
			}	
	}
}
 
int main (void) 
{
 
  // System Initialization
  SystemCoreClockUpdate();
	initPWM();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {
	}
}
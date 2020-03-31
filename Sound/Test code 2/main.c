/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.h"                    // Device header

#define PTB0_Pin 0

/*#define Cnote 262
#define Dnote 294
#define Enote 330
#define Fnote 349
#define Gnote 392
#define Anote 440
#define Bnote 494*/

/*#define Cnote 523
#define Dnote 587
#define Enote 659
#define Fnote 698
#define Gnote 783
#define Anote 880
#define Bbnote 932*/

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

int calcModValue(int freq_value)
{
  int timer_clock_freq = CLK_FREQ / PRESCALER;
	int mod_value = timer_clock_freq / freq_value - 1;
	return mod_value;
}

/* initPWM() */
void initPWM(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
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
 
  // System Initialization
  SystemCoreClockUpdate();
	initPWM();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(generate_melody, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}

/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.h"                    // Device header

#define PTB0_Pin 0

#define Cnote 1046
#define Dnote 1175
#define Enote 1319
#define Fnote 1397
#define Gnote 1568
#define Anote 1760
#define Bbnote 1864

#define C1note 2093
#define D1note 2349
#define E1note 2637
#define F1note 2793
#define G1note 3136
#define Ab1note 3322
#define A1note 3520
#define Bb1note 3729
#define B1note 3951
#define C2note 4186

#define CLK_FREQ 48000000
#define PRESCALER 128
#define DUTY_CYCLE 0.5

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

void generateQuarterNote(int freq)
{
	generateSignal(freq);
	osDelay(75);
	generateRest();
	osDelay(75);
}

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void generate_end_melody (void *argument) {
			while(1)
			{
				generateFullNote(F1note);
				generateQuarterNote(E1note);
				generateQuarterNote(F1note);
				generateHalfNote(E1note);
				generateFullNote(C1note);
				generateQuarterNote(Anote);
				generateHalfNote(D1note);
				generateQuarterNote(C1note);
				generateFullNote(F1note);
				generateQuarterNote(G1note);
				generateQuarterNote(A1note);
				generateHalfNote(C2note);
				generateFullNote(A1note);
				generateQuarterNote(D1note);
				generateQuarterNote(E1note);
				generateFullNote(D1note);
				generateHalfNote(D1note);
				generateHalfNote(C1note);
				generateQuarterNote(D1note);
				generateHalfNote(C1note);
				generateHalfNote(Bbnote);
				generateHalfNote(Bb1note);
				generateHalfNote(A1note);
				generateQuarterNote(Bb1note);
				generateHalfNote(A1note);
				generateHalfNote(G1note);
				generateHalfNote(A1note);
				generateHalfNote(F1note);
				generateQuarterNote(Bb1note);
				generateHalfNote(A1note);
				generateHalfNote(F1note);
				generateQuarterNote(Bb1note);
				generateFullNote(Ab1note);
				generateHalfNote(Fnote);
				generateQuarterNote(Bb1note);
				generateFullNote(Ab1note);
				generateHalfNote(F1note);
			}	
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	initPWM();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(generate_end_melody, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}

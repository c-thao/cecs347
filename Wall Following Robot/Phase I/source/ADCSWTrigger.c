// ADCSWTrigger.c
#include "tm4c123gh6pm.h"

// This initialization function sets up the ADC according to the
// following parameters.  Any parameters not explicitly listed
// below are not modified:
// Max sample rate: <=125,000 samples/second
// Sequencer 0 priority: 1st (highest)
// Sequencer 1 priority: 2nd
// Sequencer 2 priority: 3rd
// Sequencer 3 priority: 4th (lowest)
// SS2 triggering event: software trigger
// SS2 1st sample source: Ain2 (PE1)
// SS2 2nd sample source: Ain3 (PE0)
// SS2 2nd sample source: Ain1 (PE2)
// SS2 interrupts: flag set on completion but no interrupt requested
void ADC0_InitSWTriggerSeq2_Ch1(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000010;   // 1) activate clock for Port E
  delay = SYSCTL_RCGC2_R;         //    allow time for clock to stabilize
  GPIO_PORTE_DIR_R &= ~0x07;      // 2) make PE0-2 input
  GPIO_PORTE_AFSEL_R |= 0x07;     // 3) enable alternate function on PE0-2
  GPIO_PORTE_DEN_R &= ~0x07;      // 4) disable digital I/O on PE0-2
  GPIO_PORTE_AMSEL_R |= 0x07;     // 5) enable analog function on PE0-2
  SYSCTL_RCGC0_R |= 0x00010000;   // 6) activate ADC0 
  delay = SYSCTL_RCGC2_R;         
  SYSCTL_RCGC0_R &= ~0x00000300;  // 7) configure for 125K 
  ADC0_PC_R &= ~0xF;              // 8) clear max sample rate field
  ADC0_PC_R |= 0x1;               //    configure for 125K samples/sec
  ADC0_SSPRI_R = 0x3210;          // 8) Sequencer 2 3rd priority
  ADC0_ACTSS_R &= ~0x0004;        // 9) disable sample sequencer 2
  ADC0_EMUX_R &= ~0xF000;         // 10) seq2 is software trigger
  ADC0_SSMUX2_R = (ADC0_SSMUX2_R&0xFFFFFFF0)+2; // 11) channel Ain2 (PE1)
  ADC0_SSMUX2_R = (ADC0_SSMUX2_R&0xFFFFFF0F)+0x30; // 11) channel Ain3 (PE0)
  ADC0_SSMUX2_R = (ADC0_SSMUX2_R&0xFFFFF0FF)+0x100; // 11) channel Ain1 (PE2) for motor
  ADC0_SSCTL2_R = 0x0600;         // 12) no TS0 IE0 END0 D0 TS1 IE1 END1 D1 TS2 D2
																	//     yes IE2 END2
  ADC0_ACTSS_R |= 0x0004;         // 13) enable sample sequencer 2
}

//------------ADC0_InSeq2------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: 12-bit result of ADC conversion
void ADC0_InSeq2(unsigned long* result) {
	char i = 0;
	ADC0_PSSI_R = 0x0004;            // 1) initiate SS2
  while((ADC0_RIS_R&0x04)==0);   // 2) wait for conversion done
	while (i < 3) { // read three samples
		(*result++) = (ADC0_SSFIFO2_R&0xFFF);   // 3) read result
		i++;
	}
  ADC0_ISC_R = 0x0004;             // 4) acknowledge completion
}

#include "tm4c123gh6pm.h"
#include "ADCSWTrigger.h"
#include "Nokia5110.h"
#include "PLL.h"

#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))
#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))

// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

// global variables
unsigned long ADCvalue[3];
unsigned long PWMvalue, sens1, sens2;
unsigned long PWMleft, PWMright;

// initialize port b for
// use with motor direction
void Motor_Init(unsigned long period, unsigned long duty){
	SYSCTL_RCGC0_R |= 0x00100000;      // 1) activate PWM0
  SYSCTL_RCGC2_R |= 0x02;            // 2) activate port B
  while((SYSCTL_PRGPIO_R&0x02) == 0){};
	
	// PWM port b 4 & 5 DIR
	GPIO_PORTB_DIR_R   |= 0x30;           // PB4&5 outputs
	GPIO_PORTB_AFSEL_R |= 0x00;           // disable alt funct on PB6
  GPIO_PORTB_PCTL_R &= ~0x00000000;     // configure PB6 as PWM0
  GPIO_PORTB_PCTL_R |= 0x00000000;
  GPIO_PORTB_AMSEL_R &= ~0x30;          // disable analog functionality on PB6
  GPIO_PORTB_DEN_R |= 0x30;             // enable digital I/O on PB6
	
	// PWM port b 6 & 7 PWM
  GPIO_PORTB_AFSEL_R |= 0xC0;           // enable alt funct on PB6
  GPIO_PORTB_PCTL_R &= ~0xFF000000;     // configure PB6 as PWM0
  GPIO_PORTB_PCTL_R |= 0x44000000;
  GPIO_PORTB_AMSEL_R &= ~0xC0;          // disable analog functionality on PB6
  GPIO_PORTB_DEN_R |= 0xC0;             // enable digital I/O on PB6
  SYSCTL_RCC_R = 0x00100000 |           // 3) use PWM divider
      (SYSCTL_RCC_R & (~0x000E0000));   //    configure for /2 divider
  PWM0_0_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_0_GENA_R = 0xC8;                 // low on LOAD, high on CMPA down
  // PB6 goes low on LOAD
  // PB6 goes high on CMPA down
  PWM0_0_GENB_R = (PWM_0_GENB_ACTCMPBD_ONE|PWM_0_GENB_ACTLOAD_ZERO);
  // PB7 goes low on LOAD
  // PB7 goes high on CMPB down
  PWM0_0_LOAD_R = period;           // 5) cycles needed to count down to 0
  PWM0_0_CMPA_R = duty;             // 6) count value when output rises
	PWM0_0_CMPB_R = duty;             // 6) count value when output rises
  PWM0_0_CTL_R |= 0x00000001;           // 7) start PWM0
  PWM0_ENABLE_R |= 0x00000003;          // enable PB6&7/M0PWM0&1
}


// change duty cycle of PB4
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void PWM0A_Duty(unsigned long duty){
  PWM0_0_CMPA_R = duty;             // 6) count value when output rises
}

// change duty cycle of PB5
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void PWM0B_Duty(unsigned long duty){
  PWM0_0_CMPB_R = duty;             // 6) count value when output rises
}

// Generate low and high PWM
// using two pins of Port A
// pin 3 and pin 4
void SysTick_Handler(void){
		ADC0_InSeq2(ADCvalue);
		// obtain cm measure of distance
		sens1 = (27500 / ADCvalue[0]);
		sens2 = (27500 / ADCvalue[1]);
	  
		PWMvalue = 40000 * ADCvalue[2]/4096; // get PWM
	  if (sens1 <= 20 && sens2 <= 20) {
			if (sens1 > sens2) { // go to widest direction
				PWM0A_Duty(40000 * sens1/40);
				PWM0B_Duty(40000 * sens2/80);
			}
			else {
				PWM0A_Duty(40000 * sens1/80);
				PWM0B_Duty(40000 * sens2/40);
			}
		}
		else { // adjust regularly base on distance
			if (sens1 > 80) {
				PWM0A_Duty(20000);
			}
			else {
				PWM0A_Duty(40000 * sens1/ 80); // change PWM value
			}
			if (sens2 > 80) {
				PWM0B_Duty(20000);
			}
			else {
				PWM0B_Duty(40000 * sens2/ 80); // change PWM value
			}
		}
}

// initialize port f as leds
void LED_Init(void){  unsigned long volatile delay;
  SYSCTL_RCGC2_R    |= 0x00000020; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R  = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R    = 0x0E;       // allow changes to PF3-1
  GPIO_PORTF_DIR_R  |= 0x0E;       // (c) make PF0 in (built-in button)
  GPIO_PORTF_AFSEL_R = 0x00;       // disable alt funct on PF3-1
  GPIO_PORTF_DEN_R  |= 0x0E;       // enable digital I/O on PF3-1
  GPIO_PORTF_PCTL_R  = 0x00;       // configure PF3-1 as GPIO
  GPIO_PORTF_AMSEL_R = 0x00;       // disable analog functionality on PF3-1
	
  NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
  NVIC_ST_RELOAD_R = 500000;     // reload value for 0.010ms
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
  NVIC_ST_CTRL_R = 0x00000007;  // enable with core clock and interrupts
}

int main(void){ unsigned long volatile delay; // delay
  DisableInterrupts();  // disable interrupts while initializing
  PLL_Init();           // bus clock at 50 MHz
	Nokia5110_Init();     // initialize LCD
  ADC0_InitSWTriggerSeq2_Ch1(); // ADC initialization PE2/AIN1
  LED_Init();           // LEDs of PortF indicate motion
  Motor_Init(40000, 0);         // output from PB3/4 & PB6/7, SysTick interrupts
	EnableInterrupts();   // enable after all initialization are done
	
	GPIO_PORTF_DATA_R  = 0x02; // motor off red led
	GPIO_PORTB_DATA_R &= ~0x30; // forward motion
  while(1){
		// output value
		Nokia5110_Clear();
		Nokia5110_OutString("dist right");
		Nokia5110_SetCursor(0,1); //move to row 2
		Nokia5110_OutULDec(sens1);
		Nokia5110_OutString("cm");
		Nokia5110_SetCursor(0,2); // move to row 3
		Nokia5110_OutString("dist left");
		Nokia5110_SetCursor(0,3);// move to row 4
		Nokia5110_OutULDec(sens2);
		Nokia5110_OutString("cm");
		Nokia5110_SetCursor(0,4);// move to row 5
		Nokia5110_OutString("Duty Cycle");
		Nokia5110_SetCursor(0,5);// move to row 6
		Nokia5110_OutULDec(100*ADCvalue[2]/4096);
    for(delay=0; delay<100000; delay++){};
  }
}

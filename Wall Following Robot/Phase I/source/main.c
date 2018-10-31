/***************************************************
 * Author:           Chou Thao                     *
 * Student ID:       012647516                     *
 * Course Professor: Min He                        *
 * Course Name:      CECS 347                      *
 * Description:                                    *
 *   This program initializes the default PWM of   *
 * a motor to the input data of a potentiometer.   *
 * The PWM is updated from there on portionally    *
 * to the distance obtain from an ADC sequencer    *
 * with input data from two IR sensors. During     *
 * each PWM update, the onboard LEDS are updated   *
 * to indicate the current position of the robot.  *
 * As the program runs, information about the dis- *
 * distances and duty cycle of the PWM are dis-    *
 * played onto a LCD.                              *
 ***************************************************/
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

// Basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

// Global variables
unsigned long ADCvalue[3];
unsigned long PWMvalue, sens1, sens2;
unsigned long PWMleft, PWMright;

// Initialize Port B for the motors with hardware PWM,
// PB6 & PB7 for pwm motor control and PB4 & PB5 for 
// motor direction
void Motor_Init(unsigned long period, unsigned long duty){
    PWMvalue = 0;
    SYSCTL_RCGC0_R |= 0x00100000;        // activate PWM0
    SYSCTL_RCGC2_R |= 0x02;              // activate port B
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
    SYSCTL_RCC_R = 0x00100000 |           // use PWM divider
      (SYSCTL_RCC_R & (~0x000E0000));     // configure for /2 divider
    PWM0_0_CTL_R = 0;                     // re-loading down-counting mode
    PWM0_0_GENA_R = 0xC8;                 // low on LOAD, high on CMPA down
    // PB6 goes low on LOAD
    // PB6 goes high on CMPA down
    PWM0_0_GENB_R = (PWM_0_GENB_ACTCMPBD_ONE|PWM_0_GENB_ACTLOAD_ZERO);
    // PB7 goes low on LOAD
    // PB7 goes high on CMPB down
    PWM0_0_LOAD_R = period;               // cycles needed to count down to 0
    PWM0_0_CMPA_R = duty;                 // count value when output rises CMPA
    PWM0_0_CMPB_R = duty;                 // count value when output rises CMPB
    PWM0_0_CTL_R |= 0x00000001;           // start PWM0
    PWM0_ENABLE_R |= 0x00000003;          // enable PB6&7/M0PWM0&1
}


// Update the duty cycle of PB4
void PWM0A_Duty(unsigned long duty){
    PWMleft = duty;
    PWM0_0_CMPA_R = PWMleft;
}

// Update the duty cycle of PB5
void PWM0B_Duty(unsigned long duty){
    PWMright = duty;
    PWM0_0_CMPB_R = PWMright;
}

// Collects the data from the sensors and converts the data
// into centimeters to update the pwm and onboard leds
void SysTick_Handler(void){
    // grab the data from the ir sensors
    // and convert them into cm
    ADC0_InSeq2(ADCvalue);
    sens1 = (27500 / ADCvalue[0]);
    sens2 = (27500 / ADCvalue[1]);
    
    // update the PWM of the motors depending on the
    // distance from the sensor data
    if (PWMvalue == 0) {
         // pwm of 0 uses the duty cycle of the potentiometer
        PWMvalue = 40000 * ADCvalue[2]/4096; // get PWM
        PWM0A_Duty(PWMvalue);
        PWM0B_Duty(PWMvalue);
    }
    else if (sens1 <= 20 && sens2 <= 20) {
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
	
    // update leds the onboard leds indicating the
    // position of the robot car
    GPIO_PORTF_DATA_R &= ~0x0E; // reset leds
    if (PWMleft > PWMright) {
        GPIO_PORTF_DATA_R |= 0x08; // right turn grn
    }
    else if (PWMright > PWMleft){
        GPIO_PORTF_DATA_R |= 0x04; // left turn blue
    }
    else if (PWMright == 0 && PWMleft == 0) {
        GPIO_PORTF_DATA_R |= 0x02; // no motion red
    }
    else {
        GPIO_PORTF_DATA_R |= 0x00; // going forward none
    }
}

// Initialize Port F enabling the onboard leds
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
	
    NVIC_ST_CTRL_R = 0;              // disable SysTick during setup
    NVIC_ST_RELOAD_R = 500000;       // reload value for 0.010ms
    NVIC_ST_CURRENT_R = 0;           // any write to current clears it
    NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
    NVIC_ST_CTRL_R = 0x00000007;     // enable with core clock and interrupts
}

int main(void){
    unsigned long volatile delay; // delay variable for software delay
    DisableInterrupts();  // disable interrupts while initializing
    PLL_Init();           // initializes bus clock at 50 MHz
    Nokia5110_Init();     // initialize LCD
    ADC0_InitSWTriggerSeq2_Ch1(); // ADC initialization PE2/AIN1
    LED_Init();           // initializes LEDs of PortF indicate direction
    Motor_Init(40000, 0); // output from PB3/4 & PB6/7
    EnableInterrupts();   // enable after all initialization are done
    
    GPIO_PORTF_DATA_R  = 0x02;  // motor off red led
    GPIO_PORTB_DATA_R &= ~0x30; // forward motion
    while(1){
        Nokia5110_Clear(); // clear the lcd screen
        
        // update the lcd with the distances of them
        // walls on the left and right
	Nokia5110_OutString("dist: rt  lt");
	Nokia5110_SetCursor(0,1);  //move to row 2
	Nokia5110_OutString("cm    ");
	Nokia5110_OutULDec(sens2); // left sensor in cm
	Nokia5110_OutString("  ");
	Nokia5110_OutULDec(sens1); // right sensor in cm
        Nokia5110_SetCursor(0,2);  // move to row 3
        
        // update the lcd with the current duty cycle
        // of the left and right pwm
	Nokia5110_OutString("Duty Cycle");
	Nokia5110_SetCursor(0,3);  // move to row 4
	Nokia5110_OutULDec((100*ADCvalue[2])/4096);
	Nokia5110_SetCursor(0,4); // move to row 5
	Nokia5110_OutString("Left  Right");
	Nokia5110_OutString("  ");
	Nokia5110_SetCursor(0,5);  // move to row 6
	Nokia5110_OutString(" ");
	Nokia5110_OutULDec((100*PWMleft)/40000);  // left
	Nokia5110_OutString("     ");
	Nokia5110_OutULDec((100*PWMright)/40000); // right
        
        // software delay before updating the lcd
        for(delay=0; delay<100000; delay++){};
    }
}

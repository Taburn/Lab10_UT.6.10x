// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****

	#define SYSCTL_RCGC2_R					(*((volatile unsigned long *)0x400FE108))

	// Port B Register Addresses
	#define PORTB_DATA       				(*((volatile unsigned long *)0x400050FC))	// Bit specific: PB0-5
	#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
	#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
	#define GPIO_PORTB_PUR_R        (*((volatile unsigned long *)0x40005510))
	#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
	#define GPIO_PORTB_LOCK_R       (*((volatile unsigned long *)0x40005520))
	#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
	#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))

	// Port E Register Addresses
	#define PORTE_DATA       				(*((volatile unsigned long *)0x4002401C))	// Bit specific: PE0-2
	#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
	#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
	#define GPIO_PORTE_PUR_R        (*((volatile unsigned long *)0x40024510))
	#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
	#define GPIO_PORTE_LOCK_R       (*((volatile unsigned long *)0x40024520))
	#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
	#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))

	// Port F Register Addresses
	#define PORTF_DATA       				(*((volatile unsigned long *)0x40025028))	// Bit specific: PF1,3
	#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
	#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
	#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
	#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
	#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
	#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
	#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
	#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
	
	// SysTick Register Addresses
	#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
	#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
	#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))

// FUNCTION PROTOTYPES: Each subroutine defined
	void DisableInterrupts(void); // Disable interrupts
	void EnableInterrupts(void);  // Enable interrupts

	void SysTickInit(void);
	void SysTickWait_10ms(unsigned long delay);
	void PortBInit(void);
	void PortEInit(void);
	void PortFInit(void);
	void Wait100ms(unsigned long wait);

// ***** 3. Subroutines Section *****

int main(void){ 
  unsigned short CurrentState = 0;
	unsigned short ButtonInput = 0;
	
	struct traffic_state{
		unsigned short PortBValue;
		unsigned short PortFValue;
		unsigned short Wait;				// Units of 100 ms
		unsigned char NextState[8];
	};

		struct traffic_state IS[10] = {					// IS = intersection
	//		 Next state: N W S   k
	// PB    PF   Wait 0 1 2 3 4 5 6 7
		{0x21, 0x2, 0,  {0,1,0,1,1,1,1,1}},	// 0 SouthG
		{0x22, 0x2, 3,  {0,2,0,2,4,4,4,4}},	// 1 SouthY
		{0x0C, 0x2, 0,  {2,2,3,3,3,3,3,3}},	// 2 WestG
		{0x14, 0x2, 3,  {2,2,0,0,4,4,4,4}},	// 3 WestY
		{0x24, 0x8, 0,  {4,5,5,5,4,4,4,4}},	// 4 Walk
		{0x24, 0x0, 3,  {6,6,6,6,6,6,6,6}},	// 5 WalkFlash0
		{0x24, 0x8, 3,  {7,7,7,7,7,7,7,7}},	// 6 WalkFlash1
		{0x24, 0x0, 3,  {8,8,8,8,8,8,8,8}},	// 7 WalkFlash2
		{0x24, 0x8, 3,  {9,9,9,9,9,9,9,9}},	// 8 WalkFlash3
		{0x24, 0x0, 3,  {4,2,0,0,4,4,4,4}}	// 9 WalkFlash4
	};

	// In C90, all variable declarations must happen before any executable statements in a function or other code block.
	TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	
	SysTickInit();
	PortBInit();
	PortEInit();
	PortFInit();

  EnableInterrupts();
	
  while(1){
		PORTB_DATA = IS[CurrentState].PortBValue;			// Write values to LEDs
		PORTF_DATA = IS[CurrentState].PortFValue;
		SysTickWait_10ms(IS[CurrentState].Wait * 10);	// Wait value is in 100 ms, so *10 to go from number of 100ms to 10ms.
		ButtonInput = PORTE_DATA;											// Read buttons (see if this works as an unsigned short)
		CurrentState =	IS[CurrentState].NextState[ButtonInput];	// Set current state to next state
  }
}

void SysTickInit(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}

// for a 80 MHz clock, SysTick decrements every 12.5 ns
void SysTickWait_10ms(unsigned long delay){
	unsigned long i;
	for(i=0; i<delay; i++){
  	NVIC_ST_RELOAD_R = 800000-1;  					// number of counts to wait (800000 * 12.5 ns = 10 ms)
  	NVIC_ST_CURRENT_R = 0;       						// any value written to CURRENT clears
  	while((NVIC_ST_CTRL_R&0x00010000)==0){} // wait for count flag
	}
}

void PortBInit(void){
	volatile unsigned long delay;
	SYSCTL_RCGC2_R = SYSCTL_RCGC2_R | 0x1<<1;	// unlock port B clocks
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTB_LOCK_R = 	0x4C4F434B;	// enable write access to CR
	GPIO_PORTB_CR_R = 		0x3F;				// commit AFSEL, DEN, PUR, PDR values
	GPIO_PORTB_DIR_R = 		0x3F;				// all outputs
	GPIO_PORTB_AFSEL_R = 	0x00;				// use no peripherals
	GPIO_PORTB_DEN_R = 		0x3F;				// enable digital functions
	GPIO_PORTB_AMSEL_R = 	0x00;				// use no analog modes
	GPIO_PORTB_PCTL_R = 	0x00;				// use no peripherals
}
	
void PortEInit(void){
	volatile unsigned long delay;
	SYSCTL_RCGC2_R = SYSCTL_RCGC2_R | 0x1<<4;	// unlock port E clocks
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTE_LOCK_R = 	0x4C4F434B;	// enable write access to CR
	GPIO_PORTE_CR_R = 		0x7;				// commit AFSEL, DEN, PUR, PDR values
	GPIO_PORTE_DIR_R = 		0x0;				// all inputs
	GPIO_PORTE_AFSEL_R = 	0x0;				// use no peripherals
	GPIO_PORTE_DEN_R = 		0x7;				// enable digital functions
	GPIO_PORTE_AMSEL_R = 	0x0;				// use no analog modes
	GPIO_PORTE_PCTL_R = 	0x0;				// use no peripherals
}
	
void PortFInit(void){
	volatile unsigned long delay;
	SYSCTL_RCGC2_R = SYSCTL_RCGC2_R | 0x1<<5;	// unlock port F clocks
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTF_LOCK_R = 	0x4C4F434B;	// enable write access to CR
	GPIO_PORTF_CR_R = 		0xA;				// commit AFSEL, DEN, PUR, PDR values
	GPIO_PORTF_DIR_R = 		0xA; 				// PF1,3 are outputs
	GPIO_PORTF_AFSEL_R = 	0x0;				// use no peripherals
	GPIO_PORTF_DEN_R = 		0xA;				// enable digital functions
	GPIO_PORTF_AMSEL_R = 	0x0;				// use no analog modes
	GPIO_PORTF_PCTL_R = 	0x0;				// use no peripherals
}

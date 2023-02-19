// BEE 425 AA, Winter 2021
// Austin Gilbert, Adrian, & Carol Kao, 03/8/21
// Modified from Valvano et al, UTexas & Joseph Decuir, UWashington
// BEE425L21 Lab 4, Version 1: Wave Generator
// main.c


// 1.0 DESCRIPTION:
// ---------------------------------------------------------------------------------
// Periodic Waveform Generator with four wave functions (square, ramp, sine,
// triangle), a 10 Hz to 10 kHz variable frequency range, and 32 mV to 8 V
// peak-to-peak amplitude range.
// 
// User selects waveform type via 4x4 Keypad, then presses star. Program changes
// from Keypad-scanning mode to Waveform Display mode, displaying one of the four
// selected waveform types. SW1 and SW2 will then be active. Pressing SW1 will
// allow the user to alter the potentiometer value to change the amplitude of the
// wave. Pressing SW2 allows the user to alter the potentiometer value to change
// the frequency.
// 
// The low pass filter will use adaptive mcontrols to smooth the output depending
// on the frequency and upon each waveform.


// 2.0 PRE-PROCESSOR DIRECTIVES SECTION
// ---------------------------------------------------------------------------------
// Constant declarations to access port registers using symbolic names 
// modified to include TM4C123GH6PM.h & system_TM4C123.h definitions
#include "TM4C123GH6PM.h"				// Keil seems to ignore them
#include "system_TM4C123.h"			// ditto
#include "TExaS.h"

// master port clock 
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))

// GPIO Port F
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))

// NVIC = Systick
#define NVIC_ST_RELOAD_R				(*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CTRL_R				  (*((volatile unsigned long *)0xE000E010))

// GPIO Port E					
#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_PUR_R        (*((volatile unsigned long *)0x40024510))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_CR_R         (*((volatile unsigned long *)0x40024524))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))

// ADC AIN0 (PE3)					
#define SYSCTL_RCGCADC_R				(*((volatile unsigned long *)0x400FE638))
#define ADC0_ACTSS_R						(*((volatile unsigned long *)0x40038000))
#define ADC0_EMUX_R							(*((volatile unsigned long *)0x40038014))
#define ADC0_SSMUX3_R						(*((volatile unsigned long *)0x400380A0))
#define ADC0_SSCTL3_R						(*((volatile unsigned long *)0x400380A4))
#define ADC0_PSSI_R							(*((volatile unsigned long *)0x40038028))
#define ADC0_SSFIFO3_R					(*((volatile unsigned long *)0x400380A8))
#define ADC0_IM_R								(*((volatile unsigned long *)0x40038008))	
#define ADC0_ISC_R							(*((volatile unsigned long *)0x4003800C))	
#define ADC0_RIS_R							(*((volatile unsigned long *)0x40038004))	
#define ADC0_SSPRI_R						(*((volatile unsigned long *)0x40038020))

// GPIO Port D					
#define GPIO_PORTD_DATA_R       (*((volatile unsigned long *)0x400073FC))
#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_AFSEL_R      (*((volatile unsigned long *)0x40007420))
#define GPIO_PORTD_PUR_R        (*((volatile unsigned long *)0x40007510))
#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))
#define GPIO_PORTD_CR_R         (*((volatile unsigned long *)0x40007524))
#define GPIO_PORTD_AMSEL_R      (*((volatile unsigned long *)0x40007528))

// GPIO Port C					
#define GPIO_PORTC_DATA_R       (*((volatile unsigned long *)0x400063FC))
#define GPIO_PORTC_DIR_R        (*((volatile unsigned long *)0x40006400))
#define GPIO_PORTC_AFSEL_R      (*((volatile unsigned long *)0x40006420))
#define GPIO_PORTC_PUR_R        (*((volatile unsigned long *)0x40006510))
#define GPIO_PORTC_DEN_R        (*((volatile unsigned long *)0x4000651C))
#define GPIO_PORTC_CR_R         (*((volatile unsigned long *)0x40006524))
#define GPIO_PORTC_AMSEL_R      (*((volatile unsigned long *)0x40006528))

// GPIO Port B					
#define GPIO_PORTB_DATA_R       (*((volatile unsigned long *)0x400053FC))
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_PUR_R        (*((volatile unsigned long *)0x40005510))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_CR_R         (*((volatile unsigned long *)0x40005524))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))	


// 3.0 DECLARATIONS SECTION
// ---------------------------------------------------------------------------------
//   Global Variables
int Mode;																		// 0 = wave, 1 = scan
int SW1;																		// first switch
int SW2;  																	// second switch
int WaveM;																	// waveform mode: 1 sine, 2 ramp,
																						// 3 triangle, 4 square
int AdjustM; 																// adjustment mode: 0 amplitude, 1 frequency
int DAC;																		// DAC output
int SWI;																		// Sine Wave index: 12 entries
int RWI;																		// Ramp Wave index: 16 entries
int TWI;																		// Triangle Wave index: 12 entries
int SQWI;																		// Sqare wave index: 2 entries

// 12 sample signwave table
int SinT[] = {0x80,0xC0,0xEE,0xFF,0xEE,0xC0,0x80,0x40,0x12,0x00,0x12,0x40};
// 16 sample rampwave table
int RampT[] = {0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0,
							 0xB0, 0xC0, 0xD0, 0xE0, 0xF0};
// 12 sample trianglewave table
int TriT[] = {0x80,0xAB,0xD5,0xFF,0xD5,0xAB,0x80,0x55,0x2B,0x00,0x2B,0x55};
int SquareT[] = {0x00, 0xFF};								// 2 sample squarewave table
int KeyColCtr;															// keyboard column counter
int KeyCol;																	// Keyboard column output
int KeyColIndex;														// Keyboard column index
int KeyColumn;															// Keyboard column drive
int KeyRow;																	// Keyboard row input
int KeyRowIndex;														// Keyboard row
int KeyCode;																// 4x4 key location
int KeyHex;																	// encoded hex value
int LastKeyHex;															// last encoded hex value
int KeyDetect;															// emperical key detect
int KeyString;															// the 8-bit string of each 4-bit digit
int LPFM;																		// Low Pass Filter Mode: 0-3 off; 4-7 on
volatile int AIN0; 													// input from 12-bit ADC on PE3
double pot;																	// double casted input from 12-bit ADC on PE3
double potRatio;														// potentiometer value as a ratio/percent
double ampRatio;														// last potentiometer ratio value for amplitude
double freqRatio;														// last potentiometer ratio value for frequency
double outDouble;														// double value for waveforms
double maxDouble;														// double value for maximum timing (used in 5.8)
long min;																		// long value for minimum time
long max;																		// long value for maximum time
unsigned long ColorCount; 									// color counter
volatile unsigned long time;								// long value for NVIC value

//   Function Prototypes
void Delay(void);
void NVIC_Init(int);												// added NVIC function, variable value
void PortF_Init(void);											// set up Port F for Switches and LEDs
void KBD_Init(void);												// set up Port C & D for 4x4 keyboard
void DAC_Init(void);												// set up Port B for DAC	
void ADC_Init(void);		  									// add ADC0 function on PE3
int Output(int, double); 										// turns wave table output into
																						// amplitude-adjustable output
long Timing(long, long, double);						// takes min and max and outputs
																						// pot-adjusted value

// 4.0 MAIN CODE BLOCK
// ---------------------------------------------------------------------------------
// MAIN: Mandatory for a C Program to be executable
int main(void){
	NVIC_Init(15999); 												// Call Systick initialization to 1 msec
	PortF_Init();       											// Call initialization of SW and LEDs
	ADC_Init();																// call ADC initialization
	DAC_Init();																// Call initialization of DAC port B	
	KBD_Init();				  											// Call initialization of 4x4 keyboard
	
	while(1){
		
		// KEYPAD SCANNING MODE
		////////////////////////////////////////////////////////////////////////////////
		if (Mode == 0) {
			GPIO_PORTF_DATA_R = 0x02; 						// Turn LED red (Port F)
			GPIO_PORTB_DIR_R = 0x3F;							// turn off PB7-6; keep PB5-0 on
			time = 15999;
			
			// Keypad Scan - PC to PD - result to PB and PF
			Delay();															// wait specfied time (1ms)
			if (KeyDetect == 0) KeyColCtr +=1;		// count by 1
			KeyColIndex = (KeyColCtr) & 0x3;			// index should be 0, 1, 2 or 3
			if (KeyColIndex==0) KeyColumn = 0xE0;	// ground PC4: A, B, C, D
			if (KeyColIndex==1) KeyColumn = 0xD0;	// ground PC5: 3, 6, 9, #\F
			if (KeyColIndex==2) KeyColumn = 0xB0;	// ground PC6: 2, 5, 8, 0
			if (KeyColIndex==3) KeyColumn = 0x70;	// ground PC7: 1, 4, 7, *\E
			GPIO_PORTC_DATA_R = KeyColumn;				// drive column ports
				
			// read row ports
			Delay();															// wait for ports to settle
			KeyRow = (GPIO_PORTD_DATA_R & 0x0F);	// capture key row
				
			// detect and process keys
			if (KeyRow==0x0F)	{										// no key found
				GPIO_PORTF_DATA_R = 0x02;						// keep indicator RED
				KeyRowIndex = 0x4;									// indicating not found
				KeyDetect = 0;
				LastKeyHex = KeyHex;								// Store last key hex
				
			} else {
				KeyDetect = 1;											// key found
				GPIO_PORTF_DATA_R = 0x04;						// set indicator BLUE	
				if (KeyRow==0xE) KeyRowIndex = 0x0;	// row 0: D, #\F, 0, *\E
				if (KeyRow==0xD) KeyRowIndex = 0x1;	// row 1: C, 9, 8, 7
				if (KeyRow==0xB) KeyRowIndex = 0x2;	// row 2: B, 6, 5, 4
				if (KeyRow==0x7) KeyRowIndex = 0x3;	// row 3: A, 3, 2, 1
				KeyCode = (KeyColIndex << 2)+ KeyRowIndex; // assemble key code
				
				// GPIO_PORTB_DATA_R = KeyCode;   output key code
				// insert hex encoder: 16 key codes in, 16 hex values out
				if(KeyCode==0)	KeyHex = 0xD;
				if(KeyCode==1)	KeyHex = 0xC;
				if(KeyCode==2)	KeyHex = 0xB;
				if(KeyCode==3)	KeyHex = 0xA;
				if(KeyCode==4)	KeyHex = 0xF;	 			// map # to 15
				if(KeyCode==5)	KeyHex = 0x9;
				if(KeyCode==6)	KeyHex = 0x6;
				if(KeyCode==7)	KeyHex = 0x3;	
				if(KeyCode==8)	KeyHex = 0x0;
				if(KeyCode==9)	KeyHex = 0x8;
				if(KeyCode==10)	KeyHex = 0x5;
				if(KeyCode==11)	KeyHex = 0x2;
				if(KeyCode==12)	KeyHex = 0xE;				// map * to 14
				if(KeyCode==13)	KeyHex = 0x7;
				if(KeyCode==14)	KeyHex = 0x4;		
				if(KeyCode==15)	KeyHex = 0x1;
				
				// Create KeyString
				KeyString = KeyHex | ((LastKeyHex << 4) & 0xF0);
				Delay();
				GPIO_PORTB_DATA_R = KeyString & 0xFF; // Output to Port B
				
				// Pressing Last Key "Star"
				if ((KeyString & 0xF) == 0xE) {
					// Pressing First Key 1
					if (((KeyString >> 4) & 0xF) == 0x1) {
						WaveM = 1;											// Activate Sine Wave Mode
						Mode = 1;												// Branch to else loop
						
					// Pressing First Key 2
					} else if (((KeyString >> 4) & 0xF) == 0x2) {
						WaveM = 2;											// Activate Ramp Wave Mode
						Mode = 1;												// Branch to else loop
						
					// Pressing First Key 3
					} else if (((KeyString >> 4) & 0xF) == 0x3) {
						WaveM = 3;											// Activate Triangle Wave Mode
						Mode = 1;												// Branch to else loop
						
					// Pressing First Key 4
					} else if (((KeyString >> 4) & 0xF) == 0x4) {
						WaveM = 4;											// Activate Square Wave Mode
						Mode = 1;												// Branch to else loop
						
					// Pressing any other Key
					} else {
						int i;
						NVIC_Init(7999999);							// change timing for flashing
						// Flash RED LED
						for (i = 0; i <= 2; ++i) {			// for loop to flash twice
							GPIO_PORTF_DATA_R = 0x00;
							Delay();
							GPIO_PORTF_DATA_R = 0x02;
							Delay();
						}
						NVIC_Init(15999);								// change timing back for key-scanning
						Mode = 0;												// Restart Mode 0 loop
					}
				}
			}
		
		// WAVEFORM DISPLAY MODE
		////////////////////////////////////////////////////////////////////////////////
		} else if (Mode == 1) {
			// Initialize Ports & Timing 
			NVIC_Init(time);
			GPIO_PORTB_DATA_R = DAC;							// DAC = Port B Output
			GPIO_PORTE_DATA_R = LPFM; 						// LPFM = Port E Output
			SW1 = GPIO_PORTF_DATA_R & 0x10;				// sample port PF4
			SW2 = GPIO_PORTF_DATA_R & 0x01; 			// sample port PF0
			GPIO_PORTB_DIR_R = 0xFF;        			// restore PB7-PB0 output 
			
			// Initialize Switches
			if (SW1 == 0) AdjustM = 0;
			if (SW2 == 0) AdjustM = 1;
			
			// Variable LPFM
			if (WaveM == 1 || WaveM == 3) {	// Sine or Triangle wave
				if (time <= 133332 && time >= 3366) {
					LPFM = 1;													// 10 Hz - 396 Hz
				} else if (time < 3366 && time >= 336) {
					LPFM = 2;													// 3.96 kHz - 396 Hz
				} else if (time < 336 && time >= 132) {
					LPFM = 3; 												// 10 kHz - 3.96 kHz
				}
			} else if (WaveM == 2) { // Ramp Wave
				if (time <= 99999 && time >= 2524) {
					LPFM = 1;													// 10 Hz - 396 Hz
				} else if (time < 2524 && time >= 252) {
					LPFM = 2;													// 3.96 kHz - 396 Hz
				} else if (time < 252 && time >= 99) {
					LPFM = 3; 												// 10 kHz - 3.96 kHz
				}
			} else { // WaveM == 4
				LPFM = 0;														// LPF off
			}
			
			// Capture Current ADC Input
			ADC0_PSSI_R |= 8;											// 7) start a conversion at sequence 3
			while((ADC0_RIS_R & 8) == 0);					// 8) wait for conversion to complete
			AIN0 = ADC0_SSFIFO3_R;								// 9) capture the results
			pot = ((AIN0 >> 4) & 0xFF) * 1.0;			// get 8-bit potentiometer value
			potRatio = pot / 255.0;								// ratio of potentiometer turn
			ADC0_ISC_R = 8;						  					// 10) clear completion flag
			Delay();															// wait specified time (1ms)
			
			// Step All Wave-types
			SWI += 1;															// step sine wave index
			if(SWI == 12)	SWI = 0;								// counts up 12 times
			RWI += 1;															// step ramp wave index
			if(RWI == 16) RWI = 0;								// counts up 16 times
			TWI+=1;																// step triangle wave index
			if(TWI == 12)	TWI = 0;								// counts up 12 times
			SQWI += 1;														// step square wave index
			if(SQWI == 2) SQWI = 0;								// counts up 2 times
			
			// AMPLITUDE ADJUSTMENT MODE
			if (AdjustM == 0) {
				GPIO_PORTF_DATA_R = 0x08;						// Turn LED GREEN
				ampRatio = potRatio;								// save amplitude ratio
				
				// Sine Wave
				if (WaveM == 1) {
					if (freqRatio != 0) {							// keep freq value
						time = Timing(133332, 132, freqRatio);
					}
					DAC = Output(SinT[SWI], potRatio); // output sine
				// Ramp Wave
				} else if (WaveM == 2) {
					if (freqRatio != 0) {							// keep freq value
						time = Timing(99999, 99, freqRatio);
					}
					DAC = Output(RampT[RWI], potRatio); // output ramp
				// Triangle Wave
				} else if (WaveM == 3) {
					if (freqRatio != 0) {							// keep freq value
						time = Timing(133332, 132, freqRatio);
					}
					DAC = Output(TriT[TWI], potRatio); // output tri
				// Square Wave
				} else if (WaveM == 4) {
					if (freqRatio != 0) {							// keep freq value
						time = Timing(799999, 799, freqRatio);
					}
					DAC = Output(SquareT[SQWI], potRatio); // output square
				}
			
			// FREQUENCY ADJUSTMENT MODE
			} else { // AdjustM = 1
				GPIO_PORTF_DATA_R = 0x0C;						// Turn LED SKY BLUE
				freqRatio = potRatio;								// save frequency ratio
				
				// Sine Wave
				if (WaveM == 1) {
					time = Timing(133332, 132, potRatio); // change freq via pot
					DAC = Output(SinT[SWI], ampRatio); // output sine
				// Ramp Wave
				} else if (WaveM == 2) {
					time = Timing(99999, 99, potRatio); // change freq via pot
					DAC = Output(RampT[RWI], ampRatio); // output ramp
				// Triangle Wave
				} else if (WaveM == 3) {
					time = Timing(133332, 132, potRatio); // change freq via pot
					DAC = Output(TriT[TWI], ampRatio); // output tri
				// Square Wave
				} else if (WaveM == 4) {
					time = Timing(799999, 799, potRatio); // change freq via pot
					DAC = Output(SquareT[SQWI], ampRatio); // output square
				}
			}
		// Avoiding possible bugs
		} else {
			Mode = 0;
		}
	}
}


// 5.0 SUBROUTINE FUNCTIONS
// ---------------------------------------------------------------------------------
// 5.1 Initialize Systick timer
// 		int time -- the int value to use for NVIC time
void NVIC_Init(int time){
	volatile unsigned long Systick;
	NVIC_ST_RELOAD_R = time;		      // configure Systick with value "time"
  NVIC_ST_CTRL_R = 5;								// configure Systick for auto reload	
}
// 5.2 initialize port F pins for input and output
// PF4 and PF0 are inputs SW1 and SW2 respectively
// PF3,PF2,PF1 are outputs to the LED
void PortF_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) PF clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF 
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x11;          // 7) enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R = 0x1F;          // 8) enable digital pins PF4-PF0  
}
// 5.3 initialize keyboard ports: PC7-4 columns; PD3-0 rows; PD4-0 code results
void KBD_Init(void) {;
	SYSCTL_RCGC2_R |= 0x0C;           // 1) PC & PD clocks
	GPIO_PORTC_AMSEL_R = 0x00;        // 2) disable analog function
  GPIO_PORTC_AFSEL_R = 0x00;        // 3) no alternate function	
	GPIO_PORTC_DEN_R |= 0xF0;         // 4) enable digital pins PC7-PC4  
  GPIO_PORTC_DIR_R |= 0xF0;         // 5) PC7-PC4 output - turn off outputs
  GPIO_PORTD_AMSEL_R = 0x00;        // 6) disable analog function
	GPIO_PORTC_AFSEL_R = 0x00;        // 7) no alternate function	
	GPIO_PORTD_DEN_R |= 0xCF;         // 8) enable digital pins PD3-PD0 
  GPIO_PORTD_DIR_R &= 0x00;         // 5) PD3-PD0 input   	
  GPIO_PORTD_PUR_R = 0x0F;          // 7) enable pullup resistors on PD3-PD0 
}
// 5.4 initialize DAC output port: PB7-0
void DAC_Init(void)  {;
	SYSCTL_RCGC2_R |= 0x02;           // 1) PB clock
	GPIO_PORTB_AMSEL_R = 0x00;        // 2) disable analog function
  GPIO_PORTB_AFSEL_R = 0x00;        // 3) no alternate function	
	GPIO_PORTB_DEN_R = 0xFF;          // 4) enable digital pins PB7-PB0  
  GPIO_PORTB_DIR_R = 0xFF;          // 5) PB7-PB0 output   
}
// 5.5 initialize ADC input on PE3 - compare to Mazidi Ch7 p187, P7-1
void ADC_Init(void){volatile unsigned long AIN0; 
	SYSCTL_RCGC2_R |= 0x10;            // 0) E clock enable
	SYSCTL_RCGCADC_R |= 1;						 // 1) enable clock for ADC0	
  GPIO_PORTE_AFSEL_R |= 8;           // enable alternate function on PE3 
  GPIO_PORTE_DEN_R = 7;       		   // disable digital pin PE3, enable PE2-PE0	
  GPIO_PORTE_AMSEL_R |= 8;           // enable analog function PE3		
  GPIO_PORTE_DIR_R = 7;              // PE3 input, PE2,PE1,PE0 output 
	ADC0_ACTSS_R &= ~8;	    	  			 // 2) disable SS3 during configuration
//	SYSCTL_RCGC2_R |= 0x00010000;			 //  activate ADC0                     ?
//  GPIO_PORTE_PCTL_R = 0x0000;      // GPIO clear bit PCTL  
//  SYSCTL_RCGC2_R &= ~0x00000300;	 //  configure for 125K sample rate    ?
//	ADC0_SSPRI_R = 0x0123;					 // sequencer 3 is highest priority
	ADC0_EMUX_R &= ~0xF000;					   // 3) software trigger conversion
	ADC0_SSMUX3_R = 0;						  	 // 4) select input from channel 0
	ADC0_SSCTL3_R |= 6;		      			 // 5) sample and set time at 1st sample
	ADC0_IM_R |= (1<<3);								 // 14) enable imterrupt mask for SS3
	ADC0_ACTSS_R |= 8;			    			 // 6) enable ADC0 sequencer 0
}
// 5.6 Delay function, using Systick, 
void Delay(void){
	while ((NVIC_ST_CTRL_R & 0x10000) == 0);
}
// 5.7 Wave output function
// 		int waveIn      -- the hex input wave value
// 		double potRatio -- the potentiometer ratio value to use as a percentage
// 			returns				-- the pot-adjusted output int
int Output(int waveIn, double potRatio) {
	outDouble = (waveIn - 0x80) * 1.0;
	outDouble *= potRatio;
	return (int)outDouble + 0x80;
}
// 5.8 Frequency Timing function
// 		long max        -- max time value
//		long min        -- min time value
// 		double potRatio -- the potentiometer ratio value to use as a percentage
//			returns				-- the pot-adjusted output long
long Timing(long max, long min, double potRatio) {
	max -= min;
	maxDouble = (double)max;
	maxDouble *= potRatio;
	return (long)maxDouble + min;
}

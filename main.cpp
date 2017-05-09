/*
 * AUTHOR:        Alec Selfridge, Dylan Smith, Joshua Hightower, Luis Orozco
 * FILENAME:      main.cpp
 * VERSION:       1.1
 * PROJECT:       HOSS Rover
 * DATE CREATED:  02/02/2017
 * LAST MODIFIED: 05/08/2017
 */
 #include <math.h>
#include <LPC17xx.h>
#include "SPLC780D.h"
#include "LPC1768regmap.h"
#include "SHARP_IR.h"

// buttons & debouncing parameters
#define BTN1_PIN           0     //P0.8 - dip 6
#define BTN2_PIN           1     //P0.9 - dip 5
#define BTN3_PIN           7     //P0.7 - dip 7
#define BTN4_PIN           6     //P0.6 - dip 8
#define DEBOUNCE_MS        20
#define SYSTICK_LOAD_VALUE 99999 // 1ms
#define SYSTICK_ENABLE     0
#define SYSTICK_INT        1
#define SYSTICK_CLOCK      2
// UART parameters
#define UART2_PWR          24
#define FIFO_EN            0
#define FIFO_TX_CLR        2
#define WORD_LEN_SEL       0
#define DLAB_EN            7
#define PCLK_UART2         16
#define UART_THRE          5
#define NUMBASE            48
// motor parameters
#define M1_PIN             4 //P2.4
#define M2_PIN             3 //P2.3
#define PI                 3.14159
#define sqrt2              1.41421
#define halfspeed          0x3f69c //259740
#define threquarterspeed   0x5f1ea //389610
#define fullspeed          0x7ed38 //519480
#define IRSpace            25.4 //1" = 25.4, 1.25" = 31.75, 1.5" = 38.1, 1.75" = 44.45
                                //2" = 50.8, 2.25" = 57.15, 2.5" = 63.5, 2.75" = 69.85 
                                //3" = 76.2, 3.25" = 82.55, 3.5" = 88.9, 3.75" = 95.25
/*
#define halfspeed          0x3f69c //259740
#define threequarterspeed  0x2f8f5 //194805
#define fullspeed          0x1fb4e //129870
*/
// system initializations
void initGPIO(void);
void initSysTick(void);
void initUART2(unsigned long baud);
void initLCD(void);
void initMotor(void);
void initTimers(void);
// menu functions
void putchar(unsigned char c);
void putstring(const char *s, unsigned char start, unsigned char end);
void menuDisplay(void);
void feetToMeters(void);
void metersToFeet(void);
void startOperation(void);
void displayInt(void);
void changeState(void);
void delay(int x);
void operationStopped(void);
void powerScreenDown(void);

// IR objects
SHARP_IR ir1 = SHARP_IR(5); // IR sensor on ADC0.5
SHARP_IR ir2 = SHARP_IR(4); // IR sensor on ADC0.4

// UI variables
int btn1state   = 0;
int btn2state   = 0;
int btn3state   = 0;
int btn4state   = 0;
int btn1counter = 0;
int btn2counter = 0;
int btn3counter = 0;
int btn4counter = 0;
int btn1pressed = 0;
int btn2pressed = 0;
int btn3pressed = 0;
int btn4pressed = 0;
float distance    = 5;
int state       = 1;
bool menuState = false;
bool feet = true;
char str[3];

//motor control
int m1State = 0;
int m2State = 0;
int T0MC, T1MC, totalSteps;
int stepMax;
//radius of wheel and pipe in corresponding units (metric or imperial)
float rWheel = 40; 
float linearMax, stepArc, linearDist;
int temp = 0;

int main(void)
{
	SystemInit();
  float ir1_dist = 0;
	float ir2_dist = 0;
	float speedMult1 = 1.0;
	float speedMult2 = 1.0;
  // initializations
  initGPIO();
	initMotor();
	initTimers();
  initUART2(9600);
  initLCD();
  ir1.init();
  ir1.setImperial();
	ir2.init();
  ir2.setImperial();
  initSysTick();
  while(1)
  {
    /*
     * UI
     */
    if(state==1) //Start
		{
			if(btn4pressed == 1) {	//far right
				btn4pressed = 0;
				if(menuState) {
					if(feet) {
						feetToMeters();
						feet = false;
						menuDisplay();
					}
					else {
						metersToFeet();
						feet = true;
						menuDisplay();
					}
				}
			} //switch 4 
		
			if(btn1pressed == 1) {	//middle left
				btn1pressed = 0;
				if(menuState&(distance>1)){
					distance -= 1;
					menuDisplay();
				}
			} //switch 2
			
			if(btn2pressed == 1) { //far left
				btn2pressed = 0;
				if(menuState) {
					startOperation();
					changeState();
					delay(7500);
					powerScreenDown();
          stepArc = ((2*PI*rWheel*((1.8)/360.0)));//Calculate single step arc
          // motor calculations
          stepMax = IRSpace/stepArc; //Limits steps as a function of distance (in inches) between measurements and arc length distance
          if(feet)
            distance = (distance*0.3048);
          linearDist = distance*1000; //Maximum linear travelled distance in millimeters
          linearMax = linearDist/stepArc;
				}
				else {
					menuDisplay();
					menuState = true;
				}
			} //button 2
			
			if(btn3pressed == 1) { //middle right
				btn3pressed = 0;
				if(menuState){
					distance += 1;
					menuDisplay();
				}
			} //button 3
		} //state 1
		
		else if(state==2) //Running
		{
			if(btn2pressed == 1) { //left button - stop operation
				btn2pressed = 0;
				changeState();
				operationStopped();
			}
			else {
        /*
         * MOTORS
         */
        if(totalSteps < linearMax){
          while((T0MC < stepMax) && (T1MC < stepMax)){
            
            //Check if either timer is stopped. 
            //Increment totalSteps each iteration if either motor steps
            if(T0TCR == 0 || T1TCR == 0)
              
            
            //Checks if timer is stopped
            if(T1TCR == 0)
            {
              //Timer is reset and stopped
              T1TCR = 2;
              //Checks square wave level: 1 = high, 0 = low
              if(m2State == 0)
              {
                FIO2SET0 = (1<<M2_PIN);
                totalSteps++;
              }
              if(m2State == 1)
              {
                FIO2CLR0 = (1<<M2_PIN);
              }
              //Increments step counter
              T1MC++;
              //Flips square wave
              m2State = !m2State;
              //Resets timer
              T1TCR = 1;
            }
            //See above comments
            if(T0TCR == 0)
            {
              T0TCR = 2;
              if(m1State == 0)
              {
                FIO2SET0 = (1<<M1_PIN);
              }
              if(m1State == 1)
              {
                FIO2CLR0 = (1<<M1_PIN);
              }
              T0MC++;
              m1State = !m1State;
              T0TCR = 1;
            }
          };
          //Resets step counter
          T0MC = 0;
          T1MC = 0;
          //Reset and stop both timers
          T0TCR = 2; T1TCR = 2;
          //Update IR distance
          ir1.update();
          ir1_dist = ir1.getDistance();
          //Make a inifinity reading a relatively large number
          if(ir1_dist == -1)
            ir1_dist = 10;
          ir2.update();
          ir2_dist = ir2.getDistance();
          if(ir2_dist == -1)
            ir2_dist = 10;
          //Create speed multiplier
          speedMult1 = (ir1_dist/ir2_dist);
          speedMult2 = (ir2_dist/ir1_dist);
          //Limit to proper values
          //If using halfspeed max = 2.0
          //If using threequarterspeed max = 1.3
          //If using fullspeed max = 1.0
          //Limit all minimums to .0125 (1/8th)
          if(speedMult1 >= 2.0)
            speedMult1 = 2.0;
          if(speedMult2 >= 2.0)
            speedMult2 = 2.0;
          if(speedMult1 <= 0.0125)
            speedMult1 = 0.0125;
          if(speedMult2 <= 0.0125)
            speedMult2 = 0.0125;
          //Recalculate timer match value
          T0MR0 = speedMult1*halfspeed;
          T1MR0 = speedMult2*halfspeed;
          //Start timers
          T0TCR = 1; T1TCR = 1;
       }
       else {
         changeState();
				 operationStopped();
       }
			
      } //else
		} //state 2
		
		else // Stopped
		{
			
		} //state 3
  }
}

void initGPIO(void)
{
  // configure pin as GPIO
  PINSEL0 = 0;
  // configure buttons as inputs
  FIO0DIR1 |= (0<<BTN1_PIN);
  FIO0DIR1 |= (0<<BTN2_PIN);
  FIO0DIR0 |= (0<<BTN3_PIN);
  FIO0DIR0 |= (0<<BTN4_PIN);
}

void initSysTick(void)
{
  // load tick register  
  STRELOAD = SYSTICK_LOAD_VALUE;
  // enable systick, interrupts, and core clock
  STCTRL |= (1<<SYSTICK_ENABLE) | (1<<SYSTICK_INT) | (1<<SYSTICK_CLOCK);
}

// initializes UART2 to a specified baud rate, 8N1
void initUART2(unsigned long baud)
{
  unsigned int pclksel, pclkval, dlvalue;
  // power on and enable TXD2
  PCONP |= (1<<UART2_PWR);
  PINSEL0 |= 0x00100000;
  
  // enable FIFO and clear tx buffer
  U2FCR |= (1<<FIFO_EN) | (1<<FIFO_TX_CLR);
  // 8N1  
  U2LCR |= (0x03<<WORD_LEN_SEL) | (1<<DLAB_EN);
  
  // read PCLK value for UART2
  pclksel = (PCLKSEL1 >> PCLK_UART2) & 0x03;
  // determine the actual PCLK value
  switch(pclksel)
  {
  case 0x00:
    pclkval = SystemCoreClock / 4;
    break;
  case 0x01:
    pclkval = SystemCoreClock;
    break; 
  case 0x02:
    pclkval = SystemCoreClock / 2;
    break; 
  case 0x03:
    pclkval = SystemCoreClock / 8;
    break;
  // default case is reset
  default:
    pclkval = SystemCoreClock / 4;
  }
  
  // baud rate calculation as per the datasheet
  dlvalue = (pclkval / (16 * baud)); 
  U2DLL =  dlvalue & 0xFF;
  U2DLM = (dlvalue >> 8) & 0xFF;
  
  // clear DLAB
  U2LCR &= ~(1<<DLAB_EN);
}

void initLCD(void)
{
  // clear screen
  putchar(LCD_CMD_PREFIX);
  putchar(LCD_CLR);
  putstring("HOSS Rover V1.2", 0, 15);
}

void initMotor(void){
	// configure as GPIO
	PINSEL4 = 0x00000000;
	// configure motor pins as outputs
	FIO2DIR0 |= (1<<M1_PIN);
  FIO2DIR0 |= (1<<M2_PIN);
	stepArc = ((2*PI*rWheel*(1.8/360.0)));//Calculate single step arc
	stepMax = IRSpace/stepArc; //Limits steps as a function of distance (in inches) between measurements and arc length distance
	linearDist = 1066.8; //Maximum linear travelled distance in millimeters
	linearMax = linearDist/stepArc;
}

void initTimers(void)
{
//Enable PCTIM0,PCTIM1
PCONP |= 0x00000006;
//Set to utilize 1x clock speed
PCLKSEL0 |= 0x00000014;
//Set to interrupt on match. 
//Timer 0 interrupts to Exception 17
//Timer 1 interrupts to Exception 18
T0MCR |= 6;  T1MCR |= 6;

//NVIC_EnableIRQ(TIMER0_IRQn);
//NVIC_EnableIRQ(TIMER1_IRQn);
//Run at 2x necessary frequency
//Start at 378 matches/sec (2x50%)
T0MR0 |= fullspeed;
T1MR0 |= fullspeed;
	
//Set Match Count to 0
T0MC = 0; T1MC = 0;
//Start timer
T0TCR |= 1; T1TCR |= 1;
}

void putchar(unsigned char c)
{
  // wait for tx ready flag
  while((U2LSR & (1<<UART_THRE)) == 0);
  // load character
  U2THR = c;   
}

// prints characters from start up to, not including, end
void putstring(const char *s, unsigned char start, unsigned char end)
{
  if(start >= end)
    return;
  
  for(int i = start; i < end; i++)
    putchar(s[i]);
}

/* 
  -takes an n-digit number and converts each digit to a char
  -size indicates the size of the array
  -note that a pointer to the array is passed, not the array itself
*/
void intToChar(char *arr, int num, int size)
{
    // fill character array with zeroes
    for(int i = 0; i < size; i++) {
        *arr = NUMBASE;
        arr++;
    }
    arr--; // reset pointer to end of array
    
    int rem = num % 10;
    num /= 10;
    // each digit is transformed to char in reverse order
    for(int i = 0; i < size; i++) {
        *arr = rem + NUMBASE;
        arr--;
        rem = num % 10;
        num /= 10;
    }
}


void delay(int x)
{
	for(int i=0;i<x;i++){
		for(int j=0;j<x;j++){
		}
	}
}

void changeState(void)
{
	// 3 states: Start, Running, Stopped
	// 1 == Start
	// 2 == Running
	// 3 == Stopped
	if(state==1)
		state = 2;
	else
		state = 3;
}

void menuDisplay(void)
{
	intToChar(str, distance, 3);
	putchar(LCD_CMD_PREFIX);
  putchar(LCD_CLR);
	putstring("SET DISTANCE - +", 0, 16);
	putchar(LCD_CMD_PREFIX);
	putchar(LCD_CURR_SET);
	putchar(0x40);
	if(feet) 
		putstring("Feet: ", 0, 6);
	else
		putstring("Meters: ", 0, 8);
	putstring(str,0,3);
}

void feetToMeters(void)
{
	distance = ceil(distance*0.3048);
	if(distance < 1)
		distance = 1;
}

void metersToFeet(void)
{
	distance = ceil(distance/0.3048);
}

void startOperation(void)
{
	putchar(LCD_CMD_PREFIX);
  putchar(LCD_CLR);
	putstring("Running. . .", 0, 12);
}

void operationStopped(void)
{
	putchar(LCD_CMD_PREFIX);
  putchar(LCD_DISP_ON);
	putchar(LCD_CMD_PREFIX);
  putchar(LCD_CLR);
	putstring("Operation halted", 0, 16);
}

void powerScreenDown(void)
{
	putchar(LCD_CMD_PREFIX);
  putchar(LCD_DISP_OFF);
}


// ISR for button timer
extern "C" {
  void SysTick_Handler(void)
  {
    // read button states
    btn1state = (FIO0PIN1>>BTN1_PIN) & 0x01;
    btn2state = (FIO0PIN1>>BTN2_PIN) & 0x01;
		btn3state = (FIO0PIN0>>BTN3_PIN) & 0x01;
    btn4state = (FIO0PIN0>>BTN4_PIN) & 0x01;
    
    // debounce button 1
    if(btn1state == 0) {
      btn1counter = 0;
      btn1pressed = 0;
    }
    else 
      btn1counter++;
    if(btn1counter == DEBOUNCE_MS)
      btn1pressed = 1;
    
    // debounce button 2
    if(btn2state == 0) {
      btn2counter = 0;
      btn2pressed = 0;
    }
    else 
      btn2counter++;
    if(btn2counter == DEBOUNCE_MS)
      btn2pressed = 1;
		
		// debounce button 3
    if(btn3state == 0) {
      btn3counter = 0;
      btn3pressed = 0;
    }
    else 
      btn3counter++;
    if(btn3counter == DEBOUNCE_MS)
      btn3pressed = 1;
    
    // debounce button 4
    if(btn4state == 0) {
      btn4counter = 0;
      btn4pressed = 0;
    }
    else 
      btn4counter++;
    if(btn3counter == DEBOUNCE_MS)
      btn4pressed = 1;
  }
}
  
//ISRs for Timer0/Timer1
void TIMER0_IRQHandler(void){
	//Clear by writing 1 to 0x4000 4000
	T0TCR = 0;
	T0IR = 1;
	if(m1State == 0)
	{
		FIO2SET0 = (1<<M1_PIN);
	}
	if(m1State == 1)
	{
		FIO2CLR0 = (1<<M1_PIN);
	}
	T0MC++;
	m1State = !m1State;
	T0TCR = 1;
}

void TIMER1_IRQHandler(void){
	//Clear by writing 1 to 0x4000 8000
	T1TCR = 0;
	T1IR = 1;
	if(m2State == 0)
	{
		FIO2SET0 = (1<<M2_PIN);
	}
	if(m2State == 1)
	{
		FIO2CLR0 = (1<<M2_PIN);
	}
	T1MC++;
	m2State = !m2State;
	T1TCR = 1;
 }

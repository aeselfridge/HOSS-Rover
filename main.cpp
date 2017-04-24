/*
 * AUTHOR:        Alec Selfridge
 * FILENAME:      main.cpp
 * VERSION:       1.0
 * PROJECT:       HOSS Rover
 * DATE CREATED:  02/02/2017
 * LAST MODIFIED: 02/10/2017
 */
#include <LPC17xx.h>
#include "SPLC780D.h"
#include "LPC1768regmap.h"
#include "SHARP_IR.h"

// buttons & debouncing parameters
#define BTN1_PIN           6
#define BTN2_PIN           7
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
// motor parameters
#define M1_PIN             4 //P2.4
#define M2_PIN             3 //P2.3
#define PI                 3.14159
#define sqrt2              1.41421
#define halfspeed          0x3f69c

void initGPIO(void);
void initSysTick(void);
void initUART2(unsigned long baud);
void initLCD(void);
void initMotor(void);
void initTimers(void);
void putchar(unsigned char c);
void putstring(const char *s, unsigned char start, unsigned char end);

SHARP_IR ir1 = SHARP_IR(5); // IR sensor on ADC0.5
SHARP_IR ir2 = SHARP_IR(4); // IR sensor on ADC0.4

int btn1state   = 0;
int btn2state   = 0;
int btn1counter = 0;
int btn2counter = 0;
int btn1pressed = 0;
int btn2pressed = 0;

//motor control
int m1State = 0;
int m2State = 0;
int T0MC, T1MC;
int stepMax;
//radius of wheel and pipe in corresponding units (metric or imperial)
float rWheel = 1.08;
float pipeArc, stepArc;
int temp = 0;

int main(void)
{
	SystemInit();
  int temp = 0;
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
	stepArc = ((2*PI*rWheel*(1.8/360.0)));//Calculate single step arc
	stepMax = 2.5/stepArc; //Calculate x degree step count, floored to fall short instead of step too far
  while(1)
  {
    // read IR sensor
    /*ir1.update();
    ir1_dist = ir1.getDistance();
		ir2.update();
    ir2_dist = ir2.getDistance();*/
    
    // check buttons
    if(btn1pressed == 1) {
      btn1pressed = 0;
      temp++;
    }
    if(btn2pressed == 1) {
      btn2pressed = 0;
      temp--;
    }
		while((T0MC < stepMax) && (T1MC < stepMax)){
			
		};
		T0MC = 0;
		T1MC = 0;
		T0TCR = 0; T1TCR = 0;
		ir1.update();
    ir1_dist = ir1.getDistance();
    if(ir1_dist == -1)
      ir1_dist = 10;
		ir2.update();
    ir2_dist = ir2.getDistance();
    if(ir2_dist == -1)
      ir2_dist = 10;
		speedMult1 = (ir1_dist/ir2_dist);
		speedMult2 = (ir2_dist/ir1_dist);
		if(speedMult1 >= 2.0)
			speedMult1 = 2.0;
		if(speedMult2 >= 2.0)
			speedMult2 = 2.0;
		if(speedMult1 <= 0.0125)
			speedMult1 = 0.0125;
		if(speedMult2 <= 0.0125)
			speedMult2 = 0.0125;
		T0MR0 = speedMult1*halfspeed;
		T1MR0 = speedMult2*halfspeed;
		T0TCR = 1; T1TCR = 1;
	}
}

void initGPIO(void)
{
  // configure pin as GPIO
  PINSEL0 = 0;
  // configure buttons as inputs
  FIO0DIR0 |= (0<<BTN1_PIN);
  FIO0DIR0 |= (0<<BTN2_PIN);
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
  putstring("HOSS Rover V1.0", 0, 15);
}

void initMotor(void){
	// configure as GPIO
	PINSEL4 = 0x00000000;
	// configure motor pins as outputs
	FIO2DIR0 |= (1<<M1_PIN);
  FIO2DIR0 |= (1<<M2_PIN);
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
T0MCR |= 3;  T1MCR |= 3;
//T0EMR |= 0x00c0;	T1EMR |= 0x00c0;

NVIC_EnableIRQ(TIMER0_IRQn);
NVIC_EnableIRQ(TIMER1_IRQn);
//Run at 2x necessary frequency
//Start at 378 matches/sec (2x50%)
T0MR0 |= halfspeed;
T1MR0 |= halfspeed;
	
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

// ISR for button timer
extern "C" {
  void SysTick_Handler(void)
  {
    // read button states
    btn1state = (FIO0PIN0>>BTN1_PIN) & 0x01;
    btn2state = (FIO0PIN0>>BTN2_PIN) & 0x01;
    
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
}

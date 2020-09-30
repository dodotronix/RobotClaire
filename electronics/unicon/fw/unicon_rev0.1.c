//FIXME add licence!

/* unicon.c
 * 
 * DATE:  Wed Feb 10 16:04:02 CET 2016
 * AUTHOR: PETR PACNER
 * Rev: 0.1
 *
 * DESCRIPTION:
 * 
 * The main purpose of this program is controlling robot movemets board.
 * Is't upload in flash memory of atxmega64, which is the brain of this board.
 * This program contains different modes for distinctive robot behavior.
*/

#define F_CPU 32000000UL //Frequency 32MHz
#define Prescaler TC_CLKSEL_DIV64_gc //prescaler 8

//period equaton: fclk/(prescaler*frq-1)
#define PWM_PER (32000000/(3150)) //signal for motors [50Hz]
#define MIDPOSITION 745 // 1.5ms signal middle position
#define OPT_SPEED 680

/*LIBRARIES & HEADERS*/
#include <avr/io.h> //MACROS
#include <avr/interrupt.h> //INTERRUPT settings
#include <util/delay.h> //DELAY_MS()
#include <stdio.h>

/*GLOBAL VARIABLES*/
volatile int enk_val [] = {0, 0}; //encoders current value [enk0, enk1]
volatile int enk_val_ls [] = {0, 0}; //encoders last value
volatile int curr_spd [] = {0, 0};
volatile int last_spd [] = {0, 0};
uint8_t mode = 0;

//volatile uint8_t *pReg = (volatile uint8_t *) &PORTE_IN;

/*INTERN OSCILATOR*/
//2MHz
void mcu_clk2_init(){
  OSC.CTRL |= OSC_RC2MEN_bm; //set 2MHz oscilator
  while(!(OSC.STATUS & OSC_RC2MRDY_bm)); //wait for oscilator
  CCP = CCP_IOREG_gc;
  CLK.CTRL = (CLK.CTRL & ~CLK_SCLKSEL_gm) | CLK_SCLKSEL_RC32M_gc; //activete oscilator
}

//32MHz
void mcu_clk32_init(){
  OSC.CTRL |= OSC_RC32MEN_bm; //set 32MHz oscilator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); //wait for oscilator to be prepared
  CCP = CCP_IOREG_gc;
  CLK.CTRL = (CLK.CTRL & ~CLK_SCLKSEL_gm) | CLK_SCLKSEL_RC32M_gc;
}

/*TWI_INTERFACE*/
void twi_init(){
TWID_MASTER_BAUD = 0x10;
  TWID_MASTER_CTRLA = TWI_MASTER_ENABLE_bm;
  TWID_MASTER_STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;  
}

/*SINGLE_SLOPE_GENERATION*/
//motor0 - D4; motor1 - E3

//TIMER1 PORTD
void motor0_init(){
  PORTD.DIR |= (1<<PIN4); //set pin as output
  TCD1.CTRLB = TC_WGMODE_SS_gc|TC1_CCAEN_bm; //set single slope mode 
  TCD1.INTCTRLA = TC_OVFINTLVL_HI_gc;
  TCD1.PER = PWM_PER; //period
  TCD1.CTRLA = Prescaler; //prescaler
}

//speed meassure time - messuring speed with 50Hz
ISR (TCD1_OVF_vect){
  last_spd[0] = curr_spd[0]; //save curr_speed value as last_speed value
  curr_spd[0] = enk_val[0] - enk_val_ls[0]; //encoders values difference
  enk_val_ls[0] = enk_val[0]; //actual encoder value is now last value
  //PORTA.OUTSET = (1<<PIN5);
}

//TIMER0 PORTE
void motor1_init(){
  PORTE.DIR |= (1<<PIN3); //set pin as output
  TCE0.CTRLB = TC_WGMODE_SS_gc|TC0_CCDEN_bm; //set single slope mode
  TCE0.INTCTRLA = TC_OVFINTLVL_HI_gc;
  TCE0.PER = PWM_PER; //period
  TCE0.CTRLA = Prescaler; //prescaler
}

//speed messure time - messuring speed with 50Hz
ISR (TCE0_OVF_vect){
  last_spd[1] = curr_spd[1]; //save curr speed value as last speed value
  curr_spd[1] = enk_val[1] - enk_val_ls[1]; //encoders value difference == new speed
  enk_val_ls[1] = enk_val[1]; // save curr value in value last
  //TCE0.CCD += (4-curr_spd[1])-(curr_spd[1]-last_spd[1]);

  //PORTA.OUTSET = (1<<PIN6);
}

/*SET PWM FOR MOTORS*/
//pulswidth between 1ms - 2ms (508 - 1016)
//null point for sabertooth == 745
//increase value --> revers move
//decrease value --> forward move
void set_motors(int m0, int m1){
  TCD1.CCA = m0;
  TCE0.CCD = m1;
}

//TODO 
/*MODE INTERRUPT*/
//interrupt - C3
void interr_init(){
  PORTC.INT0MASK = (1<<PIN3);
  PORTC.PIN3CTRL = PORT_ISC_FALLING_gc;
  PORTC.INTCTRL |= PORT_INT0LVL_LO_gc;
  PMIC.CTRL |= PMIC_HILVLEN_bm |PMIC_MEDLVLEN_bm|PMIC_LOLVLEN_bm;
  sei();
}

ISR(PORTC_INT0_vect){
  if(mode == 0){
    mode +=1;
    PORTA.OUTSET |= PIN5_bm;
    set_motors(760, 760);
  }
  else{
    mode -= 1;
    PORTA.OUTCLR |= (1<<PIN5);
    set_motors(MIDPOSITION, MIDPOSITION);
  }
}

/*ENCODER INTERRUPTS*/
void enk0_init(){
  PORTD.INT0MASK = (1<<PIN2);
  PORTD.PIN2CTRL = PORT_ISC_RISING_gc;
  PORTD.INTCTRL |= PORT_INT0LVL_HI_gc;
  PMIC.CTRL |= PMIC_HILVLEN_bm |PMIC_MEDLVLEN_bm|PMIC_LOLVLEN_bm;
  sei();
}

ISR(PORTD_INT0_vect){
    //current statement of PIN3
    if((PORTD_IN &= PIN3_bm) == 0){
      enk_val[0] +=1;
      //PORTA.OUTSET = (1<<PIN6);
    }
    else{
      enk_val[0] -=1;
      //PORTA.OUTCLR = (1<<PIN6);
    }
}

void enk1_init(){
  PORTE.INT0MASK = (1<<PIN2);
  PORTE.PIN2CTRL = PORT_ISC_RISING_gc;
  PORTE.INTCTRL |= PORT_INT0LVL_HI_gc;
  PMIC.CTRL |= PMIC_HILVLEN_bm |PMIC_MEDLVLEN_bm|PMIC_LOLVLEN_bm;
  sei();
}

ISR(PORTE_INT0_vect){
  if((PORTE_IN &= PIN4_bm) == 0){
    enk_val[1] -=1;
    //PORTA.OUTCLR = (1<<PIN6);
  }
  else{
    enk_val[1] +=1;
    //PORTA.OUTSET = (1<<PIN6);
  }
}

void motor_regul(int req, volatile int curr[2], volatile int last[2], volatile int pulse[1]){
  pulse[0] += (req - curr[1])-(curr[1] - last[1]);
  //pulse += 30-curr_spd[1]-curr_spd[1]-last_spd[1];
}

int main(){
  /*LOCAL VARIABLES*/
  //volatile int pulse[] = {MIDPOSITION};

  /*INITIALIZATION*/
  mcu_clk32_init();
  PORTA.DIRSET = (1<<PIN4) | (1<<PIN5) | (1<<PIN6); //initialize LEDs (A4; A5; A6)
  interr_init();
  enk0_init();
  enk1_init();
  motor0_init();
  motor1_init();
  set_motors(MIDPOSITION, MIDPOSITION);

  while(1){   
    //motor_regul(680, curr_spd, last_spd, pulse);
    //pulse[0] += (40-curr_spd[1])-(curr_spd[1]-last_spd[1]);
    //set_motors(MIDPOSITION, 680);
    //set_motors(MIDPOSITION, pulse[0]);
    //if(pulse[0] == 680) PORTA.OUTSET = (1<<PIN6);
    //else PORTA.OUTCLR = (1<<PIN6);
      //if(last_spd[1]>5) PORTA.OUTSET = (1<<PIN6);
      //if(curr_spd[1]>5) PORTA.OUTSET = (1<<PIN4);
     //while(enk_val_ls[1]>1000) PORTA.OUTSET = (1<<PIN4);
     //while(enk_val[1]>1000) PORTA.OUTSET = (1<<PIN6);
     //set_motors(OPT_SPEED, MIDPOSITION);
  }
   
  return 0;
}

//FIXME add licence!

/* pid_xmeg.c
 * 
 * DATE:  Wed Feb 10 16:04:02 CET 2016
 * AUTHOR: PETR PACNER
 * Rev: 0.1
 *
 * DESCRIPTION:
 * THis program should be use to generate and setup curve of pid equation on the ATxmega
 *
 * WARNING
 * if you want to edit this code, make sure you didn't write an unsupported symbol (maybe a space)
 * After compile you get ERROR stray \'302\
 *
 * NOTES
 * NULL diameter sabertooth (730 - 755)
 * motor0 - PD regulator -> P=1;I=0;D=16
 * motor1 - PD regulator -> P=1/4;I=0;D=10
*/

#define F_CPU 32000000UL //Frequency 32MHz
#define Prescaler TC_CLKSEL_DIV64_gc //prescaler 64

//period equaton: fclk/(prescaler*frq-1)
#define PWM_PER (32000000/(3150)) //signal for motors [50Hz]
#define MIDPOSITION 745 // 1.5ms signal middle position for motors [width = MIDPOSITION +- 347]
#define MIDUP 755
#define MIDDOWN 730


/*LIBRARIES & HEADERS*/
#include <avr/io.h> //MACROS
#include <avr/interrupt.h> //INTERRUPT settings
#include <util/delay.h> //DELAY_MS()
#include <stdio.h>

/*GLOBAL VARIABLES*/
volatile int enk_val [] ={0, 0}; //encoders current value [enk0, enk1]
volatile int enk_val_ls [] ={0, 0}; //encoders last value
volatile int curr_spd [] ={0, 0}; //current speed array
volatile int last_spd [] ={0, 0}; //4 ticks / 20ms
volatile int req_spd [] ={0, 0}; //required speed, this variable change speed of robot
volatile int e_sum[] ={0, 0}; //integration part of regulator
volatile uint8_t front[] ={0, 0}; //array for front proximity sensor output values
volatile int e[] ={0, 0}; //speed difference
volatile int e_der[] ={0, 0};
volatile int e_enk[] ={0, 0};
uint8_t mode =0;
volatile uint8_t pulse[] ={0, 0};

static volatile int16_t pGain, iGain, dGain;
//static const int8_t dGain = 67;//134
static volatile int16_t iState[] ={0, 0};
static volatile int16_t dState[] ={0, 0};
static const int16_t iMax = 400;
static const int16_t iMin = -400;
volatile int16_t position =0;
volatile int16_t pos[] ={0, 0};

/*POINTERS TO REGISTERS*/
volatile uint8_t *twi_slv = (volatile uint8_t *) &TWIC_SLAVE_STATUS;
volatile uint8_t *twi_data = (volatile uint8_t *) &TWIC_SLAVE_DATA;
volatile uint16_t *motor0 = (volatile uint16_t *) &TCD1.CCA; //motor0
volatile uint16_t *motor1 = (volatile uint16_t *) &TCE0.CCD; //motor1
volatile uint8_t *enk0 = (volatile uint8_t *) &PORTD_IN; //PORTD STATUS
volatile uint8_t *enk1 = (volatile uint8_t *) &PORTE_IN; //PORTE STATUS
volatile uint8_t *modept = &mode;

/*INTERN OSCILATOR*/
//32MHz
void mcu_clk32_init(){
  OSC.CTRL |= OSC_RC32MEN_bm; //set 32MHz oscilator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); //wait for oscilator to be prepared
  CCP = CCP_IOREG_gc;
  CLK.CTRL = (CLK.CTRL & ~CLK_SCLKSEL_gm) | CLK_SCLKSEL_RC32M_gc;
}

void led_init(){
  PORTA.DIRSET = (1<<PIN4) | (1<<PIN5) | (1<<PIN6); //initialize LEDs (A4; A5; A6)
}

/*TWI_INTERFACE*/
void twi_init_slv(){
  TWIC_SLAVE_ADDR = 0x65; //slave address 0x18
  TWIC_SLAVE_CTRLA = TWI_SLAVE_ENABLE_bm; //enable twi slave device
}

void rpi_com(){
  uint8_t buffer[] ={0, 0};
  while(!(*twi_slv & TWI_SLAVE_APIF_bm)); //wait for master action
  //write
  if((*twi_slv & TWI_SLAVE_DIR_bm) ==0){
    cli();
    TWIC_SLAVE_CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; //send back address
    //waiting for 7bytes
    for(int i =0; i<4; ++i){
      cli();
      while(!(*twi_slv & TWI_SLAVE_DIF_bm));
      buffer[0] = TWIC_SLAVE_DATA; //save received value
      TWIC_SLAVE_CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; //send data
      //if(buffer[0] !=0) PORTA.OUTSET = (1<<PIN4);

      while(!(*twi_slv & TWI_SLAVE_DIF_bm));
      buffer[1] = TWIC_SLAVE_DATA; //save received value
      //if(buffer[1] ==220) PORTA.OUTSET = (1<<PIN4);
      if(i != 3)
        TWIC_SLAVE_CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; //send data
      else
        TWIC_SLAVE_CTRLB = TWI_SLAVE_CMD_COMPTRANS_gc;
      //sei();
      
      switch(i){
      case 0:
          pGain =(buffer[0] <<8)|buffer[1];
          //PORTA.OUTSET = (1<<PIN4);
      case 1:
          iGain =(buffer[0] <<8)|buffer[1];
      case 2:
          dGain =(buffer[0] <<8)|buffer[1];
      case 3:
          position =(buffer[0] <<8)|buffer[1];
      }
    }
    sei();
  }
  //read
  else if((*twi_slv & TWI_SLAVE_DIR_bm) !=0){ //do you want to transmitt more bytes   
    cli();
    TWIC_SLAVE_CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; //ACK after address recieved
    while(!(*twi_slv & TWI_SLAVE_DIF_bm));
    *twi_data = pulse[0];
    TWIC_SLAVE_CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; //send data
    while(!(*twi_slv & TWI_SLAVE_DIF_bm));
    *twi_data = pulse[1];
    TWIC_SLAVE_CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; //send data
    while(!(*twi_slv & TWI_SLAVE_DIF_bm));
    TWIC_SLAVE_CTRLB = TWI_SLAVE_CMD_COMPTRANS_gc | 0x04; //send NACK
    sei();
  }
}

//split number into two bytes and return the byte on submited position
void split_num(volatile int num, volatile uint8_t array[2])
{
  //uint16_t num = numpt;
  array[0] = (num >>8) & 0xff;
  array[1] = 0xff & num;
}

uint16_t pid0(int16_t r){
  volatile int16_t y = enk_val[0];
  volatile int32_t u;
  volatile int32_t pTerm;
  volatile int32_t dTerm;
  volatile int32_t iTerm;
  volatile int32_t e = r - y;

  iState[0] += e; 
  if (iState[0] > iMax)
    iState[0] = iMax;
  else if (iState[0] < iMin)
    iState[0] = iMin;

  pTerm = e;
  iTerm = 0;//(iGain*iState[0]);
  dTerm = (16*(y - dState[0]));
  dState[0] = y;

  u = pTerm +iTerm -dTerm;
  //split_num(u, pulse); 
  
  if(u >0)//move null point up?
    u += MIDUP;
  else if(u ==0)
    u += MIDPOSITION;
  else if(u <0)//move null point down
    u += MIDDOWN;

  if(u > 1080)//MAX supported motor pulse
  //if(u > 804)
      u = 1080;
  //else if(u < 581)
  else if (u < 406)//MIN supported motor pulse
      u = 406;
  return (uint16_t) u;
}

uint16_t pid1(int16_t r){
  volatile int16_t y = enk_val[1];
  volatile int32_t u;
  volatile int32_t pTerm;
  volatile int32_t dTerm;
  volatile int32_t iTerm;
  volatile int32_t e = r - y;

  iState[1] += e; 
  if (iState[1] > iMax)
    iState[1] = iMax;
  else if (iState[1] < iMin)
    iState[1] = iMin;

  pTerm = e/4;
  iTerm = 0; //(iGain*iState[1]);
  dTerm = (10*(y - dState[1]));
  dState[1] = y;

  u = pTerm +iTerm -dTerm;
  //split_num(u, pulse); 
  
  if(u >0)//move null point up?
    u += MIDUP;
  else if(u ==0)
    u += MIDPOSITION;
  else if(u <0)//move null point down
    u += MIDDOWN;

  if(u > 1080)//MAX supported motor pulse
  //if(u > 804)
      u = 1080;
  //else if(u < 581)
  else if (u < 406)//MIN supported motor pulse
      u = 406;
  return (uint16_t) u;
}

/*SINGLE_SLOPE_GENERATION*/
//motor0 - D4; motor1 - E3
//TIMER1 PORTD
void motor0_init(){
  PORTD.DIR |= (1<<PIN4); //set pin as output
  TCD1.CTRLB = TC_WGMODE_SS_gc|TC1_CCAEN_bm; //set single slope mode 
  //TCD1.INTCTRLA = TC_OVFINTLVL_HI_gc;
  TCD1.PER = PWM_PER; //period
  TCD1.CTRLA = Prescaler; //prescaler
  *motor0 = MIDPOSITION;
}

//TIMER0 PORTE
void motor1_init(){
  PORTE.DIR |= (1<<PIN3); //set pin as output
  TCE0.CTRLB = TC_WGMODE_SS_gc|TC0_CCDEN_bm; //set single slope mode
  TCE0.PER = PWM_PER; //period
  TCE0.CTRLA = Prescaler; //prescaler
  *motor1 = MIDPOSITION; //set motor in middle position
  /*SET PWM FOR MOTORS*/
  //pulswidth between (396 - 1090)
  //null point for sabertooth == 743
  //increase value --> revers move
  //decrease value --> forward move
}

void motor_ctrl(){
  TCD0.CTRLB = TC_WGMODE_SS_gc; //set single slope mode 
  TCD0.INTCTRLA = TC_OVFINTLVL_HI_gc;
  TCD0.PER = 499; //1kHz
  TCD0.CTRLA = Prescaler; //prescaler TC_CLKSEL_DIV8_gc;
}

ISR(TCD0_OVF_vect){
  if(*modept ==1){
    *motor1 = pid1(pos[1]);
    *motor0 = pid0(pos[0]);
  }
  //split_num(enk_val[0], pulse);
}

/*MODE INTERRUPT*/
//interrupt - C3
void mode_init(){
  PORTC.INT0MASK = (1<<PIN3);
  PORTC.PIN3CTRL = PORT_ISC_FALLING_gc;
  PORTC.INTCTRL |= PORT_INT0LVL_HI_gc;
  PMIC.CTRL |= PMIC_HILVLEN_bm |PMIC_MEDLVLEN_bm|PMIC_LOLVLEN_bm;
}
//swich mode
ISR(PORTC_INT0_vect){
  if(*modept == 1) 
    *modept =0;
  else
    *modept +=1;
}  

/*ENCODER INTERRUPTS*/
void enk0_init(){
  PORTD.INT0MASK = (1<<PIN2);
  PORTD.PIN2CTRL = PORT_ISC_BOTHEDGES_gc;
  PORTD.INTCTRL |= PORT_INT0LVL_HI_gc;
  PMIC.CTRL |= PMIC_HILVLEN_bm |PMIC_MEDLVLEN_bm|PMIC_LOLVLEN_bm;
}

ISR(PORTD_INT0_vect){
    //current statement of PIN3
    if((*enk0 & PIN2_bm) ==0){
      if((*enk0 & PIN3_bm) ==0){
        cli();
        enk_val[0] -=1;
        sei();
      }
      else{
        cli();
        enk_val[0] +=1;
        sei();
      }
    }
    else{
      if((*enk0 & PIN3_bm) ==0){
        cli();
        enk_val[0] +=1;
        sei();
      }
      else{
        cli();
        enk_val[0] -=1;
        sei();
      }
   }
    /*
   //overflow protection
   if(enk_val[0] ==32500 || enk_val[0] ==-32500){
     enk_val[0] =enk_val[0]-enk_val_ls[0];
     enk_val_ls[0] =0;
   }
   */
}

void enk1_init(){
  PORTE.INT0MASK = (1<<PIN2);
  PORTE.PIN2CTRL = PORT_ISC_BOTHEDGES_gc;
  PORTE.INTCTRL |= PORT_INT0LVL_HI_gc;
  PMIC.CTRL |= PMIC_HILVLEN_bm |PMIC_MEDLVLEN_bm|PMIC_LOLVLEN_bm;
}

ISR(PORTE_INT0_vect){
  //PIN4
  if((*enk1 & PIN2_bm) ==0){
    if((*enk1 & PIN4_bm) ==0){
      cli();
      enk_val[1] +=1;
      sei();
    }
    else{
      cli();
      enk_val[1] -=1;
      sei();
    }
  }
  else{
    if((*enk1 & PIN4_bm) ==0){
      cli();
      enk_val[1] -=1;
      sei();
    }
    else{
      cli();
      enk_val[1] +=1;
      sei();
    }
  }
}

void turn(int16_t tics){
  for(int i =0; i<=tics; ++i){
    pos[0] +=1;
    pos[1] -=1;
    _delay_ms(2);
  }
}

int main(){ 
  /*LOCAL VARIABLES*/

  /*INITIALIZATION*/
  mcu_clk32_init(); //initilize 32MHz intern oscilator
  led_init(); //initialize control LED connected to ATxmega64
  //twi_init_slv(); //slave device initialization

  /*INTERRUPTS_INITIALIZE*/
  mode_init(); //initialize mode button interrupt
  enk0_init(); // -||- encoder0 (motor0)
  enk1_init(); // -||- encoder1 (motor1)

  /*TIMERS_INITIALIZE*/
  motor0_init();
  motor1_init();
  motor_ctrl();

  sei(); //TURN ON ALL INTERRUPTS
  
  while(1){
    switch (*modept){
      case 0:
        PORTA.OUTCLR = (1<<PIN5) | (1<<PIN6);
        while(*modept ==0);

      case 1:
        PORTA.OUTCLR = (1<<PIN5) | (1<<PIN6);
        PORTA.OUTSET = (1<<PIN6);
        turn(1000);

        while(*modept ==1){
          if(enk_val[0] ==1000)
            PORTA.OUTSET = (1<<PIN4);
          else
            PORTA.OUTCLR = (1<<PIN4);

          //rpi_com();
        }
    }
  }
  return 0;
}

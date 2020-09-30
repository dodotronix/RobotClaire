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
 *
 * WARNING
 * if you want to edit this code, make sure you didn't write an unsupported symbol (maybe a space)
 * After compile you get ERROR stray \'302\
*/

#define F_CPU 32000000UL //Frequency 32MHz
#define Prescaler TC_CLKSEL_DIV64_gc //prescaler 8

//period equaton: fclk/(prescaler*frq-1)
#define PWM_PER (32000000/(3150)) //signal for motors [50Hz]
#define MIDPOSITION 745 // 1.5ms signal middle position
#define OPT_SPEED 4 //4 ticks / 20ms

//I2C device addresses
#define FRONT0 0x29
//#define FRONT1
//#define RIGHT
//#define LEFT
//#define BACK
#define COMPASS 0x1e
#define LED_DVR 0x60

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

/*POINTERS TO REGISTERS*/
volatile uint8_t *twi_stat = (volatile uint8_t *) &TWID_MASTER_STATUS;

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
  PORTD.PIN0CTRL |= PORT_OPC_WIREDANDPULL_gc;
  PORTD.PIN1CTRL |= PORT_OPC_WIREDANDPULL_gc;
  TWID_MASTER_BAUD = 0xff;
  TWID_MASTER_CTRLA = TWI_MASTER_ENABLE_bm ; // Enable TWI interface 
  TWID_MASTER_STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;  //set bus state to IDLE mode
}

void led_init(){
  PORTA.DIRSET = (1<<PIN4) | (1<<PIN5) | (1<<PIN6); //initialize LEDs (A4; A5; A6)
}

void sensor_init(){
  PORTH.DIRSET = (1<<PIN0) | (1<<PIN1) | (1<<PIN2) | (1<<PIN3) | (1<<PIN4) | (1<<PIN5); //initialize port for turn on/off sensors
  //set pull-ups on these pins

  PORTH.PIN0CTRL |= PORT_OPC_WIREDANDPULL_gc;
  PORTH.PIN1CTRL |= PORT_OPC_WIREDANDPULL_gc;
  PORTH.PIN2CTRL |= PORT_OPC_WIREDANDPULL_gc;
  PORTH.PIN3CTRL |= PORT_OPC_WIREDANDPULL_gc;
  PORTH.PIN4CTRL |= PORT_OPC_WIREDANDPULL_gc;
  PORTH.PIN5CTRL |= PORT_OPC_WIREDANDPULL_gc;


  /*
   * PIN0 - sensor BACK
   * PIN1 - sensor LEFT
   * PIN2 - sensor FRONT1
   * PIN3 - compass speed_up function
   * PIN4 - sensor FRONT0 (from the right side)
   * PIN5 - sensor RIGHT
  */
}

uint8_t dev_check(uint8_t addr){
  TWID_MASTER_ADDR = (addr <<1);
  while(!(*twi_stat & TWI_MASTER_WIF_bm)); //wait for write flag is set?
  if((*twi_stat & TWI_MASTER_RXACK_bm) == TWI_MASTER_RXACK_bm){
    TWID_MASTER_CTRLC = 0x03;
    //TWID_MASTER_STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;  //set bus state to IDLE mode
    return 1;
  }
  else {
    TWID_MASTER_CTRLC = 0x03;
    //TWID_MASTER_STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;  //set bus state to IDLE mode
    return 0;
  }
}

//write number 
//if the transfer go wrong - return number is not equal to variable num
uint8_t twi_write(uint8_t device, uint8_t buff[3], uint8_t num){
  uint8_t i = 0;
  TWID_MASTER_ADDR = (device <<1);

  for(; i<num;){
    while(!(*twi_stat & TWI_MASTER_WIF_bm)); //wait for write flag is set?
    if((*twi_stat & TWI_MASTER_RXACK_bm) == TWI_MASTER_RXACK_bm) return i;
    else if((*twi_stat & TWI_MASTER_RXACK_bm) == 0){
      TWID_MASTER_DATA = buff[i];
      ++i;
      //while(!(*twi_stat & TWI_MASTER_WIF_bm));
    }
    else{
      while(!(*twi_stat & TWI_MASTER_BUSSTATE_IDLE_gc)); //wait for TWI IDLE statement
      i = 0; //set variable i to NULL 
      TWID_MASTER_ADDR = (device <<1); //send address again
    }
  }
  while(!(*twi_stat & TWI_MASTER_WIF_bm)); //wait for write flag is set?
  TWID_MASTER_CTRLC = 0x03;
  //TWID_MASTER_STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;  //set bus state to IDLE mode

  return i;
}

uint8_t twi_read(uint8_t device, uint8_t buff[2], uint8_t num){
  int i = 0;
  TWID_MASTER_ADDR = (device <<1)|0x01; //send address with READ bit
  for(; i<num;){
    while(!(*twi_stat & TWI_MASTER_RIF_bm)); //wait for write flag is set?
    if((*twi_stat & TWI_MASTER_RXACK_bm) == 0){
      buff[i] = TWID_MASTER_DATA; //save received data
      ++i; //number off received bytes
      if(i < (num-1)) TWID_MASTER_CTRLC = 0x00; //send ACK
      else TWID_MASTER_CTRLC = 0x04 | 0x03; //send NACK and STOP bit
    }
    else return i;
  } 
  return i;
}

//special function for writting into & reading from sensor registers
void vlx_write(uint8_t device, uint16_t reg, uint8_t value){
  uint8_t data[3];
  data[0] = (reg >> 8) & 0xff; // reg high byte
  data[1] = reg & 0xff; // reg low byte
  data[2] = value;
  if((twi_write(device, data, 3)) !=3) PORTA.OUTSET |= (1<<PIN5);
}

//TODO read 16-bit value from AMBIENT_SENSOR - variable value
uint8_t vlx_read(uint8_t device, uint16_t reg){
  uint8_t value[] = {0};
  uint8_t data[2];
  data[0] = (reg >> 8) & 0xff; // reg high byte
  data[1] = reg & 0xff; // reg low byte
  if(twi_write(device, data, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
  else{
    if(twi_read(device, value, 1) !=1) PORTA.OUTSET |= (1<<PIN5);
  }
  return value[0]; 
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
    PORTA.OUTSET |= PIN4_bm;
    PORTH.OUTSET |= (1<<PIN0);
    //set_motors(760, 760);
  }
  else{
    mode -= 1;
    PORTA.OUTCLR |= (1<<PIN4);
    PORTH.OUTCLR |= (1<<PIN0);
    
    //set_motors(MIDPOSITION, MIDPOSITION);
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
//change sensor addres from 0x24 to 0x28 ( 
void add_addr(){
  uint8_t addr = 36;
   for(uint8_t i = 0; i<6;){
    if(i != 3){
      PORTH.OUTSET |= (1<<i);
      if(dev_check(0x29) ==0){
        vlx_write(0x29, 0x212, addr);
      }
      else PORTH.OUTCLR = (1<<i);
      ++addr;
    }
    ++i;
  }
}


/*
void vlx_init(uint8_t device){
  while(1){
    vlx_write(device, 0x010, 0x01);
    _delay_ms(1); //wait for 1ms [device boot]
    if(vlx_read
  }

}
*/
    
int main(){
  /*LOCAL VARIABLES*/
  //uint8_t buff[] = {0x06, 0x15};
  //uint8_t buffer[] = {0x000, 0x000};
  //uint8_t sens[] = {0};
  //uint8_t addr = 36;


  /*INITIALIZATION*/
  //mcu_clk32_init();
  twi_init();
  led_init();
  sensor_init();
  //interr_init();
  //enk0_init();
  //enk1_init();
  //motor0_init();
  //motor1_init();
  //set_motors(MIDPOSITION, MIDPOSITION);
  add_addr();

  //if(twi_write(LED_DVR, buff, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
  //if(twi_write(LED_DVR, buffer, 1) !=1) PORTA.OUTSET |= (1<<PIN5);
  //if(twi_read(LED_DVR, sens, 1) !=1) PORTA.OUTSET |= (1<<PIN5);

  //if(twi_write(0x29, buffer, 2) != 2) PORTA.OUTSET |= (1<<PIN5); 
  //if(twi_read(0x29, sens, 1) !=1) PORTA.OUTSET |= (1<<PIN5);

  while(1){
    //TWID_MASTER_ADDR = (0x29<<1);
    //while(!(*twi_stat & TWI_MASTER_WIF_bm));

  }
  return 0;
}

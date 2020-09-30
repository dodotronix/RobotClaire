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
#define ROTATE 660 //number of pulse to turn 90degree (680-passage)
#define SPEED 5 //speed of arm moves
#define SPDMOV 10
#define ERROR 5
#define MEASUREMENT_N 40 //number of measure samples
#define STRADDR 36 //start addres for proximity sensors 
#define ANGLE90 1450

//I2C device addresses
#define RIGHT 0x28
#define FRONT0 0x27
#define FRONT1 0x26
#define LEFT 0x25
#define BACK 0x24
#define COMPASS 0x1e
#define LED_DVR 0x60

//NATIVE SERVO POSITIONS
#define SERVO0 635
#define SERVO1 1235
#define SERVO2 1195
#define SERVO3 560
#define SERVO4 750
#define SERVO5 580

/*LIBRARIES & HEADERS*/
#include <avr/io.h> //MACROS
#include <avr/interrupt.h> //INTERRUPT settings
#include <util/delay.h> //DELAY_MS()
#include <stdio.h>

/*GLOBAL VARIABLES & ARRAYS*/
volatile int16_t enk_val [] ={0, 0}; //encoders current value [enk0, enk1]
volatile int16_t enk_val_ls [] ={0, 0}; //encoders last value
volatile int16_t curr_spd [] ={0, 0}; //current speed array
volatile int16_t last_spd [] ={0, 0}; //4 ticks / 20ms
volatile int16_t req_spd [] ={0, 0}; //required speed, this variable change speed of robot
volatile int16_t e_sum[] ={0, 0}; //integral state for PID 
volatile uint16_t motors[] ={0, 0};
volatile int16_t position[] ={0, 0};
volatile int16_t dState[] ={0, 0};
uint8_t mode =0;
uint8_t mov =0; //indicate operation in the robot movement (set exact position or set speed of motors)

//CONTROL BYTES
uint8_t refl[] ={0x06, 0x15}; //register and address of reflector
uint8_t blue0[] ={0x06, 0x05}; //register and address of blue led LED_driver
uint8_t blue1[] ={0x06, 0x11}; //register and address of second blue led

/*POINTERS TO REGISTERS*/
volatile uint8_t *twi_stat = (volatile uint8_t *) &TWID_MASTER_STATUS;
volatile uint8_t *twi_data = (volatile uint8_t *) &TWIC_SLAVE_DATA;

//ROBOTIC_ARM
volatile uint16_t *servo0 = (volatile uint16_t *) &TCF1.CCA; //robotic arm motion
volatile uint16_t *servo1 = (volatile uint16_t *) &TCF1.CCB; //main part1(biggest)
volatile uint16_t *servo2 = (volatile uint16_t *) &TCF0.CCB; //main part2(middle)
volatile uint16_t *servo3 = (volatile uint16_t *) &TCF0.CCD; //main part3(smallest)
volatile uint16_t *servo4 = (volatile uint16_t *) &TCF0.CCC; //arm jawes motion
volatile uint16_t *servo5 = (volatile uint16_t *) &TCF0.CCA; //arm jawes
volatile uint16_t *motor0 = (volatile uint16_t *) &TCD1.CCA; //motor0
volatile uint16_t *motor1 = (volatile uint16_t *) &TCE0.CCD; //motor1
volatile uint8_t *enk0 = (volatile uint8_t *) &PORTD_IN; //PORTD STATUS
volatile uint8_t *enk1 = (volatile uint8_t *) &PORTE_IN; //PORTE STATUS
volatile uint8_t *modept = &mode; //pointer to mode variable
volatile uint8_t *movpt = &mov; //pointer to mov variable

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
  PORTD.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
  PORTD.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;
  TWID_MASTER_BAUD = 0x28;//0x28; //355kHz //0x90; //106kHz
  TWID_MASTER_CTRLA = TWI_MASTER_ENABLE_bm ; // Enable TWI interface
  //TWID_MASTER_CTRLB = TWI_MASTER_SMEN_bm | TWI_MASTER_QCEN_bm;
  TWID_MASTER_STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;  //set bus state to IDLE mode
  _delay_ms(5);
}

void led_init(){
  PORTA.DIRSET = (1<<PIN4) | (1<<PIN5) | (1<<PIN6); //initialize LEDs (A4; A5; A6)
}

void portH_init(){
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
    return 1;
  }
  else {
    TWID_MASTER_CTRLC = 0x03;
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

void vlx_settings(uint8_t device){
  vlx_write(device, 0x0207, 0x01);
  vlx_write(device, 0x0208, 0x01);
  vlx_write(device, 0x0096, 0x00);
  vlx_write(device, 0x0097, 0xfd);
  vlx_write(device, 0x00e3, 0x00);
  vlx_write(device, 0x00e4, 0x04);
  vlx_write(device, 0x00e5, 0x02);
  vlx_write(device, 0x00e6, 0x01);
  vlx_write(device, 0x00e7, 0x03);
  vlx_write(device, 0x00f5, 0x02);
  vlx_write(device, 0x00d9, 0x05);
  vlx_write(device, 0x00db, 0xce);
  vlx_write(device, 0x00dc, 0x03);
  vlx_write(device, 0x00dd, 0xf8);
  vlx_write(device, 0x009f, 0x00);
  vlx_write(device, 0x00a3, 0x3c);
  vlx_write(device, 0x00b7, 0x00);
  vlx_write(device, 0x00bb, 0x3c);
  vlx_write(device, 0x00b2, 0x09);
  vlx_write(device, 0x00ca, 0x09);
  vlx_write(device, 0x0198, 0x01);
  vlx_write(device, 0x01b0, 0x17);
  vlx_write(device, 0x01ad, 0x00);
  vlx_write(device, 0x00ff, 0x05);
  vlx_write(device, 0x0100, 0x05);
  vlx_write(device, 0x0199, 0x05);
  vlx_write(device, 0x01a6, 0x1b);
  vlx_write(device, 0x01ac, 0x3e);
  vlx_write(device, 0x01a7, 0x1f);
  vlx_write(device, 0x0030, 0x00);
}

void vlx_Spcsettings(uint8_t device){
// Recommended : Public registers - See data sheet for more detail
  vlx_write(device, 0x0011, 0x10); // Enables polling for ‘New Sample ready’
  // when measurement completes
  vlx_write(device, 0x010a, 0x30); // Set the averaging sample period
  // (compromise between lower noise and
  // increased execution time)
  vlx_write(device, 0x003f, 0x46); // Sets the light and dark gain (upper
  // nibble). Dark gain should not be
  // changed.
  vlx_write(device, 0x0031, 0xFF); // sets the # of range measurements after
  // which auto calibration of system is
  // performed
  vlx_write(device, 0x0040, 0x63); // Set ALS integration time to 100ms
  vlx_write(device, 0x002e, 0x01); // perform a single temperature calibration
  // of the ranging sensor
  //Optional: Public registers - See data sheet for more detail
  vlx_write(device, 0x001b, 0x09); // Set default ranging inter-measurement
  // period to 100ms
  vlx_write(device, 0x003e, 0x31); // Set default ALS inter-measurement period
  // to 500ms
  vlx_write(device, 0x0014, 0x24); // Configures interrupt on ‘New Sample
  // Ready threshold event’
}

void vlx_init(uint8_t device){
  _delay_ms(1);
  if(vlx_read(device, 0x016) != 0x01){
    while(1){ 
      vlx_write(device, 0x010, 0x01);
      _delay_ms(1); //wait for 1ms [device boot]
      if(vlx_read(device, 0x016) == 0x01) break; //register SYSTEM__FRESH_OUT_OF_RESET set to "1"?
      else{
        vlx_write(device, 0x010, 0x00); //set GPIO to "0"
        _delay_ms(1); //wait for 1ms
      }
    }
  }
  vlx_settings(device); //load recommanded settings
  vlx_Spcsettings(device); //load special settings
  vlx_write(device, 0x016, 0x00);
}

//change sensor addres from 0x24 to 0x28
//To run this funcion it must be initialized PORTH 
uint8_t add_addr(){
  uint8_t addr = STRADDR; //start address to assign
  uint8_t dev;
  volatile uint8_t numb = 0;
  for(uint8_t i = 0; i<6;){
    if(i != 3){
      PORTH.OUTSET |= (1<<i); //turn on sensor on pin "i"
      _delay_ms(1);
      // search for sensor
      if(dev_check(0x29) ==0){
        ++numb;
        vlx_write(0x29, 0x212, addr);// create new addres -> write in its register
        vlx_init(addr); // initialize explicit sensor
      }
      else PORTH.OUTCLR = (1<<i);//if there is no sensor connected -> pin turn off
      ++addr; //increase addres to create new addres for next sensor
    }
    ++i; //counting cycles
  }
  dev = numb;
  //blink, how many sensors are connected
  for(;numb>0;){
    PORTA.OUTSET |= (1<<PIN6);
    _delay_ms(250);
    PORTA.OUTCLR |= (1<<PIN6);
    _delay_ms(250);
    numb -=1;
  }
  return dev;
}

//restart device if LED driver and sensor has fallen down
void check(uint8_t num){
  if(dev_check(0x29) ==0){
    num =add_addr();
  }
}

//set single shot mode on explicit sensor
uint8_t vlx_sing_shot(uint8_t device){
  uint8_t dist = 0;
  vlx_write(device, 0x18, 0x01); //start measurement
  while((vlx_read(device, 0x04f) & 0x04) != 0x04); //wait for bit 2 in register RESULT__INTERRUPT_STATES_GPIO
  dist = vlx_read(device, 0x062); //read distance 
  vlx_write(device, 0x015, 0x07); //clear interrupt
  return dist;
}

//this funcion is PD regulator to turn robot on exact position
uint16_t set_position(int16_t req_pos, uint8_t id){
  volatile int32_t result =0;
  volatile int32_t pTerm =0;
  volatile int32_t dTerm =0;

  if(id ==0){
    //Kp=1; Kd=12
    pTerm =(3*(req_pos -enk_val[0]))/2; //div4
    dTerm =12*(enk_val[0] -dState[0]); //12
    dState[0] =enk_val[0];
    result = pTerm -dTerm;
  }

  else if(id ==1){
    pTerm =(3*(req_pos -enk_val[1]))/2; // div4
    dTerm =12*(enk_val[1] -dState[1]); //12
    dState[1] =enk_val[1];
    result = pTerm -dTerm;
  }
  
  if(result >0)
    result += (MIDPOSITION +10); //value 10 is high bound of null position
  else if(result ==0)
    result += MIDPOSITION;
  else if(result <0)
    result += (MIDPOSITION -15); //low bound of null position
  
  //if value 'result' is bigger than 1080 or lower than 406 it will be overflow
  if(result >1080)//MAX supported motor pulse
      result =1080;
  else if (result <406)//MIN supported motor pulse
      result = 406; 

  return (uint16_t) result;
}

//PI regulator for speed control
uint16_t set_speed(uint8_t id){
  if(id ==0){
    motors[0] =*motor0;
    motors[0] += (req_spd[0] -curr_spd[0]) +e_sum[0]/2; //PI equation 
  }
  else if(id ==1){
    motors[1] =*motor1;
    motors[1] += (req_spd[1] -curr_spd[1]) +e_sum[1]/2; //PI equation
  }
  //if value 'result' is bigger than 1080 or lower than 406 it will be overflow
  if(motors[id] >1080)//MAX supported motor pulse
      motors[id] =1080;
  else if (motors[id] <406)//MIN supported motor pulse
      motors[id] = 406; 

  return motors[id];
}

void pos_clear(){
  position[0] =0;
  position[1] =0;
}

void turn(int16_t tics, uint8_t dir){
  pos_clear(); //clear position array
  if(dir ==0){
    //forwards
    for(int i =0; i<=tics;){
      position[0] +=1;
      position[1] +=1;
      _delay_ms(1);
      ++i;
    }
    while(!(((enk_val[0] >(tics -ERROR)) && (enk_val[0] <(tics +ERROR))) && ((enk_val[1] >(tics -ERROR)) && (enk_val[1] <(tics +ERROR))))){
      if(*modept ==1){
        if(twi_write(LED_DVR, blue0, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
      }
      else if(*modept ==2){
        if(twi_write(LED_DVR, blue1, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
      }
      if(((enk_val[0] == enk_val_ls[0]) && (enk_val[1] == enk_val_ls[1]))) break;
    }
  }
  else if(dir ==1){
    //turn right
    for(int i =0; i<=tics;){
      position[0] +=1;
      position[1] -=1;
      _delay_ms(1);
      ++i;
    }
    //while(!((enk_val[0] ==tics) & (enk_val[1] ==-1*tics)));
    while(!(((enk_val[0] >(tics -ERROR)) && (enk_val[0] <(tics +ERROR))) && ((enk_val[1] <(-1*(tics -ERROR))) && (enk_val[1] >(-1*(tics +ERROR)))))){
      if(*modept ==1){
        if(twi_write(LED_DVR, blue0, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
      }
      else if(*modept ==2){
        if(twi_write(LED_DVR, blue1, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
      }
      if(((enk_val[0] == enk_val_ls[0]) && (enk_val[1] == enk_val_ls[1]))) break;
    }
  }
  else if(dir ==2){
    //turn right
    for(int i =0; i<=tics;){
      position[0] -=1;
      position[1] +=1;
      _delay_ms(1);
      ++i;
    }
    while(!(((enk_val[1] >(tics -ERROR)) && (enk_val[1] <(tics +ERROR))) && ((enk_val[0] <(-1*(tics -ERROR))) && (enk_val[0] >(-1*(tics +ERROR)))))){
      if(*modept ==1){
        if(twi_write(LED_DVR, blue0, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
      }
      else if(*modept ==2){
        if(twi_write(LED_DVR, blue1, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
      }
      if(((enk_val[0] == enk_val_ls[0]) && (enk_val[1] == enk_val_ls[1]))) break;
    }
  }
  else if(dir ==3){
    //backwards
    for(int i =0; i<=tics;){
      position[0] -=1;
      position[1] -=1;
      _delay_ms(1);
      ++i;
    }
    while(!(((enk_val[0] <(-1*(tics -ERROR))) && (enk_val[0] >(-1*(tics +ERROR)))) && ((enk_val[1] <(-1*(tics -ERROR))) && (enk_val[1] >(-1*(tics +ERROR)))))){
      if(*modept ==1){
        if(twi_write(LED_DVR, blue0, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
      }
      else if(*modept ==2){
        if(twi_write(LED_DVR, blue1, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
      }
      if(((enk_val[0] == enk_val_ls[0]) && (enk_val[1] == enk_val_ls[1]))) break;
    }
  }
}

//move straight
void straight(){
  int16_t diff;
  if(req_spd[0] >0){
    diff = enk_val[0] -enk_val[1];
    if((diff <0) && (diff >-1000)){
      req_spd[1] -=1;
    }
    else if((diff >0) && (diff <1000)){
      req_spd[1] +=1;
    }
  }
  else{

  }
}

/*SINGLE_SLOPE_GENERATION*/
//servos timers (6 x servo)
//TIMER0 & 1 PORTF
void servos_init(){
  /*
   * TCF1.CCA = servo0;
   * TCF1.CCB = servo1;
   * TCF0.CCA = servo2;
   * TCF0.CCB = servo3;
   * TCF0.CCC = servo4;
   * TCF0.CCD = servo5;
  */

  PORTF.DIR |= (1<<PIN0) | (1<<PIN1) | (1<<PIN2) | (1<<PIN3) | (1<<PIN4) | (1<<PIN5);
  //timer0
  TCF0.CTRLB = TC_WGMODE_SS_gc|TC0_CCAEN_bm|TC0_CCBEN_bm|TC0_CCCEN_bm|TC0_CCDEN_bm;
  TCF0.PER = PWM_PER; //period
  TCF0.CTRLA = Prescaler; //prescaler

  //timer1 
  TCF1.CTRLB = TC_WGMODE_SS_gc|TC1_CCAEN_bm|TC1_CCBEN_bm; //set single slope mode
  TCF1.PER = PWM_PER; //period
  TCF1.CTRLA = Prescaler; //prescaler
}

void motor_ctrl(){
  TCD0.CTRLB = TC_WGMODE_SS_gc; //set single slope mode 
  TCD0.INTCTRLA = TC_OVFINTLVL_HI_gc;
  TCD0.PER = 299;//499; //1kHz
  TCD0.CTRLA = Prescaler; //prescaler TC_CLKSEL_DIV8_gc;
}

ISR(TCD0_OVF_vect){ 
  if(*movpt ==2){
    *motor0 =set_position(position[0], 0);
    *motor1 =set_position(position[1], 1);
  }
}

//motor0 - D4; motor1 - E3
//TIMER1 PORTD
void motor0_init(){
  PORTD.DIR |= (1<<PIN4); //set pin as output
  TCD1.CTRLB = TC_WGMODE_SS_gc|TC1_CCAEN_bm; //set single slope mode
  TCD1.INTCTRLA = TC_OVFINTLVL_HI_gc;
  TCD1.PER = PWM_PER; //period
  TCD1.CTRLA = Prescaler; //prescaler
  *motor0 = MIDPOSITION;
}

ISR (TCD1_OVF_vect){
  curr_spd[0] = enk_val[0] - enk_val_ls[0]; //encoders values difference
  enk_val_ls[0] = enk_val[0]; //actual encoder value is now last value
  if(*movpt ==1){
    *motor0 =set_speed(0);
  }
}

//TIMER0 PORTE
void motor1_init(){
  PORTE.DIR |= (1<<PIN3); //set pin as output
  TCE0.CTRLB = TC_WGMODE_SS_gc|TC0_CCDEN_bm; //set single slope mode 
  TCE0.INTCTRLA = TC_OVFINTLVL_HI_gc;  
  TCE0.PER = PWM_PER; //period
  TCE0.CTRLA = Prescaler; //prescaler
  *motor1 = MIDPOSITION; //set motor in middle position
}

ISR (TCE0_OVF_vect){
  curr_spd[1] = enk_val[1] - enk_val_ls[1]; //encoders values difference
  enk_val_ls[1] = enk_val[1]; //actual encoder value is now last value
  if(*movpt ==1){
    *motor1 =set_speed(1);
  }
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
  if(*modept == 2) 
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
  //overflow protection
  if(enk_val[0] ==32001 || enk_val[0] ==-32001){
    enk_val[0] = enk_val[0] -enk_val_ls[0];
    enk_val_ls[0] =0;
  }
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
  //overflow protection
  if(enk_val[1] ==32001 || enk_val[1] ==-32001){
    enk_val[1] = enk_val[1] -enk_val_ls[1];
    enk_val_ls[1] =0;
  }
}

//set every servo to his native value
void servo_set(){
  *servo5 = SERVO5; //arm jawes open/close, increase open
  *servo0 = SERVO0; //arm motion, increase value to turn left
  *servo1 = SERVO1;//1216; //main part1 (the biggest), decrease to move up
  *servo2 = SERVO2; //main part2 (middle), increase to move up
  *servo3 = SERVO3; //main part3 (the smallest), increase to move up
  *servo4 = SERVO4; //arm jawes motion, increase to turn left
}

//set servo to new position from his actual position
void servo_pos(volatile uint16_t *servo, uint16_t onposition){
  if(*servo >onposition){
    while(*servo >onposition){
      *servo -=1;
      _delay_ms(SPEED);
    }
  }
  else{
    while(*servo <onposition){
    *servo +=1;
    _delay_ms(SPEED);
    }
  }
}

//function take an predefined object
void take_object(int16_t turn){
  uint16_t pulse =612 +((2*(turn))/3);
  //uint16_t pulse =620 +((5*(turn))/9);

  servo_pos(servo2, 850);
  servo_pos(servo3, 800);
  *servo5 =970; 
  //servo_pos(servo3, (5*(*servo2))/9);
  servo_pos(servo1, 800);
  servo_pos(servo3, 570);
  servo_pos(servo0, pulse);
  servo_pos(servo4, (6*(*servo0))/5);
  servo_pos(servo3, 620);
  servo_pos(servo1, 600);
  *servo5 =650;
  servo_pos(servo3, 800);
  if(*servo0 != SERVO0)
    servo_pos(servo0, SERVO0);
  servo_pos(servo1, SERVO1);
  servo_pos(servo2, SERVO2);
  servo_pos(servo4, SERVO4);
}


void get_me_object(){
  servo_pos(servo2, 900);
  servo_pos(servo3, 1000);
  servo_pos(servo1, 800);
  servo_pos(servo2, 500);
  servo_pos(servo5, 950);
  _delay_ms(1000);
  servo_pos(servo5, SERVO5);
  servo_pos(servo3, SERVO3);
  servo_pos(servo2, 900);
  servo_pos(servo1, SERVO1);
  servo_pos(servo2, SERVO2);
}

//turn servos in their back position
void servo_back(){
  servo_pos(servo0, SERVO0);
  servo_pos(servo1, SERVO1);
  servo_pos(servo2, SERVO2);
  servo_pos(servo3, SERVO3);
  servo_pos(servo4, SERVO4);
  servo_pos(servo5, SERVO5);
}

void enk_clear(){
  for(int i =0; i<2; ++i){
    enk_val[i] =0;
    enk_val_ls[i] =0;
  }
}

//make average from measured data
uint8_t avg_filter(uint16_t addr){
  uint16_t sum =0;
  for(uint8_t i =0; i <MEASUREMENT_N; ++i){
    sum +=vlx_sing_shot(addr); 
  }
  return  sum/MEASUREMENT_N;
}

int main(){
  /*LOCAL VARIABLES*/
  uint8_t node =0; //watch the actual level of progress
  uint8_t sens_num;
  uint8_t sensors[] ={0, 0}; //array for sensors sample average

  /*INITIALIZATION*/
  mcu_clk32_init(); //initilize 32MHz intern oscilator
  led_init(); //initialize control LED connected to ATxmega64
  portH_init(); //initialize PORTH to turn on/off i2c sensors
  twi_init(); // initialize TWI (I2C)
  sens_num =add_addr(); //create addresses to every proximity_sensor
  //if you want to send more bytes via I2C attend to turn off all interrupts [cli()]

  /*INTERRUPTS_INITIALIZE*/
  mode_init(); //initialize mode button interrupt
  enk0_init(); // -||- encoder0 (motor0)
  enk1_init(); // -||- encoder1 (motor1)

  /*TIMERS_INITIALIZE*/
  motor_ctrl();
  motor0_init();
  motor1_init();
  servos_init();
  sei(); //TURN ON ALL INTERRUPTS
  servo_set(); //set servos to their initial position

  while(1){
    if(*modept ==0){
      if(twi_write(LED_DVR, refl, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
      PORTA.OUTCLR = (1<<PIN5) | (1<<PIN6) | (1<<PIN4); //clear error indicator
      while(*modept ==0);
    }
    else if(*modept ==1){
      PORTA.OUTCLR = (1<<PIN5); //clear error indicator
      if(twi_write(LED_DVR, blue0, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
        while(*modept ==1){
        //start moving just in case all proximity sensors detected
          if(sens_num ==5){
            enk_clear(); //set encoders to zero 
            _delay_ms(500);
            req_spd[0] =10;
            req_spd[1] =10;
            *movpt =1; //start moving
            while(*modept ==1){
              if(node ==0){
                if(vlx_sing_shot(FRONT0) <200 || vlx_sing_shot(FRONT1) <200){
                  PORTA.OUTSET =(1<<PIN4);
                  req_spd[0] =0;
                  req_spd[1] =0;
                  while(!(curr_spd[0] ==0 && curr_spd[1] ==0)){
                    if(twi_write(LED_DVR, blue0, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
                  }//wait till the robot stay
                  node =1; 
                }
                else
                  PORTA.OUTCLR =(1<<PIN4);
              }
              else if(node ==1){
                if(vlx_sing_shot(RIGHT) >200){
                  enk_clear(); //set encoders to zero 
                  *movpt =2;
                  turn(ANGLE90, 1);
                  *movpt =1;
                  node =2;
                } 
                else if(vlx_sing_shot(LEFT) >200){
                  enk_clear(); //set encoders to zero 
                  *movpt =2;
                  turn(ANGLE90, 2);
                  *movpt =1;
                  node =2;
                }
                else if(vlx_sing_shot(BACK) >200){
                  req_spd[0] =-10;
                  req_spd[1] =-10;
                  node =3;
                }  
              }
              else if(node ==2){
                _delay_ms(500);
                req_spd[0] = SPDMOV;
                req_spd[1] = SPDMOV;
                node =0;
              }
              else if(node ==3){
                if(vlx_sing_shot(RIGHT) >200 || vlx_sing_shot(LEFT) >200){
                  req_spd[0] =0;
                  req_spd[1] =0;
                  while(!(curr_spd[0] ==0 && curr_spd[1] ==0)){
                    if(twi_write(LED_DVR, blue0, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
                  }//wait till the robot stay
                  enk_clear(); //set encoders to zero 
                  *movpt =2;
                  turn(700, 3);
                  *movpt =1;
                   node =1;
                }
                else if(vlx_sing_shot(BACK) <200){
                  req_spd[0] =0;
                  req_spd[1] =0;
                  while(!(vlx_sing_shot(BACK) >200)){
                    if(twi_write(LED_DVR, blue0, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
                  }
                  req_spd[0] =-10;
                  req_spd[1] =-10;
                }
              }
            if(twi_write(LED_DVR, blue0, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
            }
            *movpt =1;
            req_spd[0] =0;
            req_spd[1] =0;
          }
        }
    }
    else if(*modept ==2){
      *movpt =1;
      node =0;
      req_spd[0] =0;
      req_spd[1] =0;
      PORTA.OUTCLR = (1<<PIN5); //clear error indicator
      if(twi_write(LED_DVR, blue1, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
      if(sens_num ==5){
        enk_clear(); //set encoders to zero
        _delay_ms(500);
        req_spd[0] =10;
        req_spd[1] =10;
        *movpt =1; //start moving
        while(*modept ==2){
          if(node ==0){
            if(vlx_sing_shot(FRONT0) <90 || vlx_sing_shot(FRONT1) <90){
              PORTA.OUTSET =(1<<PIN4);
              req_spd[0] =0;
              req_spd[1] =0;
              while(!(curr_spd[0] ==0 && curr_spd[1] ==0));//wait till the robot stay
              node =1;
            }
            else PORTA.OUTCLR =(1<<PIN4);
          }
          else if(node ==1){
            sensors[0] =avg_filter(FRONT0);
            sensors[1] =avg_filter(FRONT1);
            take_object(sensors[0] -sensors[1]);
            enk_clear(); //set encoders to zero
            _delay_ms(500);
            *movpt =2;
            turn(ANGLE90, 1);
            enk_clear(); //set encoders to zero
            turn(ANGLE90, 1);
            *movpt =1;
            _delay_ms(500);
            req_spd[0] = SPDMOV;
            req_spd[1] = SPDMOV;
            node =2;
          }
          else if(node ==2){
            if(vlx_sing_shot(FRONT0) <120 || vlx_sing_shot(FRONT1) <120){
              PORTA.OUTSET =(1<<PIN4);
              req_spd[0] =0;
              req_spd[1] =0;
              while(!(curr_spd[0] ==0 && curr_spd[1] ==0));//wait till the robot stay
              _delay_ms(1000);
              get_me_object();
              *modept =0;
            }
          }
        }
        *movpt =1;
        req_spd[0] =0;
        req_spd[1] =0;
      }
    }
  }
  return 0;
}

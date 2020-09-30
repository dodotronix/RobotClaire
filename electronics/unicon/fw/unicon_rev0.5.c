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

//I2C device addresses
#define RIGHT 0x28
#define FRONT0 0x27
#define FRONT1 0x26
#define LEFT 0x25
#define BACK 0x24
#define COMPASS 0x1e
#define LED_DVR 0x60

//NATIVE SERVO POSITIONS
#define SERVO0 630 
#define SERVO1 1235
#define SERVO2 1200
#define SERVO3 650
#define SERVO4 820
#define SERVO5 560

/*LIBRARIES & HEADERS*/
#include <avr/io.h> //MACROS
#include <avr/interrupt.h> //INTERRUPT settings
#include <util/delay.h> //DELAY_MS()
#include <stdio.h>

/*GLOBAL VARIABLES*/
volatile int enk_val [] = {0, 0}; //encoders current value [enk0, enk1]
volatile int enk_val_ls [] = {0, 0}; //encoders last value
volatile int curr_spd [] = {0, 0}; //current speed array
volatile int last_spd [] = {0, 0}; //4 ticks / 20ms
volatile int req_spd [] = {0, 0}; //required speed, this variable change speed of robot
volatile uint8_t front[] = {0, 0}; //array for front proximity sensor output values
uint8_t mode = 0;
uint8_t allsens =0; //check if all sensors connected

/*POINTERS TO REGISTERS*/
volatile uint8_t *twi_stat = (volatile uint8_t *) &TWID_MASTER_STATUS;
//ROBOTIC_ARM
volatile uint16_t *servo0 = (volatile uint16_t *) &TCF1.CCA; //robotic arm motion
volatile uint16_t *servo1 = (volatile uint16_t *) &TCF1.CCB; //main part1(biggest)
volatile uint16_t *servo2 = (volatile uint16_t *) &TCF0.CCB; //main part2(middle)
volatile uint16_t *servo3 = (volatile uint16_t *) &TCF0.CCD; //main part3(smallest)
volatile uint16_t *servo4 = (volatile uint16_t *) &TCF0.CCC; //arm jawes motion
volatile uint16_t *servo5 = (volatile uint16_t *) &TCF0.CCA; //arm jawes

//volatile uint8_t *pReg = (volatile uint8_t *) &PORTE_IN;
volatile uint8_t *modept = &mode;

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
void add_addr(){
  uint8_t addr = 36; //start address to assign
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
  allsens = numb;
  //blink, how many sensors are connected
  for(;numb>0;){
    PORTA.OUTSET |= (1<<PIN6);
    _delay_ms(250);
    PORTA.OUTCLR |= (1<<PIN6);
    _delay_ms(250);
    numb -=1;
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

//motor0 - D4; motor1 - E3
//TIMER1 PORTD
void motor0_init(){
  PORTD.DIR |= (1<<PIN4); //set pin as output
  TCD1.CTRLB = TC_WGMODE_SS_gc|TC1_CCAEN_bm; //set single slope mode 
  TCD1.INTCTRLA = TC_OVFINTLVL_HI_gc;
  TCD1.PER = PWM_PER; //period
  TCD1.CTRLA = Prescaler; //prescaler
  TCD1.CCA = MIDPOSITION;
}

//speed meassure time - messuring speed with 50Hz
ISR (TCD1_OVF_vect){
  last_spd[0] = curr_spd[0]; //save curr_speed value as last_speed value
  curr_spd[0] = enk_val[0] - enk_val_ls[0]; //encoders values difference
  enk_val_ls[0] = enk_val[0]; //actual encoder value is now last value
  TCD1.CCA += (req_spd[0]-curr_spd[0])-(curr_spd[0]-last_spd[0]);
}

//TIMER0 PORTE
void motor1_init(){
  PORTE.DIR |= (1<<PIN3); //set pin as output
  TCE0.CTRLB = TC_WGMODE_SS_gc|TC0_CCDEN_bm; //set single slope mode
  TCE0.INTCTRLA = TC_OVFINTLVL_HI_gc;
  TCE0.PER = PWM_PER; //period
  TCE0.CTRLA = Prescaler; //prescaler
  TCE0.CCD = MIDPOSITION; //set motor in middle position
  /*SET PWM FOR MOTORS*/
  //pulswidth between 1ms - 2ms (508 - 1016)
  //null point for sabertooth == 745
  //increase value --> revers move
  //decrease value --> forward move
}

//speed messure time - messuring speed with 50Hz
ISR (TCE0_OVF_vect){
  last_spd[1] = curr_spd[1]; //save curr speed value as last speed value
  curr_spd[1] = enk_val[1] - enk_val_ls[1]; //encoders value difference == new speed
  enk_val_ls[1] = enk_val[1]; // save curr value in value last
  TCE0.CCD += (req_spd[1]-curr_spd[1])-(curr_spd[1]-last_spd[1]);
}

/*MODE INTERRUPT*/
//interrupt - C3
void mode_init(){
  PORTC.INT0MASK = (1<<PIN3);
  PORTC.PIN3CTRL = PORT_ISC_FALLING_gc;
  PORTC.INTCTRL |= PORT_INT0LVL_LO_gc;
  PMIC.CTRL |= PMIC_HILVLEN_bm |PMIC_MEDLVLEN_bm|PMIC_LOLVLEN_bm;
}
//swich mode
ISR(PORTC_INT0_vect){
  if(*modept == 2) *modept = 0;
  else *modept +=1;
}  

/*ENCODER INTERRUPTS*/
void enk0_init(){
  PORTD.INT0MASK = (1<<PIN2);
  PORTD.PIN2CTRL = PORT_ISC_RISING_gc;
  PORTD.INTCTRL |= PORT_INT0LVL_HI_gc;
  PMIC.CTRL |= PMIC_HILVLEN_bm |PMIC_MEDLVLEN_bm|PMIC_LOLVLEN_bm;
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
}

ISR(PORTE_INT0_vect){
  if((PORTE_IN &= PIN4_bm) == 0){
    enk_val[1] -=1;
  }
  else{
    enk_val[1] +=1;
  }
}
//set every servo to his native value
void servo_set(){
  *servo0 = 630; //arm motion, increase value to turn left
  *servo1 = 1235;//1216; //main part1 (the biggest), decrease to move up
  *servo2 = 1200; //main part2 (middle), decrease to move up
  *servo3 = 650; //main part3 (the smallest), increase to move up
  *servo4 = 820; //arm jawes motion, increase to turn left
  *servo5 = 560; //arm jawes open/close, increase open
}

int main(){
  /*LOCAL VARIABLES*/
  uint8_t node = 0; //watch the actual level of progress
  volatile uint16_t cycle = 0; //count cycles
  uint8_t speed = 5; //speed of arm movement
  uint8_t refl[] = {0x06, 0x15};
  uint8_t blue0[] = {0x06, 0x05};
  uint8_t blue1[] = {0x06, 0x11};

  /*INITIALIZATION*/
  mcu_clk32_init(); //initilize 32MHz intern oscilator
  led_init(); //initialize control LED connected to ATxmega64
  portH_init(); //initialize PORTH to turn on/off i2c sensors
  twi_init(); // initialize TWI (I2C)
  add_addr(); //create addresses to every proximity_sensor
  //if you want to send more bytes via I2C attend to turn off all interrupts [cli()]

  /*INTERRUPTS_INITIALIZE*/
  mode_init(); //initialize mode button interrupt
  enk0_init(); // -||- encoder0 (motor0)
  enk1_init(); // -||- encoder1 (motor1)

  /*TIMERS_INITIALIZE*/
  motor0_init();
  motor1_init();
  servos_init();

  sei(); //TURN ON ALL INTERRUPTS

  //if(twi_read(LED_DVR, sens, 1) !=1) PORTA.OUTSET |= (1<<PIN5);
  //if(twi_write(0x29, buffer, 2) != 2) PORTA.OUTSET |= (1<<PIN5); 
  //(twi_read(0x29, sens, 1) !=1) PORTA.OUTSET |= (1<<PIN5);
  //if(sens[0] == 180) PORTA.OUTSET |= (1<<PIN4);

  while(1){
    switch (*modept){
      case 0:
        if(twi_write(LED_DVR, refl, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
        while(*modept == 0);

      case 1:
        PORTA.OUTCLR = (1<<PIN5); //clear error indicator
        if(twi_write(LED_DVR, blue0, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
        //start moving just in case all proximity sensors detected
        if(allsens ==5){
          _delay_ms(500);
          req_spd[0] =8;
          req_spd[1] =8;
          while(*modept ==1){
            //front[0] = vlx_sing_shot(FRONT0);
            if(node ==0){
              if(vlx_sing_shot(FRONT0) <255 || vlx_sing_shot(FRONT1) <255){
                //PORTA.OUTSET =(1<<PIN4);
                req_spd[0] =0;
                req_spd[1] =0;
                node =1; //set control node number
              }
            }
            else if(node ==1 && curr_spd[0] ==0 && curr_spd[1] ==0){
              node =2;
            }
            //where can the robot go?
            else if(node ==2){
              //is on the right side any barrier?
              if(vlx_sing_shot(RIGHT) ==255){
                //PORTA.OUTSET = (1<<PIN4);
                req_spd[0] =8; //change required speed
                req_spd[1] =-8;
                _delay_ms(2000);
                node =3; //set control node
              } 
              else if(vlx_sing_shot(LEFT) ==255){
                //PORTA.OUTSET = (1<<PIN4);
                req_spd[0] =-8;
                req_spd[1] =8;
                _delay_ms(2000);
                node=3;
              }
              else if(vlx_sing_shot(BACK) ==255){
                req_spd[0] =-8;
                req_spd[1] =-8;
                node =5;
              }
            }
            else if(node ==3){
              req_spd[0] =0;
              req_spd[1] =0;
              node =4;
            }
            else if(node ==4 && curr_spd[0] ==0 && curr_spd[1] ==0){
              req_spd[0] =8;
              req_spd[1] =8;
              node =0;
            }
            
            else if(node ==5){
              if(vlx_sing_shot(RIGHT) ==255 || vlx_sing_shot(LEFT) ==255){
                if(vlx_sing_shot(BACK) ==255 && cycle ==30){
                  req_spd[0] =0;
                  req_spd[1] =0;
                  cycle =0;
                  node =2;
                }
                else if(vlx_sing_shot(BACK) <255 && cycle !=90){
                  req_spd[0] =0;
                  req_spd[1] =0;
                }
                else ++cycle;
              }
            }
          }
          req_spd[0] = 0;
          req_spd[1] = 0;
          node =0;
          cycle =0;
        }
        else{
          PORTA.OUTSET = (1<<PIN5);
          while(*modept ==1);
        }

      case 2:
        PORTA.OUTCLR = (1<<PIN5); //clear error indicator
        if(twi_write(LED_DVR, blue1, 2) !=2) PORTA.OUTSET |= (1<<PIN5);
        _delay_ms(500);
        servo_set(); //set all servos to their initial position

        //*servo2 = 900;
        while(*modept == 2){
          if((SERVO2 - cycle) >900){
            *servo2 -=1;
          ++cycle;
          _delay_ms(speed);
          }
        }
    }
  }

  return 0;
}

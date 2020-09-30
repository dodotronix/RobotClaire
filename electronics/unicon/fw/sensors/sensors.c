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
#define STRADDR 36
#define MEASUREMENT_N 10

#define FRONT0 0x27
#define FRONT1 0x26

/*LIBRARIES & HEADERS*/
#include <avr/io.h> //MACROS
#include <avr/interrupt.h> //INTERRUPT settings
#include <util/delay.h> //DELAY_MS()
#include <stdio.h>

/*GLOBAL VARIABLES*/
volatile uint8_t front[] ={0, 0}; //array for front proximity sensor output values
volatile uint8_t sens0 =0;
volatile uint8_t sens1 =0;
volatile uint8_t sensor0 [] ={0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t sensor1 [] ={0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/*POINTERS TO REGISTERS*/
volatile uint8_t *twi_stat = (volatile uint8_t *) &TWID_MASTER_STATUS;
volatile uint8_t *twi_data = (volatile uint8_t *) &TWIC_SLAVE_DATA;

volatile uint8_t *twi_stat_slv = (volatile uint8_t *) &TWIC_SLAVE_STATUS;
volatile uint8_t *twi_data_slv = (volatile uint8_t *) &TWIC_SLAVE_DATA;

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

/*TWI_INTERFACE_MASTER*/
void twi_init(){
  PORTD.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
  PORTD.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;
  TWID_MASTER_BAUD = 0x28;//0x28; //355kHz //0x90; //106kHz
  TWID_MASTER_CTRLA = TWI_MASTER_ENABLE_bm ; // Enable TWI interface
  //TWID_MASTER_CTRLB = TWI_MASTER_SMEN_bm | TWI_MASTER_QCEN_bm;
  TWID_MASTER_STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;  //set bus state to IDLE mode
  _delay_ms(5);
}

/*TWI_INTERFACE_SLAVE*/
void twi_init_slv(){
  TWIC_SLAVE_ADDR = 0x65; //slave address 0x18
  TWIC_SLAVE_CTRLA = TWI_SLAVE_ENABLE_bm; //enable twi slave device
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

//set single shot mode on explicit sensor
uint8_t vlx_sing_shot(uint8_t device){
  uint8_t dist = 0;
  vlx_write(device, 0x18, 0x01); //start measurement
  while((vlx_read(device, 0x04f) & 0x04) != 0x04); //wait for bit 2 in register RESULT__INTERRUPT_STATES_GPIO
  dist = vlx_read(device, 0x062); //read distance 
  vlx_write(device, 0x015, 0x07); //clear interrupt
  return dist;
}

void rpi_com(){
  while(!(*twi_stat_slv & TWI_SLAVE_APIF_bm)); //wait for master action
  //write
  if((*twi_stat_slv & TWI_SLAVE_DIR_bm) ==0){
    TWIC_SLAVE_CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; //send back address
    //waiting for 7bytes
  }
  //read
  else if((*twi_stat_slv & TWI_SLAVE_DIR_bm) !=0){ //do you want to transmitt more bytes
    cli();
    TWIC_SLAVE_CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; //ACK after address recieved
    while(!(*twi_stat_slv & TWI_SLAVE_DIF_bm));
    *twi_data_slv = sens0;

    TWIC_SLAVE_CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; //send data
    while(!(*twi_stat_slv & TWI_SLAVE_DIF_bm));
    *twi_data_slv = sens1;

    TWIC_SLAVE_CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; //send data
    while(!(*twi_stat_slv & TWI_SLAVE_DIF_bm));
    TWIC_SLAVE_CTRLB = TWI_SLAVE_CMD_COMPTRANS_gc | 0x04; //send NACK
    sei();
  }
}

uint8_t average(volatile uint8_t array[MEASUREMENT_N]){
  uint16_t sum =0;
  for(int i =0; i<MEASUREMENT_N; ++i){
    sum += array[i]; 
  }
  return  sum/MEASUREMENT_N;
}

// this function take carre of sorting the submited array
void insertionSort(uint8_t cislo, uint8_t pole[], uint8_t misto) {
  uint8_t k = misto;
  if(misto > 0) {
    while (pole[k-1] > cislo && k != 0) {
      pole[k] = pole[k-1];
      --k;
    }
    pole[k] = cislo;
  }
  else {
    pole[misto] = cislo;
  }
}

//TODO make better (faster) array check 
//function return the most frequented number from sensor
int8_t filter_most_freq(volatile uint8_t pole[], uint8_t pocet) {
  uint8_t nejvic = 0;
  uint8_t pomocPole[pocet];
  uint8_t k = 0;
  int16_t soucet = 0;
  uint8_t delitel = 1;    
  for (uint8_t i = 0; i < pocet; ++i) insertionSort(pole[i], pomocPole, i);
    soucet = pomocPole[0]; //jestlize je v poli zastoupen jen jeden prvek fce vrati jeho hodnotu 
  for(int8_t i = 0; i < pocet; ++i) //go thrue the array and count the most freq numbers
  {
    if(pomocPole[i] != pomocPole[i+1])
    {
      ++k;
      if(k > nejvic)
      {
        nejvic = k; // ulozi nejpocetnejsi cislo pro aktualni cas
        soucet = pomocPole[i]; // ulozi hodnotu pole do promenne 'soucet'
      }
      else if(k == nejvic)
      {
        soucet = soucet + pomocPole[i];
        ++delitel;
      }  
      k = 0; //vynuluje promenno 'k' pro pocitani dalsich totoznych cisel
    }
    else ++k;
  }
  return soucet/delitel;
}

int main(){ 
  /*LOCAL VARIABLES*/

  /*INITIALIZATION*/
  mcu_clk32_init(); //initilize 32MHz intern oscilator
  led_init(); //initialize control LED connected to ATxmega64
  portH_init(); //initialize PORTH to turn on/off i2c sensors
  twi_init(); // initialize TWI (I2C)
  twi_init_slv();
  add_addr(); //create addresses to every proximity_sensor
  //if you want to send more bytes via I2C attend to turn off all interrupts [cli()]  
  PORTA.OUTSET = (1<<PIN6);
  while(1){
    for(int i =0; i<MEASUREMENT_N; ++i){
      sensor0[i] = vlx_sing_shot(FRONT0);
      sensor1[i] = vlx_sing_shot(FRONT1);
    }
    sens0 =average(sensor0);
    sens1 =average(sensor1);
    //sens0 = filter_most_freq(sensor0, MEASUREMENT_N);
    //sens1 = filter_most_freq(sensor1, MEASUREMENT_N);
    //sens0 = vlx_sing_shot(FRONT0);
    //sens1 = vlx_sing_shot(FRONT1);
    rpi_com();
  }
  return 0;
}

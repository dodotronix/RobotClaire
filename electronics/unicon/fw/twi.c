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


/*LIBRARIES & HEADERS*/
#include <avr/io.h> //MACROS
#include <avr/interrupt.h> //INTERRUPT settings
#include <util/delay.h> //DELAY_MS()
#include <stdio.h>

/*POINTERS TO REGISTERS*/
volatile uint8_t *twi_stat = (volatile uint8_t *) &TWID_MASTER_STATUS;

void mcu_clk32_init(){
  OSC.CTRL |= OSC_RC32MEN_bm; //set 32MHz oscilator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); //wait for oscilator to be prepared
  CCP = CCP_IOREG_gc;
  CLK.CTRL = (CLK.CTRL & ~CLK_SCLKSEL_gm) | CLK_SCLKSEL_RC32M_gc;
}


/*TWI_INTERFACE*/
void twi_init(){
  PORTD.PIN0CTRL = PORT_OPC_WIREDAND_gc;
  PORTD.PIN1CTRL = PORT_OPC_WIREDAND_gc;
  TWID_MASTER_BAUD = 0x40; //0x23; //400kHz
  TWID_MASTER_CTRLA = TWI_MASTER_ENABLE_bm ; // Enable TWI interface
  TWID_MASTER_CTRLB = TWI_MASTER_SMEN_bm;  
  TWID_MASTER_STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;  //set bus state to IDLE mode
}

int main (){
  mcu_clk32_init();  
  twi_init();

for(uint8_t i = 0; i<250; ++i){
    TWID_MASTER_ADDR = (0x29<<1);
    while(!(TWID_MASTER_STATUS &= TWI_MASTER_WIF_bm));
    //if((*twi_stat & 0x03) == 3) PORTA.OUTSET = (1<<PIN5);
    if((TWID_MASTER_STATUS & 0x03) == 3) PORTA.OUTSET = (1<<PIN5);
    TWID_MASTER_CTRLC = 0x03;
  }
  
  while(1);


  return 0;
}

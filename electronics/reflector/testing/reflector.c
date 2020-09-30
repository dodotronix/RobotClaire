/* reflector.c
 * date: Wed Dec 30 12:14:01 CET 2015
 * front reflector testing
*/

#define _BSD_SOURCE
#define ADDRESS 0x60

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>  // uint8_t, etc
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

int I2CFile;

void deviceInit()
{
  I2CFile = open("/dev/i2c-1", O_RDWR);   // Open the I2C device
  ioctl(I2CFile, I2C_SLAVE, ADDRESS);   // Specify the address of the I2C Slave to communicate with
}

uint8_t readReg(uint16_t reg)
{
  uint8_t value[] = {0};
  uint8_t data[2];
  data[0] = (reg >> 8) & 0xff;  // reg high byte
  data[1] = reg & 0xff;         // reg low byte
  if(write(I2CFile, data, 2) != 2) {printf ("ERROR writing\n");}
  else {
      if(read(I2CFile, value, 1) != 1) {printf ("ERROR reading\n");}
  }

  return value[0];
}

void writeReg(uint16_t reg, uint8_t value)
{
  uint8_t data[3];
  data[0] = (reg >> 8) & 0xff;  // reg high byte
  data[1] = reg & 0xff;         // reg low byte
  data[2] = value;
  if(write(I2CFile, data, 3) != 3) {printf ("ERROR writing\n");}
}

void closeCOM ()
{
  close(I2CFile);
}

int main() {

  //uint8_t variable[] = {0};
  uint8_t data[] = {0x06, 0x015}; //REFLECTOR STATE ON
  deviceInit();
  if(write(I2CFile, data, 2) != 2) printf ("ERROR writing\n");
  //if(read(I2CFile, variable, 1) != 1) printf ("ERROR reading\n");
  //printf("LED = %u\n", variable[1]);
  //printf("LED = %u\n", readReg(0x000));
  closeCOM();
  
  return 0;
}

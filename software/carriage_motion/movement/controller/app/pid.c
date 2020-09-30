/* pid.c
 *
 * datum: Wed Dec 30 12:14:01 CET 2015
 * Author: idle_rug
 *
 * measure progress of pid
 *
*/

//#define CRTSCTS  020000000000
//#define __USE_MINGW_ANSI_STDIO 1 //or gcc prog.c -std=c99 -D__USE_MINGW_ANSI_STDIO

#define _BSD_SOURCE
#define ADDRESS 0x32

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <inttypes.h>  // uint8_t, etc
#include <pthread.h>   // THREADS (POSIX_library)
#include <unistd.h>    // TIMING
#include <time.h> 
#include <math.h>
#include <string.h>    // memcmp, memcpy
#include <assert.h>    // assert
#include <stdlib.h>    // EXIT_SUCCESS...
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

uint8_t readReg(uint8_t reg)
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

void writeReg(uint8_t reg, uint8_t value)
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

//split 16-bit number
void split_num(int16_t array0[4], uint8_t array1[8]){
  for(int i =0; i<4; ++i){
    array1[2*i] = (array0[i] >>8) &0xff;
    array1[2*i+1] = array0[i] & 0xff;
  }
}

int main(int argc, char*argv[]){
  FILE*fp;
  fp = fopen("/home/pi/programs/pid_out.txt", "w");
  int x;
  uint8_t send[] ={0, 0, 0 ,0, 0, 0, 0, 0};
  int16_t save[] ={0, 0, 0, 0};
  //int16_t save[] ={2, 4, 5, 30};
  int16_t output =0;
  uint8_t buffer[] ={0, 0};

  if(argc >1){
    //save arguments as numbers
    for(int i =0; i<4; ++i){
      x = atoi(argv[i+1]);
      save[i] = x;
    }
    split_num(save, send); 

    /*for debugging
    printf("send0 = %u\n", send[0]);
    printf("send0 = %u\n", send[1]);
    printf("send0 = %u\n", send[2]);
    */
    deviceInit();
    if(write(I2CFile, send, 8) != 8) printf ("ERROR writing\n"); //set speed of motion (max 20/20ms)
    for(int i =0; i<500; ++i){
      if(read(I2CFile, buffer, 2) != 2) printf ("ERROR reading\n");
      else{
        output =(buffer[0] <<8)|buffer[1];
        fprintf(fp,"%i %i\n", i, output);
        printf("respons: %i\n", output);
        usleep(10000); //frequenc 100Hz
      }
    }
  closeCOM();
  }
  else
    printf("ERROR - no gain constants were submitted!\n");
  return 0;
}





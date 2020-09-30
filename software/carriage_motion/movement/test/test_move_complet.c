/*-----------------------------------------------------------------------------*/
/* ATmega8_control_board.c
 * date : 24.12.2014 11:27
 *  Author: Petr Pacner
*/

#define CRTSCTS  020000000000
#define MEASUREMENT_N 20 // constant - the number of measurement
#define dID 0x32 // define the ID of the device on the I2C Bus
#define _BSD_SOURCE
#define _XOPEN_SOURCE

#include <stdio.h>
#include <fcntl.h>
#include <stdint.h>
#include <errno.h> //errno, perror, strerror
#include <pthread.h> // THREADS (POSIX_library)
#include <unistd.h>  // TIMING
#include <termios.h> // SET_UART
#include <math.h>
#include <string.h>  // memcmp, memcpy
#include <assert.h>  // assert
#include <stdlib.h>  // EXIT_SUCCESS...
#include <wiringPiI2C.h> // I2C_ACCESS 

//variables
volatile int required = 0; //submited speed from user
volatile int speedA = 0; //current speed A
volatile int lspeedA = 0; //last speed A
volatile int speedB = 0; //current speed B
volatile int lspeedB = 0; //last speed B
//encoder 0xB0
volatile uint32_t a = 0; //actual number of counts
volatile uint32_t lastA = 0; //last number of counts
//encoder 0xC0
volatile uint32_t b = 0; //actual number of counts
volatile uint32_t lastB = 0; //last number of counts

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER; // static intialization of mutex

int maestroSetTarget(int fd, unsigned char channel, unsigned short target) {
  unsigned char command[] = {
    0xAA, 0xC, 0x04, channel, target & 0x7F, target >> 7 & 0x7F
  };

  if (write(fd, command, sizeof(command)) == -1) {
    perror("error writing");
    return -1;
  }
  return 0;
}


int maestroSetMotors (int fd, unsigned short Motor0, unsigned short Motor1) {
  unsigned char command[] = {
    0xAA, 0xC, 0x1F, 2, 0, Motor0 & 0x7F, Motor0 >> 7 & 0x7F, Motor1 & 0x7F, Motor1 >> 7 & 0x7F
  };

  if (write(fd, command, sizeof(command)) == -1) {
    perror("error writing");
    return -1;
  }
  return 0;
}

int readI2Cdata (int fd, unsigned char device) { 
unsigned char x = 0;
unsigned char periodNum = 0; // number of periods - first coming number from ATmega  8
uint32_t number = 0; // the value read from device
  
if(fd < 0) {
  printf("error opening I2C channel.\n");
}

else {
  if((wiringPiI2CWrite(fd, device))<0){
    printf("Writing to slave: %s [%i]\n", strerror(errno), errno);
  }
  
  if(( periodNum = wiringPiI2CRead (fd))<0){
    printf("ERROR reading form slave\n"); // response to previous request from RPi -   returned number of coming bytes in next loop
  }
  for (int i = periodNum; i>0; --i) {
    if(( x = wiringPiI2CRead (fd))<0){
      printf("ERROR reading from slave\n");
    }
    
    number = (number | ((x << ((periodNum-i)*8)))); // gathering function
  }
}
return number;
}

void *EncoderThreadAB(void *fd){
  int filedes;
  filedes = *(int *)fd;
  while(1){
    pthread_mutex_lock(&mutex);
    lastA = a;
    //lastB = b;
    a = readI2Cdata(filedes, 0xB0);
   //b = readI2Cdata(filedes, 0xC0);
    pthread_mutex_unlock(&mutex);
    usleep(20000);

    pthread_mutex_lock(&mutex);
    lastA = a;
    //lastB = b;
    //b = readI2Cdata(filedes, 0xC0);
    a = readI2Cdata(filedes, 0xB0);
    pthread_mutex_unlock(&mutex);
    usleep(20000);
  }
}

void *SpeedThreadA(){
  while(1){
    pthread_mutex_lock(&mutex);
    lspeedA = speedA; // last speed past to the last0 variable
    speedA = (a - lastA); // pulses per second
    pthread_mutex_unlock(&mutex);
    usleep(1000);
  }
}

 void *SpeedThreadB(){
   while(1){
    pthread_mutex_lock(&mutex);
    lspeedB = speedB; // last speed past to the last0 variable
    speedB = (b - lastB); // pulses per second
    pthread_mutex_unlock(&mutex);
    //usleep(10000);
   }
 }

void *RegulThreadA(void *fd){
  int filedes;
  int Tset = 5936;
  filedes = *(int *)fd;

  while(1){
    pthread_mutex_lock(&mutex);
    Tset += 0.1*(required-speedA)-0.09*(speedA-lspeedA);
    maestroSetTarget(filedes, 0, Tset);
    pthread_mutex_unlock(&mutex);
    usleep(10000);
  }
}

void *RegulThreadB(void *fd){                                                    
  int filedes;
  int Tset = 5936;
  filedes = *(int *)fd;
 
 while(1){
    pthread_mutex_lock(&mutex);
    Tset += 0.4*(required-speedB)-0.01*(speedB-lspeedB);
    maestroSetTarget(filedes, 1, Tset);
    pthread_mutex_unlock(&mutex);
    usleep(10000);
   }
 }
                        
int main(int argc, char *argv[]) {
    argc = argc; argv = argv;

//variables & constants
const char *device = "/dev/ttyAMA0"; // Linux
int fd = open(device, O_RDWR | O_NOCTTY);
int fd1;
if((fd1 = wiringPiI2CSetup(dID))<0){
  printf("ERROR opening I2C channel\n");
}

//pointers
pthread_t id0, id1, id2;
void *Atmfd;
void *mstfd;

Atmfd = &fd1; //pointer on file descriptor from ATmega
mstfd = &fd; //pointer on file descriptor from maestro

struct termios options;
tcgetattr(fd, &options);
cfsetispeed(&options, B9600);
cfsetospeed(&options, B9600);
  
options.c_cflag &= ~PARENB;
options.c_cflag &= ~CSTOPB;
options.c_cflag &= ~CSIZE;
options.c_cflag |= CS8;
  
// no flow control
options.c_cflag &= ~CRTSCTS;
 
options.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
options.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
  
options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
options.c_oflag &= ~OPOST; // make raw
  
// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
options.c_cc[VMIN] = 0;
options.c_cc[VTIME] = 20;

if (tcsetattr(fd, TCSANOW, &options) < 0){
  perror("init_serialport: Couldn't set term attributes");
  return -1;
}

if (fd == -1) {
  perror(device);
  return -1;
}

maestroSetMotors(fd, 5936, 5936);
//maestroSetTarget(fd, 0, 5936);

printf("submit the required speed (count/second):");
scanf(" %i", &required);

//printf("creating first thread\n");
pthread_create(&id0, NULL, &EncoderThreadAB, Atmfd); //measure actual value from enc.
//pthread_create(&id1, NULL, &SpeedThreadA, NULL); // compute the actual speedA
//pthread_create(&id1, NULL, &SpeedThreadB, NULL); // compute the actual speedB

//pthread_create(&id0, NULL, &RegulThreadA, mstfd); //compensation of current speed
//pthread_create(&id0, NULL, &RegulThreadB, mstfd); //compensation of current speed
//maestroSetMotors(fd, 7000, 7000);

while(1){
  pthread_mutex_lock(&mutex);
  printf("SPEED: %i, %i\n", a);
  pthread_mutex_unlock(&mutex);
  usleep(40000);
}
                              
if (close(fd))
  return EXIT_FAILURE;
else
  return EXIT_SUCCESS;
 
return 0;
}
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/* ATmega8_control_board.c
 * date : 24.12.2014 11:27
 *  Author: Idle-Rug
*/
// the reason of wrong speed counting could be in comunication on i2c

#define CRTSCTS  020000000000
#define MEASUREMENT_N 10 // constant - the number of measurement
#define dID 0x32 // define the ID of the device on the I2C Bus
#define _BSD_SOURCE
#define _XOPEN_SOURCE

#include <stdio.h>
#include <fcntl.h>
#include <stdint.h>
#include <errno.h> //errno, perror, strerror
#include <unistd.h>  // TIMING
#include <termios.h> // SET_UART
#include <math.h>
#include <string.h>  // memcmp, memcpy
#include <assert.h>  // assert
#include <stdlib.h>  // EXIT_SUCCESS...
#include <wiringPiI2C.h> // I2C_ACCESS 

int maestroGetPosition(int fd, unsigned char channel) {
  unsigned char command[] = {0xAA, 0xC, 0x10, channel};

  if(write(fd, command, sizeof(command)) == -1) {
    perror("error writing");
    return -1;
  }

  int n = 0;
  char response[2];
  do {
    int ec = read(fd, response+n, 1);

    if (ec < 0) {
      perror("error reading");
      return ec;
    }

    if (ec == 0) continue;

    n++;
  } while (n < 2);

  return response[0] + 256*response[1];
}

int maestroGetError(int fd) {
  unsigned char command[] = { 0xAA, 0xC, 0x21 };

  if (write(fd, command, sizeof(command)) != 3) {
    perror("error writing");
    return -1;
  }

  int n = 0;
  unsigned char response[2];

  do {
    int ec = read(fd, response+n, 1);

    if(ec < 0) {
      perror("error reading");
      return ec;
    }

    if (ec == 0) continue;

    n++;
  } while (n < 2);

  return (int)sqrt(response[0] + 256*response[1]);
}

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

// this function take carre of sorting the submited array
void insertionSort(int cislo, int pole[], int misto) {
  int k = misto;

  if (misto > 0) {
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

// function return the most frequented number from sensor
int filter_most_freq(int pole[], int pocet) {
  unsigned char nejvic = 0;
  int pomocPole[pocet];
  unsigned char k = 0;
  int soucet = 0;
  unsigned char delitel = 0;

  //seradi prvky ze zadaneho pole do pomocneho pole podle velikosti
  
  for (int i = 0; i < pocet; ++i) insertionSort(pole[i], pomocPole, i);

  // v prvnim projiti najde nejpocetnejsi cislo
  for(int i = 0; i < pocet; ++i)
  {
    if(pomocPole[i] != pomocPole[i+1])
    {
      ++k;
      if(k > nejvic)
      {
        nejvic = k; // ulozi nejpocetnejsi cislo pro aktualni cas
        soucet = pomocPole[i]; // ulozi hodnotu pole do promenne 'soucet'
        delitel = 1;
      }

      else if(k == nejvic)
      {
        soucet = soucet + pomocPole[i];
        ++delitel;
      }
      
      k = 0; //vynuluje promenno 'k' pro pocitani dalsich totoznych cisel
    }

    else{++k;}
  }

  return soucet/delitel;
}

// function arguments, fd - file descriptor; array - actual speed array; arrayL - last speed array
// function read new values from both encoders via I2C and save the last values in second arrayL
void readI2Cencoders (int fd, long array[2]){ 
unsigned char x = 0;
memset(array, 0, sizeof(long int)*2); 

if(fd < 0) {
  printf("error opening I2C channel.\n");
}

  if((wiringPiI2CWrite(fd, 0xB0))<0){
    printf("Writing to slave: %s [%i]\n", strerror(errno), errno);
  }
  
  for (int a = 0; a < 2; ++a) {
    for (int i = 4; i>0; --i) {
      if(( x = wiringPiI2CRead (fd))<0){
        printf("ERROR reading from slave\n");
      } 
      array[a] = (array[a] | ((x << ((4-i)*8)))); // gathering function
    }
  }
}

//function take the actual and last values from encoders and calculate the speed -> and save the new values in actSpeed and last values in lstSpeed
void speed(long actEnc[2], long lstEnc[2], int actSpeed[2], int lstSpeed[2]){
  for(int i = 0; i<2; ++i){
    lstSpeed[i] = actSpeed[i];
    actSpeed[i] = actEnc[i] - lstEnc[i];
  }
}

// motors regulation
void regulMotor(int fd, unsigned char required[2], int actSpeed[2], int lstSpeed[2], int signal [2]){ 
  for(int i=0; i<2; ++i){
  signal[i] += 1.9*(required[i]-actSpeed[i])-(actSpeed[i]-lstSpeed[i]); //1.9-1
  maestroSetTarget(fd, i, signal[i]);
  }
}
// read and transform data from sensor to sensors array
void sensor(int fd, int channel, int position, int array[4][MEASUREMENT_N], int distance[4]){
  int x;
  array[channel-2][position] = maestroGetPosition(fd, channel); //create new value in array
  x = filter_most_freq(array[channel-2], MEASUREMENT_N); //filtr the array
  if(x > 80 && x < 650){
    distance[channel-2] = (24141/(x-8))-0.17; //save the new distance value in sensor array - every element belong to one sensor -> 4 elemets in common
  }
  else distance[channel-2] = 0;
}

// function returns native values if the condition is executed
void sensorCheck(int pole[4], unsigned char x, int distance, unsigned char req_speed[2], int speed){
  if(pole[x] > 0 && pole[x] <= distance){
    req_speed[0] = speed;
    req_speed[1] = speed;
    unsigned char x = 0;
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
int PWM[] = {5936, 5936}; //array for controling the speed of motors
unsigned char reqSpeed[] = {0,0};
unsigned char position = 0;
unsigned char channel = 2; // switch variable to manage measurement - every cycle one measurement
unsigned char checknum = 0; //variable to check accure distance of given sensor
int degree = 0;

//arrays
long encodAct[] = {0,0};// encoders values position 0 - encoderA, 1 - encoderB
long encodLst[] = {0,0}; //last encoders vaules
int actSpeed[] = {0,0};
int lstSpeed[] = {0,0};

//sensors array
int sensors [4][MEASUREMENT_N];
// position:
// 0 - front_sensor
// 1 - right_sensor
// 2 - left_sensor
// 3 - back_sensor

// sensor distance array
int sensdist [] = {0,0,0,0};

struct termios options;
tcgetattr(fd, &options);
//it works with i2c baudrate 30000,38400(actual baud rate), 40000
cfsetispeed(&options, B57600); //57600
cfsetospeed(&options, B57600);

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

// filling sensors array with starting values
 for(int i = 0; i < MEASUREMENT_N; ++i){
    sensors[0][i] = maestroGetPosition(fd, 2);
    sensors[1][i] = maestroGetPosition(fd, 3);
    sensors[2][i] = maestroGetPosition(fd, 4);
    sensors[3][i] = maestroGetPosition(fd, 5);
 }
// native speed
reqSpeed[1] = 10;
reqSpeed[0] = 10;

for(; channel <= 6; ++channel){

 if(channel == 6){
   channel = 2;
   ++position;
 }

 if (position == MEASUREMENT_N) position = 0;

 encodLst[0] = encodAct[0];
 encodLst[1] = encodAct[1];
 readI2Cencoders(fd1, encodAct);
 sensor(fd, channel, position, sensors, sensdist);
 speed(encodAct, encodLst, actSpeed, lstSpeed);
 regulMotor(fd, reqSpeed, actSpeed, lstSpeed, PWM);

 if(sensdist[0] > 0 && sensdist[0] <= 200){
   reqSpeed[0] = 1;
   //reqSpeed[1] = 0;
   checknum = 1;
 }
/*
  if(sensdist[1] == 0){
    reqSpeed[0] = 3;
    checknum = 1;
  }
}
 
  else if(sensor[2] == 0){

  }

  else if(sensor[3] == 0){

  }

  else{
    //stop moving - you are surrounded with walls wait a minute if anything change 
  }
 
 }
*/

 if(checknum != 0){
   //sensorCheck(sensdist, checknum, 80, reqSpeed, 15);
   if(degree == 255){
     reqSpeed[0] = 0;
     reqSpeed[1] = 0;
     checknum = 0;
   }

  else if(actSpeed[0] == reqSpeed[0] || actSpeed[0] == reqSpeed[0]+1 || actSpeed[0] == reqSpeed[0]-1){++degree;}
 }

 printf("SPEED: %i, %i, %i\n", encodAct[0], encodAct[1], sensdist[0]);
 if(reqSpeed[0] == 0 && reqSpeed[1] == 0 && actSpeed[0] == 0 && actSpeed[1] == 0) break;
}

if (close(fd))
  return EXIT_FAILURE;
else
  return EXIT_SUCCESS;
 
return 0;
}
/*-----------------------------------------------------------------------------*/

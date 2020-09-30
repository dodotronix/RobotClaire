// FIXME add license!

/* ATmega8_control_board.c
 *
 * datum : 24.12.2014 11:27
 *  Author: Petr Pacner
 *
 *  program ma za ukol kontrolovat vsechny senzory na robotovi a podle toho vyhodnocovat danou situaci
 *  komunikuje z AddPU - (pridavnou deskou), ktera sbira informace z enkoderu, reflexnich senzoru,
 *  a potenciometru a posila je do RPi
 *
*/

/* senzory
 * konstanty byly zjisteny s pomoci programu gnuplot
 * rovnice pro linearizaci:
 * R = (1/(m*A+b))-k
 *
 * A - hodnota s a/d prevodniku
 * R - vzdalenost prekazky od senzoru
 * 
 * R = (1/(m*V+b))-k
 *
 * v celociselne matematice (integer math) se pri deleni ingoruji cisla za desetinnou carkou
 * -> proto je cela rovnice pro vypocet vzdalenosti prepsana s delenim
 * uprava pro pocitani s celymi cisly: R = (m'/(V+b'))-k, m' = 1/m; b' = b/m
 *
*/

#define CRTSCTS  020000000000
#define MEASUREMENT_N 20 // konstanta pro nastaveni mnozstvi mereni
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

// variables
long int a = 0;
uint32_t lastA = 0;
uint32_t b = 0;
uint32_t lastB = 0;

int speed0 = 0; //encoder 0xB0 
int last0 = 0;
int speed1 = 0; //encoder 0xC0
int last1 = 0;

int req_speed = 100;

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

void printPole(int pole[],int  pocet) {
  for (int x = 0; x < pocet; ++x) printf("cislo: %i\n", pole[x]);
}

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

int readI2Cdata (int fd, unsigned char device) {
  /*devices adresses
   * Motor0 - 0xB0
   * Motor1 - 0xC0
   * IR_ref_sensor0 == 0x00
   * IR_ref_sensor1 == 0x01
   * IR_ref_sensor2 == 0x02
   * Potenciometer0 == 0x03
  */

  unsigned char x = 0;
  unsigned char periodNum = 0; // number of periods - first coming number from ATmega8
  uint32_t number = 0; // the value read from device

  if(fd < 0) {
     printf("error opening I2C channel.\n");
   }
  
  else {

   if((wiringPiI2CWrite(fd, device))<0){
     printf("Writing to slave: %s [%i]\n", strerror(errno), errno);
   }

   if(( periodNum = wiringPiI2CRead (fd))<0){
     printf("ERROR reading form slave\n"); // response to previous request from RPi - returned number of coming bytes in next loop
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

// Encoder thread 
void *EncoderThread (void *fd){
  int filedes;
  filedes = *(int *)fd;
 while(1){
  lastA = a;
  lastB = b;
  a = readI2Cdata(filedes, 0xB0);
  b = readI2Cdata(filedes, 0xC0);
  usleep(20000);
  
  lastA = a;
  lastB = b;
  b = readI2Cdata(filedes, 0xC0); 
  a = readI2Cdata(filedes, 0xB0);
  usleep(20000);
 }
}

//actual speed motor0
void *SpeedThread0 (){
  while(1){
    last0 = speed0; // last speed past to the last0 variable
    speed0 = (a - lastA)*50; // pulses per second
  }
}

// actual speed motor1
void *SpeedThread1 (){
  while(1){
    last1 = speed1; // last speed past to the last1 variable
    speed1 = (b - lastB)*50; // pulses per second
  }
}

void *RegulationThreadEnc0 (void *fd){
  int target;
  int filedes;
  filedes = *(int *)fd;
  int curr_speed;
  int lastspeed;
  int Reqspeed;

  while(1){
    Reqspeed = req_speed; // load the actual values
    curr_speed = speed0;
    lastspeed = last0;
    target += 10*(Reqspeed - curr_speed) - 10*(curr_speed - lastspeed);
    maestroSetTarget(filedes, 1, target);
  }
}

int main(int argc, char *argv[]) {
  argc = argc; argv = argv;

  // Nastavi virtualni COM port pro Maestro
  const char *device = "/dev/ttyAMA0"; // Linux
  int fd = open(device, O_RDWR | O_NOCTTY);
  int fd1;
  if((fd1 = wiringPiI2CSetup(dID))<0){
    printf("ERROR opening I2C channel\n");
  }

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

  if (tcsetattr(fd, TCSANOW, &options) < 0) {
    perror("init_serialport: Couldn't set term attributes");
    return -1;
  }

  if (fd == -1) {
    perror(device);
    return -1;
  }

  maestroSetMotors(fd, 5936, 5936);
// maestroSetTarget(fd, 0, 6000);
// maestroSetTarget(fd, 1, 6000);
 
  //pointers
  pthread_t id0, id1, id2;
  void *atmegfd;
  void *maestfd;

  atmegfd = &fd1;
  maestfd = &fd;

  printf("creating first thread\n");
  //pthread_create(&id0, NULL,&EncoderThread, atmegfd);
  //pthread_create(&id1, NULL,&SpeedThread0, NULL);
  //pthread_create(&id2, NULL,&RegulationThreadEnc0, maestfd);
  //pthread_create(&id0, NULL,&EncoderThread, filedes);

  maestroSetTarget(fd, 0, 7000);
/*
   while(1){
     printf("SPEED: %i\n", speed0);
     usleep(1000);
  }
*/
/* debugging
  while(1){
    printf("output number: %i\n",readI2Cdata(fd1, 0xB0));
    usleep(20000);
  }
  //maestroSetMotors(fd, 6000, 6000);
*/
  if (close(fd))
    return EXIT_FAILURE;
  else
    return EXIT_SUCCESS;
}

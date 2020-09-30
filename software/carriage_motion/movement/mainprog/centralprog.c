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
#define dID 0x28 // define the ID of the device on the I2C Bus

#include <stdio.h>
#include <fcntl.h>
#include <pthread.h> // THREADS (POSIX_library)
#include <unistd.h>  // TIMING
#include <termios.h> // SET_UART
#include <math.h>
#include <string.h>  // memcmp, memcpy
#include <assert.h>  // assert
#include <stdlib.h>  // EXIT_SUCCESS...
#include <wiringPiI2C.h> // I2C_ACCESS

// konstanta pro nastaveni mnozstvi mereni
#define MEASUREMENT_N 20

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

/*
// total - total number of channels, first - first channel number, target - target value
int maestroMultiSetTarget(int fd, unsigned char total, unsigned char first, unsigned short target) {
  unsigned char command[] = {
    0xAA, 0xC, 0x1F, total, first, target & 0x7F, target >> 7 & 0x7F, target & 0x7F, target >> 7 & 0x7F
};

  if (write(fd, command, sizeof(command)) == -1) {
    perror("error writing");
    return -1;
  }

return 0;
}
*/

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
   int periodNum; // number of periods - first coming number from ATmega8                           
   long int number; // the value read from device                                
                                                                                 
   if(fd < 0) {                                                                  
      printf("error opening I2C channel.\n");                                    
    }                                                                            
                                                                                 
   else {                                                                        
     wiringPiI2CWrite (fd, device); // device is identificator for every input on the ATmega8
     periodNum = wiringPiI2CRead (fd);  // response to previous request from RPi - returned number        of coming bytes in next loop
     for (int i = periodNum; i>0; --i) { // value = number of period             
       number = (number | ((wiringPiI2CRead (fd) << ((periodNum-i)*8)))); // assembly the coming          number
     }                                                                           
   }                                                                             
   return number;                                                                
 }                                                                               

// submit acceleration constant, and final speed
int accelerate (int fd, int  acceleration, int finalSpeed) {
  int usleep();
  int const time = 100;
  int rest; // modulo
  int i = 0; // 
  int position = maestroGetPosition(fd, 0); // get first position

  finalSpeed = finalSpeed - position; // the variation of finalSpeed and actual position
  rest = finalSpeed % acceleration; //modulo of finalSpeed and acceleration

  for(; i < finalSpeed;) {
    i += acceleration; //increase acceleration parameter
    maestroSetMotors(fd, position+i, position+i); // figure up the starting position and actual acceleration share 'i'
    usleep(time); // time to wait
  }

  if (rest != 0) { // if modulo != 0 figure up the rest to the i and take off one more acceleration number
    i += rest-acceleration; // it's nessesery to take off once acceleration number from last cyclus
    maestroSetMotors(fd, position+i, position+i);
  }
  printf ("%i\n", i);

  return 0;
}

int deccelerate (int fd, int  decceleration, int finalSpeed) {
  int usleep();
  int const time = 100;
  int rest; // modulo
  int i = 0; 
  int position = maestroGetPosition(fd, 0); // get first position

  finalSpeed = position - finalSpeed; // the variation of finalSpeed and actual position
  rest = finalSpeed % decceleration; //modulo of finalSpeed and acceleration

  for(; i < finalSpeed;) {
    i += decceleration; //increase acceleration parameter
    maestroSetMotors(fd, position-i, position-i); // figure up the starting position and actual acceleration share 'i'
    usleep(time); // time to wait
  }

  if (rest != 0) { // if modulo != 0 figure up the rest to the i and take off one more acceleration number
    i += rest-decceleration; // it's nessesery to take off once acceleration number from last cyclus
    maestroSetMotors(fd, position-i, position-i);
  }
  printf ("%i\n", i);

  return 0;
}

int main(int argc, char *argv[]) {
  argc = argc; argv = argv;

  // Nastavi virtualni COM port pro Maestro
  const char *device = "/dev/ttyAMA0"; // Linux
  int fd = open(device, O_RDWR | O_NOCTTY);

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

  //I2C arguments an dID
  int fd1 = wiringPiI2CSetup(dID);
  int b;  // promenna b urcuje, kolik bude precteno bajtu
  int cislo;


  int senzor3[MEASUREMENT_N];
  memset(senzor3, 0, MEASUREMENT_N);
  unsigned char kanal = 4;
  int pozice = 0;
  int distance;
  int akcelerace = 0;
  char ques0;
  char ques1;
  unsigned char acc;
/*
  maestroSetMotors(fd, 6000, 6000);

  for (;;) {
   printf("Do you want to accelerate or deccelerate? (a/d) ");
   scanf("%s", &ques0);

   if (ques0 == 'a') {
     printf("submit the acceleration: ");
     scanf("%i", &acc);
     accelerate (fd, acc, 7000);
   }
   else if (ques0 == 'd') {
     printf("submit the decceleration: ");
     scanf("%i", &acc);
     deccelerate(fd, acc, 6000);
   }

  printf("Do you want to continue (y/n) ?"); 
  scanf("%s", &ques1);
  if (ques1 == 'y') {continue;}
  if(ques1 == 'n') { break;}

  }
*/

printf("I2C output: %i\n", readI2Cdata (fd1, 0xB0));
/*
  for (;pozice < MEASUREMENT_N;) {
    senzor3[pozice] = maestroGetPosition(fd, kanal);
    ++pozice;
  }
  pozice = 0;

  for (int i = 1; i < ((8000-6000)/akcelerace); ++i) {
    senzor3[pozice] = maestroGetPosition(fd, kanal);
    ++pozice;
    int x = filter_most_freq(senzor3, MEASUREMENT_N);
    distance = 32333.4/(x+21.3)+0.043;

    if(distance < 300) {
      


    }

    if (pozice == MEASUREMENT_N) pozice = 0;
    maestroSetMotors(fd,(akcelerace*i)+6000 ,(akcelerace*i)+6000);
  }
*/
  
/*
  for (;;) {
    senzor3[pozice] = maestroGetPosition(fd, kanal);
    ++pozice;
    int x = filter_most_freq(senzor3, MEASUREMENT_N);
//#if DEBUG
    printf("vyfiltrovane: %i\n", x);
//#endif

    if (pozice == MEASUREMENT_N) pozice = 0;
  }
*/

  if (close(fd))
    return EXIT_FAILURE;
  else
    return EXIT_SUCCESS;
}

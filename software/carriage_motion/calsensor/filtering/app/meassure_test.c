// FIXME add license!

#define CRTSCTS  020000000000

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <math.h>
#include <string.h>  // memcmp, memcpy
#include <assert.h>  // assert
#include <stdlib.h>  // EXIT_SUCCESS...

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

int filter_mean(int pole[], int pocet) {
  int res = 0;
  for (int i = 0; i < pocet; ++i) res += pole[i];
  return res / pocet;
}

int filter_median(int pole[], int pocet) {
  int pole2[pocet];
  //memcpy(pole2, pole, pocet);
  assert(pocet%2 == 0);

  for (int i = 0; i < pocet; ++i) insertionSort(pole[i], pole2, i);

  return (pole2[pocet/2 -1] + pole2[pocet/2]) / 2;
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
// FIXME 32333.4/(adhodnota+21.3)+0.043;

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

  FILE*fp;
  fp = fopen("/home/pi/programs/time_output.txt", "w");
  int b;
  int d;
  char ques1;
  int distance;
  int senzor3[MEASUREMENT_N]; // array used by filtr function
  memset(senzor3, 0, MEASUREMENT_N);
  unsigned char kanal = 4;
  int pozice = 0;


  //for(int i = 0; i < MEASUREMENT_N; ++i)
  //  senzor3[i] = maestroGetPosition(fd, kanal);

  for (;;) {
    //printf("zadeje vzdalenost(mm):");
    //scanf("%i", &distance); // insert distance -> save in file
/*
    pozice = 0;
// loading 20 elements to the array -> filtred the most freq
    for (int i=1;i<=20;++i) {
   // usleep(20000);//50Hz // for measure changes in time
    senzor3[pozice] = maestroGetPosition(fd, kanal); // place the meassured time on pozice
    ++pozice;
    x = filter_most_freq(senzor3, MEASUREMENT_N);
   // int x = maestroGetPosition(fd, kanal);
    printf("vyfiltrovane: %i\n", x);

   b = (24141/(x-8))-0.17; the best linearized output equation
   d = (23980/(x-8.42))+0.42;  
   printf("distance2: %i\n", b);
   printf("distance3: %i\n", d);
   senzor3[pozice] = maestroGetPosition(fd, kanal);
   printf("Do you want to continue (y/n) ?");
   scanf("%s", &ques1);
   if (ques1 == 'y') {continue;}
   if(ques1 == 'n') { break;}

*/
    senzor3[pozice] = maestroGetPosition(fd, kanal);
    ++pozice;
    //int x = filter_median(senzor3, MEASUREMENT_N);
    //int x = filter_mean(senzor3, MEASUREMENT_N);
    int x = filter_most_freq(senzor3, MEASUREMENT_N);
//#if DEBUG
    printf("vyfiltrovane: %i\n", x);
//#endif

    if (pozice == MEASUREMENT_N) pozice = 0;
  }

  if (close(fd))
    return EXIT_FAILURE;
  else
    return EXIT_SUCCESS;
}

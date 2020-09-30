//FIXME create licence
  /* ATmega8_control_board.c
   * date : 24.12.2014 11:27
   *  Author: Idle-Rug
  */
  // the reason of wrong speed counting could be in comunication on i2c

  #define CRTSCTS  020000000000
  #define MEASUREMENT_N 10 // constant - the number of measurement
  #define FORWARD_SPEED 4
  #define REVERSE_SPEED -4
  #define SKID_CONST 635 //slippery constant must be set depend on surface
  #define SENSITIVITY_D 6600  //the limit of decreasing motor PWM signal - undetected barrier
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

  //TODO make better (faster) array check 
  // function return the most frequented number from sensor
  int filter_most_freq(int pole[], int pocet) {
    unsigned char nejvic = 0;
    int pomocPole[pocet];
    unsigned char k = 0;
    int soucet = 0;
    unsigned char delitel = 1;

    //seradi prvky ze zadaneho pole do pomocneho pole podle velikosti
    
    for (int i = 0; i < pocet; ++i) insertionSort(pole[i], pomocPole, i);
      soucet = pomocPole[0]; //jestlize je v poli zastoupen jen jeden prvek fce vrati jeho hodnotu 
    for(int i = 0; i < pocet; ++i) //go thrue the array and count the most freq numbers
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

  // function arguments, fd - file descriptor; array - actual speed array; arrayL - last speed array
  // function read new values from both encoders via I2C and save the last values in second arrayL
  void readI2Cencoders (int fd, int16_t array[2]){ 
  unsigned char x = 0;
  memset(array, 0, sizeof(int16_t)*2); 

  if(fd < 0) {
    printf("error opening I2C channel.\n");
  }

    if((wiringPiI2CWrite(fd, 0xB0))<0){
      printf("Writing to slave: %s [%i]\n", strerror(errno), errno);
    }
    
    for (int a = 0; a < 2; ++a) {
      for (int i = 2; i>0; --i) {
        if(( x = wiringPiI2CRead (fd))<0){
          printf("ERROR reading from slave\n");
        } 
        array[a] = (array[a] | ((x << ((2-i)*8)))); // gathering function
      }
    }
  }

  //function take the actual and last values from encoders and calculate the speed -> and save the new values in actSpeed and last values in lstSpeed
  void speed(int16_t actEnc[2], int16_t lstEnc[2], int actSpeed[2], int lstSpeed[2]){ 
    for(int i = 0; i<2; ++i){ 
      lstSpeed[i] = actSpeed[i];
      //number 15 count/time is maximum speed of robot
      if(lstEnc[i] > 0 && lstEnc[i]+100 > 32767 && actEnc[i] < lstEnc[i]) 
         actSpeed[i] = (actEnc[i] - (lstEnc[i]-32767));
      else if(lstEnc[i] < 0 && lstEnc[i]-100 < -32767 && actEnc[i] > lstEnc[i]) 
         actSpeed[i] = (actEnc[i] - (32767+lstEnc[i]));
      else 
         actSpeed[i] = (actEnc[i] - lstEnc[i]); 
    }
  }

  // motors regulation
  void regulMotor(int fd, int16_t required[2], int actSpeed[2], int lstSpeed[2], int signal[2], float accel){ 
    signal[0] += accel*(required[0]-actSpeed[0])-(actSpeed[0]-lstSpeed[0]); //1.9-1
    signal[1] += accel*(required[1]-actSpeed[1])-(actSpeed[1]-lstSpeed[1]);
    maestroSetMotors(fd, signal[0], signal[1]);
    //maestroSetTarget(fd, 0, signal[0]);
    //maestroSetTarget(fd, 1, signal[1]);
  }

  // read and transform data from sensor to sensors array
  void sensor(int fd, unsigned char channel, unsigned char position, int array[4][MEASUREMENT_N], int distance[4], int lastdist[4]){
    int x;
    lastdist[channel-2] = distance[channel-2]; // save the last value
    array[channel-2][position] = maestroGetPosition(fd, channel); //create new value in array
    x = filter_most_freq(array[channel-2], MEASUREMENT_N); //filtr the array
    if(x > 100 && x < 600) distance[channel-2] = (24141/(x-8))-0.17; //save the new distance value in sensor array - every element belong to one sensor -> 4 elemets in common
    else if(x < 10) distance[channel-2] = 0;
}

// function returns native values if the condition is executed
void sensorCheck(int pole[4], unsigned char x, int distance, int16_t req_speed[2], int speed){
  if(pole[x] > 0 && pole[x] <= distance){
    req_speed[0] = speed;
    req_speed[1] = speed;
    x = 0;
  }
}

// function go straight, need diference of two status of every MOTOR
void straight(int16_t actEnc[2], int16_t lstEnc[2], int16_t reqSpeed[2], int16_t native, int16_t variable, int check[2]){
  int Value[2]; // variable for preserving actual value from encodersi
  int x;
  
  for(int i=0; i<2; ++i){
    //100 is constat for arbitrate if the comming value is overflow
    if(lstEnc[i]>0 && lstEnc[i]+100>32767 && lstEnc[i]>actEnc[i] && check[i] == 0)
       //Value[i] = actEnc[i]+32767;
       check[i] += 1;

    else if(lstEnc[i]<0 && lstEnc[i]-100<-32767 && lstEnc[i]<actEnc[i] && check[i] == 0)
       //Value[i] = actEnc[i]-32767;
       check[i] -= 1;
  }

  if((check[0] == 1 && check[1] == 1) || (check[0] == -1 && check[1] == -1)){
      check[0] = 0;
      check[1] = 0;
  }

   Value[0] = actEnc[0] + variable + check[0]*(32767);
   Value[1] = actEnc[1] + check[1]*(32767);

  if(native < 0) 
    x = -1;
  else
    x = 1;

  //equation -> controlling req_speed
  if(Value[0] < Value[1]){
    reqSpeed[1] = native - x;
    reqSpeed[0] = native;
  }

  else if(Value[0] > Value[1]){ 
    reqSpeed[0] = native - x;
    reqSpeed[1] = native;
  }

  else{
    reqSpeed[0] = native;
    reqSpeed[1] = native;
  }
}

//function return difference of both encoders
int difference(int16_t actEnc[2], int check[2]){
  check[0] = 0;
  check[1] = 0;
  
  return actEnc[1] - actEnc[0];
}

//FIXME this function is interrupting other moving function
// the robot has undetected barrier with sensor therefore it must immediately stop the motors
void stopbarr(int fd, int actSpeed[2], int16_t reqSpeed[2], unsigned char *checknum, int time[2], int PWM[2]){
  for(int i = 0; i<2; ++i){
    if(actSpeed[i] == 0 && PWM[i] > SENSITIVITY_D && reqSpeed[i] != 0){
      if(time[i] == 10){
        *checknum = 7; //the robot stuck on undetected barrieri
        time[0] = 0;
        time[1] = 0;
      }
      else time[i]+=1;
    }
     else time[i] = 0;
  }
}

void turnRound (int fd, int16_t reqSpeed[2], int actSpeed[2], int variable[2], int reqCount[2], int direction[2]){
   reqSpeed[0] = 0.0025*(reqCount[0] - variable[0]) + direction[0];
   reqSpeed[1] = 0.0025*(reqCount[1] - variable[1]) + direction[1];

   if(reqCount[0] == variable[0]){
     maestroSetTarget(fd, 0, 5892);
     reqSpeed[0] = 0;
   }
   else
     variable[0] += actSpeed[0];

   if(reqCount[1] == variable[1]){
     maestroSetTarget(fd, 1, 5892);
     reqSpeed[1] = 0;
   }
   else
     variable[1] += actSpeed[1];
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
int16_t reqSpeed[] = {0,0};
unsigned char position = 0;
unsigned char channel = 2; // switch variable to manage measurement - every cycle one measurement
unsigned char checknum = 0; //variable to check accure distance of given sensor
int degree = 0; //rotation control constant
float accel = 3; //1.9
int16_t diff; //difference value
int nativeSpeed = FORWARD_SPEED;
int sensnum = 0; //variable to save number of sensor

//pointers
unsigned char *checknumpt;
checknumpt = &checknum;

//arrays
int direction[2];
int variable[] = {0,0}; //variable to save sum of all count per time
int reqCount[] = {0,0}; //required count -> to control degree of motion 
int time[2];
int overflow[] = {0,0}; //array to check overflowing values from encoders
int PWM[] = {5892, 5892}; //array for controling the speed of motors
int16_t encodAct[] = {0,0};// encoders values position 0 - encoderA, 1 - encoderB
int16_t encodLst[] = {0,0}; //last encoders vaules
int actSpeed[] = {0,0};
int lstSpeed[] = {0,0};

//sensors array
int sensors [4][MEASUREMENT_N];
// position:
// 0 - front_sensor
// 1 - right_sensor
// 2 - left_sensor
// 3 - back_sensor

// actual sensor distance array
int sensdist [] = {0,0,0,0};
// last sensor distance array
int lastdist [] = {0,0,0,0};

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
// initialize motors -> 5936 is middle point
maestroSetMotors(fd, 5936, 5936);
// filling sensors array with starting values
 for(int i = 0; i < MEASUREMENT_N; ++i){
    sensors[0][i] = maestroGetPosition(fd, 2);
    sensors[1][i] = maestroGetPosition(fd, 3);
    sensors[2][i] = maestroGetPosition(fd, 4);
    sensors[3][i] = maestroGetPosition(fd, 5);
 }

//set required native speed
reqSpeed[0] = FORWARD_SPEED;
reqSpeed[1] = FORWARD_SPEED;
diff = difference(encodAct, overflow); //the start encoder difference (mainly 0)
variable[0] = 0;
variable[1] = 0;

//main cyclus with behaviour and controlling logic
for(; channel <= 6; ++channel){

if(sensdist[sensnum] > 0 && sensdist[sensnum] <= 290 && checknum == 0){
  reqSpeed[0] = 0;
  reqSpeed[1] = 0;
  checknum = 1; //stop moving
}

else if(checknum == 2){

  if(sensdist[1] >= 190 || sensdist[1] == 0){
    reqSpeed[0] = REVERSE_SPEED;
    reqSpeed[1] = FORWARD_SPEED;
    direction[0] = -1;
    direction[1] = 1;
    reqCount[0] = -SKID_CONST;
    reqCount[1] = SKID_CONST;
    checknum = 3;
  }

  else if(sensdist[2] >= 190 || sensdist[2] == 0){
    reqSpeed[0] = FORWARD_SPEED;
    reqSpeed[1] = REVERSE_SPEED;
    direction[0] = 1;
    direction[1] = -1;
    reqCount[0] = SKID_CONST;
    reqCount[1] = -SKID_CONST;
    checknum = 3;
  }

  else if(sensdist[3] >= 190 || sensdist[3] == 0){
    reqSpeed[0] = REVERSE_SPEED;
    reqSpeed[1] = REVERSE_SPEED;
    diff = difference(encodAct, overflow);
    checknum = 5;
  }

  else
    break; //exit program
}
else if(checknum == 7){
   reqSpeed[0] = 0;
   reqSpeed[1] = 0;
   maestroSetMotors(fd, 5936, 5936);
   checknum = 8;
}

 if(channel == 6){
   channel = 2;
   ++position;
   if(position == MEASUREMENT_N) position = 0;
 }

 stopbarr(fd, actSpeed, reqSpeed, checknumpt, time, PWM);
 encodLst[0] = encodAct[0];
 encodLst[1] = encodAct[1];
 readI2Cencoders(fd1, encodAct);
 sensor(fd, channel, position, sensors, sensdist, lastdist);
 speed(encodAct, encodLst, actSpeed, lstSpeed);
 regulMotor(fd, reqSpeed, actSpeed, lstSpeed, PWM, accel);

if(checknum == 0) //status 0 -> run straight 
   straight(encodAct, encodLst, reqSpeed, nativeSpeed, diff, overflow);

 // 85 turn around 90 degrees
 if(checknum != 0){
   //checknum 1 -> control deceleration
   if(checknum == 1 && actSpeed[0] == 0 && actSpeed[1] == 0){
     if(degree == 200){
      degree = 0;
      checknum = 2;
     }
     else ++degree;
  }

   //checknum 3 -> rotate control
   else if(checknum == 3){
   //218 == 90degrees
    if(reqCount[0] == variable[0] && reqCount[1] == variable[1]){
      reqSpeed[0] = 0;
      reqSpeed[1] = 0;
      checknum = 4;
    }
    else
      turnRound(fd, reqSpeed, actSpeed, variable, reqCount, direction);
   }

   //checknum 4 -> turn the cyclus on start status
   else if(checknum == 4 && actSpeed[0] == 0 && actSpeed[1] == 0){
       if(degree == 200){
         reqSpeed[0] = FORWARD_SPEED;
         reqSpeed[1] = FORWARD_SPEED;
         diff = difference(encodAct, overflow);
         variable[0] = 0;
         variable[1] = 0;
         checknum = 0;
         degree = 0;
    }
    else
      ++degree;
 }

   //reverse
   else if(checknum == 5){
     if(sensdist[1] == 0 || sensdist[1] >= 200){
       if(degree == 200){
         reqSpeed[0] = 0;
         reqSpeed[1] = 0;
         degree = 0;
         checknum = 6;
       }
       else ++degree;
     }
   }

   else if(checknum == 6 && actSpeed[0] == 0 && actSpeed[1] == 0){
     if(degree == 200){
       reqSpeed[0] = FORWARD_SPEED;
       reqSpeed[1] = REVERSE_SPEED;
       reqCount[0] = SKID_CONST;
       reqCount[1] = -SKID_CONST;
       checknum = 3;
       degree = 0;
     }
     else ++degree;
  }

  else if(checknum == 8){
    if(degree == 200){
      reqSpeed[0] = REVERSE_SPEED;
      reqSpeed[1] = REVERSE_SPEED;
      degree = 0;
      checknum = 5;
    }
    else ++degree;
  }
}

printf("SPEED: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", encodAct[0], encodAct[1], reqSpeed[0], reqSpeed[1], actSpeed[0], actSpeed[1], diff, sensdist[0], variable[0], variable[1], checknum, reqCount[0], reqCount[1]);
}

if (close(fd))
  return EXIT_FAILURE;
else
  return EXIT_SUCCESS;
 
return 0;
}
/*-----------------------------------------------------------------------------*/

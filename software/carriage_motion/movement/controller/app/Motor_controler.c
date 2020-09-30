/*-----------------------------------------------------------------sssssssss-*/
/*Motor_control							                                                 */
/* created by Idle-Rug					                              	             */

#define CRTSCTS  020000000000 

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <math.h>


// function channel save te user input to parameter channel
int channel()
{
  int chan;
  do
   {
      printf ("channel: ");
      scanf ("%i", &chan);
      if (chan < 0)
      {
         printf("Invalid input.Iteger couldn't be signed. Select new number.\n");
      }

      if (chan > 11)
      {
         printf("invalid input. Integer couldn't be bigger then number 11\n");
      }
    }while ((chan < 0) || (chan > 11));
             
 return chan;
}

// function signal save the user input to parameter signal
int signal ()
{
  int sig;
  do
   {
      printf ("signal: ");
      scanf ("%i", &sig);
      if (sig < 992)
      {
         printf("Invalid input.Iteger couldn't be signed. Select new number.\n");
      }

      if (sig > 2000)
      {
         printf("invalid input. Integer couldn't be bigger then number 11\n");
      }
    }while ((sig < 992) || (sig > 2000));
             
   return sig*4;
}


/**
* This command will return the error number on the Pololu Maestro Board
* The error number is made of two bytes. so the response needs to add the returned
* bytes together.
*
* To get the error we have to write 0xA1 to the controller to set it into error mode
* Then read the error
*
* @param int fd - The file descriptor to the device
*
* @returns int - The number that represents the Error. See Pololu documentation for error numbers
*/
int maestroGetError(int fd)
{
    unsigned char command[] = { 0xAA, 0xC, 0x21 };
    if (write(fd, command, sizeof(command)) != 3)
    {
        perror("error writing");
        return -1;
    }

    int n = 0;
    unsigned char response[2];
    do
    {
        int ec = read(fd, response+n, 1);
        if(ec < 0)
        {
            perror("error reading");
            return ec;
        }
        if (ec == 0)
        {
            continue;
        }
        n++;

    } while (n < 2);
    
    //Helpfull for debugging
    //printf("Error n: %d\n", n);
    //printf("Error secon: %d\n", response[1]);

    return (int)sqrt(response[0] + 256*response[1]);
}



/**
* This function is responsible for getting the current position of a servo
* which is identified by its channel. To get the current position of a channel,
* we first need to write 2 bytes to the Pololu board, where the first byte 0x90
* represents that we want to get the position, and the second byte represents the channel
* Once the write has been done, we need to read. Read returns two bytes representing one number
*
* @param int fd - The file descriptor to the device
* @param unsigned char channel - The channel number represented in 8 bit binary
*
* @returns int - The collation of two bytes as one single number.
*/
int maestroGetPosition(int fd, unsigned char channel)
{
    unsigned char command[] = {0xAA, 0xC, 0x10, channel};
    if(write(fd, command, sizeof(command)) == -1)
    {
        perror("error writing");
        return -1;
    }

    int n = 0;
    char response[2];
    do
    {
        int ec = read(fd, response+n, 1); //read 1 integer to buffer response+n; n change ever cyclus 
        if(ec < 0)
        {
            perror("error reading"); // if read response == -1 -> error
            return ec;
        }
        if (ec == 0)
        {
            continue;
        }
        n++;

    } while (n < 2);

    return response[0] + 256*response[1];
}



/**
* This function writes a new target to a given servo channel, To set a servo target
* the Pololu board requires 4 byte command where:
* byte1 - 0x84 - Telling the board that we are sending it set command
* byte2 - unsigned char - Telling the board which channel to set,
* byte3-byte4 - The collation of the numbers represent the target, as 2 bits are required to reach bigger values
*
* Its worth reading more on the Pololu documentation about the target bitsetting as you require
* bit shifting to provide a valid servo target command
*
* @param int fd - The file descriptor to the device
* @param unsigned char channel - The channel number represented in 8 bit binary
* @param unsigned short target - We can represent two bytes in a unsigned short target.
*
* @returns int - The collation of two bytes as one single number.
*/
int maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
    unsigned char command[] = {0xAA, 0xC, 0x04, channel, target & 0x7F, target >> 7 & 0x7F};
    if (write(fd, command, sizeof(command)) == -1)
    {
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
  return 0;
}

int deccelerate (int fd, int  decceleration, int finalSpeed) {
  int usleep();
  int const time = 100;
  int rest; // modulo
  int i = 0; 
  int position = maestroGetPosition(fd, 0); // get first position

  finalSpeed = position - finalSpeed; // the variation of finalSpeed and actual position
  rest = finalSpeed % decceleration; //modulo of finalSpeed and decceleration

  for(; i < finalSpeed;) {
    i += decceleration; //increase decceleration parameter
    maestroSetMotors(fd, position-i, position-i); // figure up the starting position and actual decceleration share 'i'
    usleep(time); // time to wait
  }

  if (rest != 0) { // if modulo != 0 figure up the rest to the i and take off one more decceleration number
    i += rest-decceleration; // it's nessesery to take off once acceleration number from last cyclus
    maestroSetMotors(fd, position-i, position-i);
  }
  return 0;
}


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


//---------------------------------------------------------------------------//
int main ()
{
   char ques;

 // Open the Maestro's virtual COM port.
    const char * device = "/dev/ttyAMA0"; // Linux
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

  for (;;)
  {
      if (tcsetattr(fd, TCSANOW, &options) < 0)
       {
            perror("init_serialport: Couldn't set term attributes");
            return -1;
        }
    
     if (fd == -1)
       {
            perror(device);
            return -1;
       }

      int error = maestroGetError(fd);
      printf("Errornumber is: %i\n", error);

      maestroSetTarget(fd, 0, 6000);
  char ques0;
  char ques1;
  unsigned char acc;

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
 





       }


return 0;
}

   
/*---------------------------------------------------------------------------*/


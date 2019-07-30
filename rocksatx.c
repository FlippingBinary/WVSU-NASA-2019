#include <unistd.h> 
#include <fcntl.h> 
#include <termios.h>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

uint64_t micros(struct timespec *tv) {
  return tv->tv_sec*(uint64_t)1000000+(tv->tv_nsec/1000);
}

int main() {
  struct timespec a, b;
  int sfd0 = open("/dev/serial0", O_RDWR | 
                         O_NOCTTY );
  if (sfd0 == -1){
    printf ("Failed to open /dev/serial0.\nError no is : %d\n", errno);
    printf("Error description is : %s\n",strerror(errno));
    return(-1);
  };
  int sfd1 = open("/dev/serial1", O_RDWR | 
                         O_NOCTTY );
  if (sfd1 == -1){
    printf("Failed to open /dev/serial1.\nError no is : %d\n", errno);
    printf("Error description is : %s\n",strerror(errno));
    return(-1);
  };
  
  struct termios options0;
  tcgetattr(sfd0, &options0);
  cfsetspeed(&options0, B57600);
  options0.c_cflag &= ~CSTOPB;
  options0.c_cflag |= CLOCAL;
  options0.c_cflag |= CREAD;
  options0.c_iflag |= IGNBRK;
  options0.c_iflag &= ~BRKINT;
  options0.c_iflag &= ~ICRNL;
  options0.c_oflag = 0;
  options0.c_lflag = 0;

  options0.c_cflag |= CS8;                             /* select 8 data bits */
  options0.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // choosing raw input
  options0.c_iflag &= ~INPCK;                          // disable parity check
  options0.c_iflag &= ~(IXON | IXOFF | IXANY);         // disable software flow control

  options0.c_cflag &= ~PARENB;
  options0.c_cflag &= ~CSTOPB;
  options0.c_cflag &= ~CSIZE;
  options0.c_cflag |= CS8;

  cfmakeraw(&options0);
  tcsetattr(sfd0, TCSANOW, &options0);

  struct termios options1;
  tcgetattr(sfd1, &options1);
  cfsetspeed(&options1, B57600);
  options1.c_cflag &= ~CSTOPB;
  options1.c_cflag |= CLOCAL;
  options1.c_cflag |= CREAD;
  options1.c_iflag |= IGNBRK;
  options1.c_iflag &= ~BRKINT;
  options1.c_iflag &= ~ICRNL;
  options1.c_oflag = 0;
  options1.c_lflag = 0;

  options1.c_cflag |= CS8;                             /* select 8 data bits */
  options1.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // choosing raw input
  options1.c_iflag &= ~INPCK;                          // disable parity check
  options1.c_iflag &= ~(IXON | IXOFF | IXANY);         // disable software flow control

  options1.c_cflag &= ~PARENB;
  options1.c_cflag &= ~CSTOPB;
  options1.c_cflag &= ~CSIZE;
  options1.c_cflag |= CS8;

  cfmakeraw(&options1);
  tcsetattr(sfd1, TCSANOW, &options1);
  
  
  printf("Entering main telemetry loop\n");
  fflush(stdout);
  
  uint8_t buf[512];
  int len;
  clock_gettime(CLOCK_REALTIME,&a);
  while ( 1 )
  {
    clock_gettime(CLOCK_REALTIME,&b);
    if ( micros(&b) - micros(&a) >= 15000 || micros(&b) < micros(&a) )
    {
      write(sfd0,"WVSU",4);
      write(sfd0,&(b.tv_sec),8);
      write(sfd0,"\xFF",1);
    }
    memset(buf, '\0', sizeof(buf));
    len = read(sfd0,buf,512);
    if ( len  > 0 )
    {
      write(sfd1,buf,len);

      FILE *write_ptr;

      write_ptr = fopen("rawtelemetry.dat","ab");

      if ( write_ptr == NULL ) {
        printf("Unable to open rawtelemetry.dat\n");
        fflush(stdout);
      } else
      {
        fwrite(buf,len,1,write_ptr);
        fflush(write_ptr);
        fclose(write_ptr);
      };
    };
    
  };
};

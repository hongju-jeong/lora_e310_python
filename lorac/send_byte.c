#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <wiringPi.h>
#include <signal.h>
#include <time.h>

typedef unsigned char byte;

#define TERMINAL    "/dev/ttyAMA0"
#define M0  4    //BCM GPIO 23
#define M1  5   //BCM GPIO 24

static int fd, imgfd;

void sending_mode() {
    digitalWrite(M0, LOW);
    printf("digitalRead (M0) : %d\n", digitalRead (M0));
    digitalWrite(M1, LOW);
    printf("digitalRead (M1) : %d\n", digitalRead (M1));
}

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void SigIntHandler(int sig)
{
    close(fd);
    close(imgfd);
    exit(0);
}

int main() {

    char *portname = TERMINAL;

    signal(SIGINT, SigIntHandler);

    
    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    imgfd = open("test.jpg", O_RDONLY);
    

    if (fd < 0) {
    printf("Error opening %s: %s\n", portname, strerror(errno));
    return -1;
    }
    

    if(wiringPiSetup() == -1){
        return 1;
    }

    pinMode(M0, OUTPUT);
    pinMode(M1, OUTPUT);

    sending_mode();

    set_interface_attribs(fd, B115200);

    byte send_buff[190];

    for(int i =0; i<190;i++) {
        send_buff[i] = 0x45;
    }
    
    int cnt = 0;
    int buff_size = 0;
    while((buff_size = read(imgfd, send_buff, 190)) > 0){
        printf("%d \n", buff_size);
        
        write(fd,send_buff,buff_size);
        tcdrain(fd);
        tcflush(fd,TCOFLUSH);
        usleep(14000);
    }

    close(fd);
    close(imgfd);
    

    return 0;
}


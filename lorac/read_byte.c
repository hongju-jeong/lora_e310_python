#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <wiringPi.h>
#include <signal.h>
#include <error.h>
#include <time.h>

#define TERMINAL    "/dev/ttyAMA0"
#define M0  4    //BCM GPIO 23
#define M1  5   //BCM GPIO 24

typedef unsigned char byte;

static int fd, text_fd;


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
    close(text_fd);
    exit(0);
}

int main() {

    char *portname = TERMINAL;
    clock_t start = clock();
    clock_t end;

    signal(SIGINT, SigIntHandler);

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    
    text_fd = open("test.jpg", O_RDWR | O_CREAT, 0644); //| O_APPEND

    if (text_fd < 0) {
        printf("Error opening %s: %s\n", "test.jpg", strerror(errno));
        return -1;
    }


    if(wiringPiSetup() == -1){
        return 1;
    }

    pinMode(M0, OUTPUT);
    pinMode(M1, OUTPUT);

    sending_mode();

    set_interface_attribs(fd, B115200);

    int count = 0;
    int rdlen = 0;

    while(1){
        end = clock();
        //printf("%lf \n", ((double)(end - start)/CLOCKS_PER_SEC));
        if(((double)(end - start)/CLOCKS_PER_SEC) >= 1){
            printf("%d \n", count);
            start = clock();
            count = 0;
        }
        byte send_buff[190];
        rdlen = read(fd,send_buff,sizeof(send_buff));
        //printf("%d \n", rdlen);
        if(rdlen > 0){
            write(text_fd, send_buff, rdlen);
        }
        count++;
    }
    // int rdlen;
    // byte send_buff[190];
    // rdlen = read(fd,send_buff,sizeof(send_buff)-1);
    // printf("%d %x \n",rdlen, send_buff[0]);


    return 0;
}


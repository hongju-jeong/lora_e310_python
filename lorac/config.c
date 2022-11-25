#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <wiringPi.h>

typedef unsigned char byte;

#define TERMINAL    "/dev/ttyAMA0"
#define M0  4    //BCM GPIO 23
#define M1  5   //BCM GPIO 24

byte now_addh = 0;
byte now_addl = 0;
byte now_netid = 0;
byte now_ch = 0;
byte now_baud = 0;
byte now_parity = 0;
byte now_airrate = 0;
byte now_mode = 0;
byte now_txpower = 0;

enum UART_BPS_TYPE {
	UART_BPS_1200 = 0b000,
	UART_BPS_2400 = 0b001,
	UART_BPS_4800 = 0b010,
	UART_BPS_9600 = 0b011,
	UART_BPS_19200 = 0b100,
	UART_BPS_38400 = 0b101,
	UART_BPS_57600 = 0b110,
	UART_BPS_115200 = 0b111
};

void print_baudrate() {
	switch (now_baud) {
	case UART_BPS_1200:
		printf("1200bps");
		break;
	case UART_BPS_2400:
		printf("2400bps");
		break;
	case UART_BPS_4800:
		printf("4800bps");
		break;
	case UART_BPS_9600:
		printf("9600bps (default)");
		break;
	case UART_BPS_19200:
		printf("19200bps");
		break;
	case UART_BPS_38400:
		printf("38400bps");
		break;
	case UART_BPS_57600:
		printf("57600bps");
		break;
	case UART_BPS_115200:
		printf("115200bps");
		break;
	default:
		printf("Invalid UART Baud Rate!");
	}
}

enum UART_PARITY {
	MODE_00_8N1 = 0b00,
	MODE_01_8O1 = 0b01,
	MODE_10_8E1 = 0b10,
	MODE_11_8N1 = 0b11
};

void print_parity() {
	switch (now_parity) {
	case MODE_00_8N1:
		printf("8N1 (Default)");
		break;
	case MODE_01_8O1:
		printf("8O1");
		break;
	case MODE_10_8E1:
		printf("8E1");
		break;
	case MODE_11_8N1:
		printf("8N1 (equal to 00)");
		break;
	default:
		printf("Invalid UART Parity!");
	}
}

enum AIR_DATA_RATE {
	AIR_DATA_RATE_000_12 = 0b000,
	AIR_DATA_RATE_001_24 = 0b001,
	AIR_DATA_RATE_010_48 = 0b010,
	AIR_DATA_RATE_011_96 = 0b011,
	AIR_DATA_RATE_100_192 = 0b100,
	AIR_DATA_RATE_101_384 = 0b101,
	AIR_DATA_RATE_110_700 = 0b110,
	AIR_DATA_RATE_111_1250 = 0b111
};

void print_airrate() {
	switch (now_airrate) {
	case AIR_DATA_RATE_000_12:
		printf("1.2kbps (default)");
		break;
	case AIR_DATA_RATE_001_24:
		printf("2.4kbps");
		break;
	case AIR_DATA_RATE_010_48:
		printf("4.8kbps");
		break;
	case AIR_DATA_RATE_011_96:
		printf("9.6kbps");
		break;
	case AIR_DATA_RATE_100_192:
		printf("19.2kbps");
		break;
	case AIR_DATA_RATE_101_384:
		printf("38.4kbps");
		break;
	case AIR_DATA_RATE_110_700:
		printf("70.0kbps");
		break;
	case AIR_DATA_RATE_111_1250:
		printf("125.0kbps");
		break;
	default:
		printf("Invalid Air Data Rate!");
	}
}

byte send_buf[4];

byte get_config(int fd, byte input[]) {
    byte recv_buf[4];
    write(fd, input, 3);
    read(fd, recv_buf, 4);
    if (recv_buf[0] == input[0] && recv_buf[1] == input[1] && recv_buf[2] == input[2]) {
        return recv_buf[3];
    } else {
        return 0xFF;
    }
}

byte set_config(int fd, byte input[]) {
    byte recv_buf[4];
    write(fd, input, 4);
    read(fd, recv_buf, 4);
    if (recv_buf[0] == 0xc1 && recv_buf[1] == input[1] && recv_buf[2] == input[2] && recv_buf[3] == input[3]) {
        return recv_buf[3];
    } else {
        return 0xFF;
    }
}

void get_configuration(int fd) {
    printf("-----------E310 Configuration-----------\n\n");

    printf("================ Before ================\n");
    printf("ADDH = ");
    send_buf[0] = 0xc1; send_buf[1] = 0x00; send_buf[2] = 0x01;
    now_addh = get_config(fd, send_buf);
    printf("0x%x\n\n", now_addh);
    
    printf("ADDL = ");
    send_buf[0] = 0xc1; send_buf[1] = 0x01; send_buf[2] = 0x01;
    now_addl = get_config(fd, send_buf);
    printf("0x%x\n\n", now_addl);

    printf("NETID = ");
    send_buf[0] = 0xc1; send_buf[1] = 0x02; send_buf[2] = 0x01;
    now_netid = get_config(fd, send_buf);
    printf("0x%x\n\n", now_netid);

    printf("CH = ");
    send_buf[0] = 0xc1; send_buf[1] = 0x05; send_buf[2] = 0x01;
    now_ch = get_config(fd, send_buf);
    printf("%f\n\n", 900 + now_ch * 0.2);

    send_buf[0] = 0xc0; send_buf[1] = 0x03; send_buf[2] = 0x01; // CH
    byte reg0 = get_config(fd, send_buf);
    now_baud = 0; now_parity = 0; now_airrate = 0;
    for (int i = 0; i < 8; i++){
        if (i < 3) {
            now_airrate *= 2;
            now_airrate += reg0 % 2;
        } else if (i >= 3 && i < 5) {
            now_parity *= 2;
            now_parity += reg0 % 2;
        } else {
            now_baud *= 2;
            now_baud += reg0 % 2;
        }
        reg0 /= 2;
    }
    printf("BAUD = ");
    print_baudrate();
    printf("\nPARITY = ");
    print_parity();
    printf("\nAIR_RATE = ");
    print_airrate();
    printf("\n======================================\n\n");
}

void configuration(int fd) {
    printf("-----------E310 Configuration-----------\n\n");

    printf("================ Before ================\n");
    printf("ADDH = ");
    send_buf[0] = 0xc1; send_buf[1] = 0x00; send_buf[2] = 0x01;
    now_addh = get_config(fd, send_buf);
    printf("0x%x\n\n", now_addh);
    
    printf("ADDL = ");
    send_buf[0] = 0xc1; send_buf[1] = 0x01; send_buf[2] = 0x01;
    now_addl = get_config(fd, send_buf);
    printf("0x%x\n\n", now_addl);

    printf("NETID = ");
    send_buf[0] = 0xc1; send_buf[1] = 0x02; send_buf[2] = 0x01;
    now_netid = get_config(fd, send_buf);
    printf("0x%x\n\n", now_netid);

    printf("CH = ");
    send_buf[0] = 0xc1; send_buf[1] = 0x05; send_buf[2] = 0x01;
    now_ch = get_config(fd, send_buf);
    printf("%f\n\n", 425.0 + now_ch * 0.2);

    send_buf[0] = 0xc0; send_buf[1] = 0x03; send_buf[2] = 0x01; // CH
    byte reg0 = get_config(fd, send_buf);
    now_baud = 0; now_parity = 0; now_airrate = 0;
    for (int i = 0; i < 8; i++){
        if (i < 3) {
            now_airrate *= 2;
            now_airrate += reg0 % 2;
        } else if (i >= 3 && i < 5) {
            now_parity *= 2;
            now_parity += reg0 % 2;
        } else {
            now_baud *= 2;
            now_baud += reg0 % 2;
        }
        reg0 /= 2;
    }
    printf("BAUD = ");
    print_baudrate();
    printf("\nPARITY = ");
    print_parity();
    printf("\nAIR_RATE = ");
    print_airrate();
    printf("\n======================================\n\n");

    printf("================ After =================\n");
    printf("modified ADDH=");
    send_buf[0] = 0xc0; send_buf[1] = 0x00; send_buf[2] = 0x01; send_buf[3] = 0x00; // ADDH
    now_addh = set_config(fd, send_buf);
    printf("0x%x\n\n", now_addh);

    printf("modified ADDL=");
    send_buf[0] = 0xc0; send_buf[1] = 0x01; send_buf[2] = 0x01; send_buf[3] = 0x02; // ADDL
    now_addl = set_config(fd, send_buf);
    printf("0x%x\n\n", now_addl);

    printf("modified NETID=");
    send_buf[0] = 0xc0; send_buf[1] = 0x02; send_buf[2] = 0x01; send_buf[3] = 0x00; // NETID
    now_netid = set_config(fd, send_buf);
    printf("0x%x\n\n", now_netid);

    printf("modified CH=");
    send_buf[0] = 0xc0; send_buf[1] = 0x05; send_buf[2] = 0x01; send_buf[3] = 0x41; // CH
    now_ch = set_config(fd, send_buf);
    printf("%f\n\n", 900.0 + now_ch * 0.2);

    send_buf[0] = 0xc0; send_buf[1] = 0x03; send_buf[2] = 0x01; send_buf[3] = 0b11111111; // baud(7,6,5), parity(4,3), airrate(2,1,0)
    reg0 = set_config(fd, send_buf);
    byte temp;
    while (reg0 > 0) {
        temp *= 2;
        printf("%d", reg0 % 2);
        temp += reg0 % 2;
        reg0 /= 2;
    }
    printf("\n");
    reg0 = temp;
    now_baud = 0; now_parity = 0; now_airrate = 0;
    for (int i = 0; i < 8; i++){
        if (i < 3) {
            now_airrate *= 2;
            now_airrate += reg0 % 2;
        } else if (i >= 3 && i < 5) {
            now_parity *= 2;
            now_parity += reg0 % 2;
        } else {
            now_baud *= 2;
            now_baud += reg0 % 2;
        }
        reg0 /= 2;
    }
    printf("modified BAUD = ");
    print_baudrate();
    printf("\nmodified PARITY = ");
    print_parity();
    printf("\nmodified AIR_RATE = ");
    print_airrate();
    printf("\n======================================\n\n");
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

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}

void config_mode() {
    digitalWrite(M0, LOW);
    printf("digitalRead (M0) : %d\n", digitalRead (M0));
    digitalWrite(M1, HIGH);
    printf("digitalRead (M1) : %d\n", digitalRead (M1));
}

int main()
{
    char *portname = TERMINAL;
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

    if(wiringPiSetup() == -1){
        return 1;
    }

    pinMode(M0, OUTPUT);
    pinMode(M1, OUTPUT);

    config_mode();



    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B9600);
    //set_mincount(fd, 0);                /* set to pure timed read */

    get_configuration(fd);

    close(fd);

}
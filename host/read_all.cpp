#include <errno.h>
#include <fcntl.h> 
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>

#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"

const double G_GRAVITY = 9.80665;

int set_interface_attribs(int fd, int speed, int parity)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        fprintf(stderr, "error %d from tcgetattr\n", errno);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "error %d from tcsetattr\n", errno);
        return -1;
    }
    return 0;
}

// read exactly 'len' bytes from fd
int read_bytes(int fd, uint8_t *buf, int len)
{
    while (len > 0) {
        int n = read(fd, buf, len);
        if (n > 0) {
            buf += n;
            len -= n;
        } else {
            break;
        }
    }
    return len;
}

void read_magnetometer(int fd)
{
    uint8_t buf[128] = {0};
    int res = 0;
    write(fd, "m", 1);
    res = read_bytes(fd, buf, 6);
    if (res > 0) {
        fprintf(stderr, ">> expected %d more byte(s)\n", res);
    }
    int16_t x, y, z;
    x = (buf[0] << 8) | buf[1];
    z = (buf[2] << 8) | buf[3];
    y = (buf[4] << 8) | buf[5];
    float fx, fy, fz;
    float mxy = 1100.0, mz = 980.0;
    fx = (x / mxy) * 100;
    fy = (y / mxy) * 100;
    fz = (z / mz) * 100;
    float heading = (atan2(y, x) * 180) / M_PI;
    if (heading < 0) {
        heading += 360;
    }
    printf(YEL "magnet: %6hd %6hd %6hd | %7.2f %7.2f %7.2f | heading: %7.2f\n", x, y, z, fx, fy, fz, heading);
}

void read_accelerometer(int fd)
{
    uint8_t buf[128] = {0};
    int res = 0;
    write(fd, "a", 1);
    res = read_bytes(fd, buf, 6);
    if (res > 0) {
        fprintf(stderr, ">> expected %d more byte(s)\n", res);
    }
    int16_t x, y, z;
    x = (buf[1] << 8) | buf[0];
    y = (buf[3] << 8) | buf[2];
    z = (buf[5] << 8) | buf[4];
    float fx = x * 0.001 * G_GRAVITY;
    float fy = y * 0.001 * G_GRAVITY;
    float fz = z * 0.001 * G_GRAVITY;
    printf(CYN " accel: %6hd %6hd %6hd | %7.2f %7.2f %7.2f\n", x, y, z, fx, fy, fz);
}

void read_temperature(int fd)
{
    uint8_t buf[128] = {0};
    int res = 0;
    write(fd, "t", 1);
    res = read_bytes(fd, buf, 2);
    if (res > 0) {
        fprintf(stderr, ">> expected %d more byte(s)\n", res);
    }
    int16_t temp = (buf[0] << 8) | buf[1];
    temp = temp >> 4;
    printf(GRN "temp: %02X %02X | value: %4d\n", buf[0], buf[1], temp);
}

int main(int argc, char *argv[])
{
    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "cannot open serial port\n");
        return 1;
    }
    set_interface_attribs(fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)

    while (true) {
        getchar();
        read_magnetometer(fd);
        read_accelerometer(fd);
        read_temperature(fd);
    }
}


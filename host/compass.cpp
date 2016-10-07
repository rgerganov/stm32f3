#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h> 
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>

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

double read_compass_heading(int fd, bool debug)
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
    double heading = (atan2(y, x) * 180) / M_PI;
    if (heading < 0) {
        heading += 360;
    }
    if (debug) {
        printf("magnet: %6hd %6hd %6hd | %7.2f %7.2f %7.2f | heading: %7.2f\n", x, y, z, fx, fy, fz, heading);
    }
    return heading;
}

int main(int argc, char *argv[])
{
    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "cannot open serial port\n");
        return 1;
    }
    // set speed to 115,200 bps, 8n1 (no parity)
    set_interface_attribs(fd, B115200, 0);

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Window *window = SDL_CreateWindow("Compass", SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED, 500, 500, SDL_WINDOW_SHOWN);
    if (window == NULL) {
        fprintf(stderr, "CreateWindow Error: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (renderer == NULL) {
        fprintf(stderr, "CreateRenderer Error: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Texture *compass = IMG_LoadTexture(renderer, "compass.png");
    if (compass == NULL) {
        fprintf(stderr, "Cannot load compass Error: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Event e;
    bool quit = false;
    double heading = 0;

    while (!quit) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                quit = true;
            } else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) {
                quit = true;
            }
        }
        SDL_SetRenderDrawColor(renderer, 0xdc, 0xdc, 0xdc, 0xFF);
        SDL_RenderClear(renderer);
        SDL_Point center = {250, 250};
        SDL_RendererFlip flip = SDL_FLIP_NONE;
        double new_heading = read_compass_heading(fd, false);
        if (fabs(heading - new_heading) > 3) {
            heading = new_heading;
        }
        SDL_RenderCopyEx(renderer, compass, NULL, NULL, -heading, &center, flip);
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

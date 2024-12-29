#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

static const char* device = "/dev/ttyAMA0";

int main(void) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Error opening serial port");
        return 1;
    }
    
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    tcsetattr(fd, TCSANOW, &options);

    char buffer[256];
    while (1) {
        int bytes_read = read(fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            printf("data received: %s\n", buffer);
        }
    }

    // never actually reached but whatever
    close(fd);
    return 0;
}

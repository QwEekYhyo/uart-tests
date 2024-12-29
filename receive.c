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
    
    // UART configuration
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200); // set input baudrate
    cfsetospeed(&options, B115200); // set output baudrate, unused for now
    options.c_cflag |= (CLOCAL | CREAD); // ignore modem signals and enable RX
    options.c_cflag &= ~CSIZE; // clear data bit size mask
    options.c_cflag |= CS8; // set word length to 8
    options.c_cflag &= ~PARENB; // no parity bit
    options.c_cflag &= ~CSTOPB; // only 1 stop bit
    options.c_cflag &= ~CRTSCTS; // disable hardware flow control
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // input handling
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // disable software flow control
    options.c_oflag &= ~OPOST; // disable output post-processing, unused for now
    tcsetattr(fd, TCSANOW, &options);

    fcntl(fd, F_SETFL, FNDELAY);

    char buffer[256];
    while (1) {
        int bytes_read = read(fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            printf("data received: %s\n", buffer);
        }
        usleep(100000);
    }

    // never actually reached but whatever
    close(fd);
    return 0;
}

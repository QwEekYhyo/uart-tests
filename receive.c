#include <fcntl.h>
#include <stdio.h>
#include <sys/epoll.h>
#include <termios.h>
#include <unistd.h>

#define MAX_EVENTS 10
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

    int epoll_fd = epoll_create1(0);
    if (epoll_fd == -1) {
        perror("Error creating epoll instance");
        close(fd);
        return 1;
    }

    struct epoll_event ev, events[MAX_EVENTS];
    ev.events = EPOLLIN;
    ev.data.fd = fd;
    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev) == -1) {
        perror("Error adding fd to epoll");
        close(fd);
        close(epoll_fd);
        return 1;
    }

    char buffer[256];
    while (1) {
        // this should always be one in our case
        int nb_file_descriptors = epoll_wait(epoll_fd, events, MAX_EVENTS, -1);
        if (nb_file_descriptors == -1) {
            perror("Error in epoll_wait");
            break;
        }

        for (int i = 0; i < nb_file_descriptors; i++) {
            // if serial port has data ready to be read
            if (events[i].events & EPOLLIN) {
                int bytes_read = read(events[i].data.fd, buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';
                    printf("data received: %s\n", buffer);
                }
            }
        }
    }

    // never actually reached but whatever
    close(fd);
    close(epoll_fd);
    return 0;
}

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

int main() {
    int uart0_filestream = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart0_filestream == -1) {
        std::cerr << "UART açılırken hata!" << std::endl;
        return -1;
    }
    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_filestream, TCIFLUSH);
    tcsetattr(uart0_filestream, TCSANOW, &options);

    while (true) {

        char buffer[256];
        int rx_length = read(uart0_filestream, (void*)buffer, 255);
        if (rx_length > 0) {
            buffer[rx_length] = '\0';
            std::cout << "Gelen veri: " << buffer << std::endl;
        }
        usleep(100000);
    }

    close(uart0_filestream);
    return 0;
}

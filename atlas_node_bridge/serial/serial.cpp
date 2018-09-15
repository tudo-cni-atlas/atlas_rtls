#include "serial.h"

#include <string>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>


Serial::Serial( std::string port, unsigned int baud )
{
    int fd;
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
    if (fd == -1)
    {
        std::cerr << "Error - Unable to open port " << port << std::endl;
        abort();
    }
    else
    {
        std::cout << "Port " << port << " opened at " << fd << std::endl;
        m_portfd = fd;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_oflag &= ~(OPOST);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF);
    tcsetattr(fd, TCSANOW, &options);

    fcntl(fd, F_SETFL, FNDELAY);

    usleep(1000);
    tcflush(fd, TCIOFLUSH);
    usleep(1000);
}

Serial::~Serial()
{
    std::cout << "Closed Port at " << m_portfd << std::endl;
    close(m_portfd);
}

void Serial::flush(void)
{
    usleep(1000);
    tcflush(m_portfd, TCIOFLUSH);
    usleep(1000);
}

void Serial::send( const uint8_t *buffer, size_t length )
{
    write(m_portfd, buffer, length);
}

int Serial::receive( uint8_t *c )
{
    int32_t available = 0;
    available = read(m_portfd, c, 1);
    return available;
}



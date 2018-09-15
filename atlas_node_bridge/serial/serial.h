#ifndef __SERIAL_INTERFACE_H_
#define __SERIAL_INTERFACE_H_

#include <string>

class Serial
{
public:
    Serial( std::string port, unsigned int baud );
    ~Serial();

    void flush(void);
    void send( const uint8_t *buffer, size_t length );
    int receive( uint8_t *c );

private:
    int m_portfd;
};

#endif
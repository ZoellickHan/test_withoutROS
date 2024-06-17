#include <vector>
#include <bitset>
#include "newDriver.hpp"
#include "crc.hpp"
#include "protocol.hpp"
#include <cstring>
#include <deque>
#include <iostream>
#include <chrono>

#define READ_BUFFER_SIZE 64
#define HEADER_SIZE 4
#define PROTOCOL_SIZE 64
#define MAX_BUFFER_SIZE 2048
#define PKG_SIZE 88

using namespace std;
using namespace rm_serial_driver;
using namespace newSerialDriver;

uint8_t readBuffer[PKG_SIZE];
uint8_t decodeBuffer[READ_BUFFER_SIZE];
deque<uint8_t> buffer;
int read_sum;
int error_sum;
uint8_t content[88];

Port::PkgState checkData()
{
    int size = buffer.size();

    if( size < PKG_SIZE )
    {
        return Port::PkgState::HEADER_INCOMPLETE;
    }
    for(int i = 0; i < size; i++)
    {
        if(buffer[i] == 0xAA)
        {
            if(buffer[i+1] == 0xAA && buffer[i+2] == 0xAA && buffer[i+3] == 0xAA && i + 3 < size)
            {
                // printf("boji\n");
                if( i + 3 + PKG_SIZE > size)
                {
                    buffer.erase(buffer.begin(),buffer.begin() + i);
                    return Port::PkgState::PAYLOAD_INCOMPLETE;
                }
                // i i+1 i+2 i+3 i+4
                for(int j = 0; j < PKG_SIZE; j++)
                {
                   if( buffer[j + i ] == content[j] )
                   {
                        printf("all :%d  decode error %d \n",read_sum,error_sum );
                   }
                   else
                   {
                        error_sum ++ ;
                   }
                }
            }
        }
    }
}

int main()
{
    shared_ptr<SerialConfig> config = make_shared<SerialConfig>(2000000,8,0,SerialConfig::StopBit::TWO,SerialConfig::Parity::NONE);
    shared_ptr<Port>         port   = make_shared<Port>(config);

    if(port->openPort())
    {
        port->init();
    }

    content[4] = 80;
    content[5] = 0;
    content[6] = 85;
    content[87] = 12;
    content[88] = 39;
    
    for(int i = 0; i < 90; i ++)
    {
        if(i < 4) 
            content[i] = 170;

        if( i >= 7 && i <= 86)
            content[i] = i - 7;
    }

    int num_per_read;
    while(true)
    {        
        num_per_read = read(port->fd,readBuffer,READ_BUFFER_SIZE);
        read_sum += num_per_read;
     
        if( num_per_read > 0)
        {
            buffer.insert(buffer.end(),readBuffer,readBuffer+num_per_read);
            checkData();
        }
        else
        {
            printf(" X( ");
        }
    }
    return 0;
}
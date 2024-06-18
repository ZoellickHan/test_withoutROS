#include <vector>
#include <bitset>
#include "newDriver.hpp"
#include "crc.hpp"
#include "protocol.hpp"
#include <cstring>
#include <deque>
#include <chrono>
#include <iostream>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <cstdlib>


#define READ_BUFFER_SIZE 64
#define HEADER_SIZE 4
#define PROTOCOL_SIZE 64
#define MAX_BUFFER_SIZE 2048
#define PKG_SIZE 91
#define HEADER_SIZE 4+3+2 

using namespace std;
using namespace chrono;
using namespace rm_serial_driver;
using namespace newSerialDriver;

uint8_t readBuffer[PKG_SIZE];
uint8_t decodeBuffer[PKG_SIZE];
deque<uint8_t> buffer;

rm_serial_driver::Header_4sof header_4sof;

uint8_t content[91] = 
{
  // Start word
  0xAA,0xAA,0xAA,0xAA,
  // Data length, little endian, dec 80, hex 0x0050
  0x50,0x00,
  // protocol id
  0x55,
  // CRC16
  0xBE, 0x88,
  // Payload, 0~79
  0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,
  0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,
  0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,
  0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,
  0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,0x30,0x31,
  0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,
  0x3C,0x3D,0x3E,0x3F,0x40,0x41,0x42,0x43,0x44,0x45,
  0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,
  // CRC16
  0x94,0x2A
  };

int read_sum;
int error_sum;
int num_per_read;
bool crc_ok_header = false;
bool crc_ok        = false;
auto start =  high_resolution_clock::now();
//i i1 i2 i3 // i4 i5 i6 i7 // i8
void ifusepipe()
{
    for(int i = 0; i < PKG_SIZE; i++)
    {
        if(decodeBuffer[i] != content[i])
        {
            error_sum++;
        }
        else{
            auto stop =  high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop-start);
            printf("all :%d  decode error %d time: %f ms \n",read_sum,error_sum,double(duration.count()/1000));
        }
    }
}

Port::PkgState decode()
{
    int size = buffer.size();
    if( size < HEADER_SIZE )
        return Port::PkgState::HEADER_INCOMPLETE;
    
    // cout<<"boji is here!"<<std::endl;
    for(int i = 0; i < size; i++)
    {
        if(buffer[i] == 0xAA)
        {
            if(buffer[i+1] == 0xAA && buffer[i+2] == 0xAA && buffer[i+3] == 0xAA && i + 3 < size)
            {
                std::copy(buffer.begin() + i, buffer.begin()+ i + HEADER_SIZE,decodeBuffer);
                crc_ok_header = crc16::Verify_CRC16_Check_Sum(decodeBuffer, HEADER_SIZE );

                if( !crc_ok_header )
                {
                    error_sum ++;
                    buffer.erase(buffer.begin() + i, buffer.begin() + i + HEADER_SIZE);
                    return Port::PkgState::CRC_HEADER_ERRROR;
                }

                header_4sof = convertToStruct<Header_4sof>(decodeBuffer);
                // pkg length = payload(dataLen) + header len (include header crc) + 2crc 
                if( i + (header_4sof.dataLen + sizeof(Header_4sof) + 2) > size )
                {
                    return Port::PkgState::PAYLOAD_INCOMPLETE;
                }

                // std::copy(buffer.begin() + i ,buffer.begin() + i + buffer[i+4], decodeBuffer);
                std::copy(buffer.begin() + i ,buffer.begin() + i + header_4sof.dataLen + sizeof(Header_4sof) + 2, decodeBuffer);
                crc_ok = crc16::Verify_CRC16_Check_Sum(decodeBuffer,PKG_SIZE);

                if(!crc_ok)
                {
                    error_sum ++;
                    buffer.erase(buffer.begin(), buffer.begin() + i + PKG_SIZE);
                    return Port::PkgState::CRC_PKG_ERROR;
                }

                ifusepipe(); // return the decode buffer;
                buffer.erase(buffer.begin(), buffer.begin() + i + PKG_SIZE);
                return Port::PkgState::COMPLETE;

            }
        }
    }

}

int main()
{
    // pid_t receive;
    // pid_t transmit;

    // uint8_t readPipe[MAX_BUFFER_SIZE];
    // uint8_t writePipe[MAX_BUFFER_SIZE];

    shared_ptr<SerialConfig> config = make_shared<SerialConfig>(2000000,8,0,SerialConfig::StopBit::TWO,SerialConfig::Parity::NONE);
    shared_ptr<Port>         port   = make_shared<Port>(config);

    if(port->openPort())
    {
        port->init();
    }

    while(true)
    {        
        num_per_read = read(port->fd,readBuffer,READ_BUFFER_SIZE);
        read_sum += num_per_read;
     
        if( num_per_read > 0)
        {
            buffer.insert(buffer.end(),readBuffer,readBuffer+num_per_read);
            decode();
        }
        else
        {
            printf(" X( ");
        }
    }
    return 0;
}
// void prepareContent()
// {
//     content[4] = 80;
//     content[5] = 0;
//     content[6] = 85;
//     content[87] = 12;
//     content[88] = 39;
    
//     for(int i = 0; i < 90; i ++)
//     {
//         if(i < 4) 
//             content[i] = 170;

//         if( i >= 7 && i <= 86)
//             content[i] = i - 7;
//     }
// }


// Port::PkgState checkData()
// {
//     int size = buffer.size();

//     if( size < PKG_SIZE )
//         return Port::PkgState::HEADER_INCOMPLETE;
    
    
//     for(int i = 0; i < size; i++)
//     {
//         if(buffer[i] == 0xAA)
//         {
//             if(buffer[i+1] == 0xAA && buffer[i+2] == 0xAA && buffer[i+3] == 0xAA && i + 3 < size)
//             {
//                 // printf("boji\n");
//                 if( i  + PKG_SIZE > size)
//                 {
//                     buffer.erase(buffer.begin(),buffer.begin() + i);
//                     return Port::PkgState::PAYLOAD_INCOMPLETE;
//                 }
//                 for(int j = 0; j < PKG_SIZE; j++)
//                 {
//                    if( buffer[j + i ] == content[j] )
//                    {
//                         printf("all :%d  decode error %d \n",read_sum,error_sum );
//                    }
//                    else
//                    {
//                         error_sum ++ ;
//                         buffer.erase(buffer.begin(),buffer.begin()+i+j);
//                         return Port::PkgState::CRC_PKG_ERROR;
//                    }
//                 }
//             }
//         }
//     }
// }
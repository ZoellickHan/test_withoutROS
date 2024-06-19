#include <vector>
#include <bitset>
#include "newDriver.hpp"
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
#include <cstdint>

#define READ_BUFFER_SIZE 64
#define HEADER_SIZE 4
#define PROTOCOL_SIZE 64
#define MAX_BUFFER_SIZE 2048
#define PKG_SIZE 91


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

/**
 * TODO:
 * 1.achieve transmit process
 * 2.achieve read process
 * 3.adjust the piroty
 * 4.monitor the state of the process.
 */

int read_sum;
int error_sum;
int num_per_read;
bool crc_ok_header = false;
bool crc_ok        = false;
auto start =  high_resolution_clock::now();


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

                header_4sof = arrayToStruct<Header_4sof>(decodeBuffer);
                // pkg length = payload(dataLen) + header len (include header crc) + 2crc 
                if( i + (header_4sof.dataLen + sizeof(Header_4sof) + 2) > size )
                {
                    return Port::PkgState::PAYLOAD_INCOMPLETE;
                }

                std::copy(buffer.begin() + i ,buffer.begin() + i + header_4sof.dataLen + sizeof(Header_4sof) + 2, decodeBuffer);
                crc_ok = crc16::Verify_CRC16_Check_Sum(decodeBuffer,PKG_SIZE);

                if(!crc_ok)
                {
                    error_sum ++;
                    buffer.erase(buffer.begin(), buffer.begin() + i + PKG_SIZE);
                    return Port::PkgState::CRC_PKG_ERROR;
                }

                // ifusepipe(); // return the decode buffer;
                buffer.erase(buffer.begin(), buffer.begin() + i + PKG_SIZE);
                return Port::PkgState::COMPLETE;

            }
        }
    }

}

void receiveProcess(int readPipefd[2])
{
    shared_ptr<SerialConfig> config = make_shared<SerialConfig>(2000000,8,0,SerialConfig::StopBit::TWO,SerialConfig::Parity::NONE);
    shared_ptr<Port>         port   = make_shared<Port>(config);
    if(port->openPort())
        port->init();

    while(true)
    {
        close;
    }

}

void transmitProcess(int writePipefd[2], int writeSize)
{
    shared_ptr<SerialConfig> config = make_shared<SerialConfig>(2000000,8,0,SerialConfig::StopBit::TWO,SerialConfig::Parity::NONE);
    shared_ptr<Port>         port   = make_shared<Port>(config);
    if(port->openPort())
        port->init();
    
    uint8_t data[MAX_BUFFER_SIZE];
    
    while(true)
    {
        close(writePipefd[1]);  //close the write

        if (read(writePipefd[0], data, writeSize * sizeof(uint8_t)) == -1) 
        {
            perror("read");
            close(writePipefd[0]);
            exit(EXIT_FAILURE);
            break;
        }

        if(port->transmit(data) < 0) 
            port->reopen();
    }    
}

void mainReceiveProcess()
{

}


void mainTransmitProcess(int readPipefd[2], int writePipefd[2])
{

    TwoCRC_ChassisCommand  twoCRC_ChassisCommand;
    TwoCRC_GimbalCommand   twoCRC_GimbalCommand;
    uint8_t buff1[sizeof(TwoCRC_ChassisCommand)],buff2[sizeof(TwoCRC_GimbalCommand)];
    // datalen : payload. = all - 2crc - header; 
    twoCRC_ChassisCommand.header.dataLen = 19 - 2 - sizeof(Header);
    twoCRC_ChassisCommand.vel_w = 0.5;
    twoCRC_ChassisCommand.vel_x = 0.5;
    twoCRC_ChassisCommand.vel_y = 0.5;
    twoCRC_ChassisCommand.header.crc_1 = 7;
    twoCRC_ChassisCommand.header.crc_2 = 7;
    twoCRC_ChassisCommand.header.protocolID = 3;
    twoCRC_ChassisCommand.header.sof = 170;

    twoCRC_GimbalCommand.header.dataLen  = sizeof(TwoCRC_GimbalCommand)  - 2 - sizeof(Header);
    twoCRC_GimbalCommand.shoot_mode   = 1;
    twoCRC_GimbalCommand.target_pitch = 0.5;
    twoCRC_GimbalCommand.target_yaw   = 0.5;

    structToArray(twoCRC_ChassisCommand,buff1);
    structToArray(twoCRC_GimbalCommand,buff2);

    crc16::Append_CRC16_Check_Sum(buff1,sizeof(Header));
    crc16::Append_CRC16_Check_Sum(buff2,sizeof(Header));

    crc16::Append_CRC16_Check_Sum(buff1,sizeof(TwoCRC_ChassisCommand));
    crc16::Append_CRC16_Check_Sum(buff2,sizeof(TwoCRC_GimbalCommand));

    //transmit 
    close(writePipefd[0]); //close the read

    if (write(writePipefd[1], buff1, sizeof(TwoCRC_ChassisCommand) * sizeof(uint8_t)) == -1) 
    {
        perror("write");
        close(writePipefd[1]);
        exit(EXIT_FAILURE);
    }

    if(write(writePipefd[1], buff2, sizeof(TwoCRC_GimbalCommand) * sizeof(uint8_t)) == -1)
    {
        perror("write");
        close(writePipefd[1]);
        exit(EXIT_FAILURE);        
    }
}


int main()
{
    //port
    int writeSize = sizeof(TwoCRC_ChassisCommand) + sizeof(TwoCRC_GimbalCommand);

    //process and pipe

    pid_t receive;
    pid_t transmit;

    int readPipefd[2];
    int writePipefd[2];

    if (pipe(readPipefd) == -1 || pipe(writePipefd) == -1) {
        std::cerr << "Pipe creation failed!" << std::endl;
        exit(1);
    }

    receive = fork();

    if(receive < 0)
    {
        printf(" receive process died... \n");
        exit(1);
    }
    else if (receive == 0)
    {
        printf(" start receive process with id : %d \n",getpid());
        while(true)
        {
           receiveProcess(readPipefd); 
        }
    }
    else
    {
        transmit = fork();
    }

    if(transmit < 0)
    {
        printf("transmit process died... \n");
        exit(1);
    }
   else if (transmit == 0)
   {
        printf(" start transmit process with id : %d \n", getpid());
        transmitProcess(writePipefd,writeSize);   
   }
   else
   {
        while(true)
        {
           mainTransmitProcess(readPipefd,writePipefd); 
        }
   }

    // while(true)
    // {        
    //     num_per_read = read(port->fd,readBuffer,READ_BUFFER_SIZE);
    //     read_sum += num_per_read;
     
    //     if( num_per_read > 0)
    //     {
    //         buffer.insert(buffer.end(),readBuffer,readBuffer+num_per_read);
    //         decode();
    //     }
    //     else
    //     {
    //         printf(" X( ");
    //     }
    // }

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


// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <stdint.h>
// #include <sys/types.h>
// #include <sys/wait.h>

// #define ARRAY_SIZE 10

// int main() {
//     int pipefd[2];
//     pid_t pid;
//     uint8_t data[ARRAY_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

//     // Create a pipe
//     if (pipe(pipefd) == -1) {
//         perror("pipe");
//         exit(EXIT_FAILURE);
//     }

//     // Fork the process
//     pid = fork();
//     if (pid == -1) 
//     {
//         perror("fork");
//         exit(EXIT_FAILURE);
//     }

//     if (pid > 0) {  // Parent process
//         // Close the read end of the pipe
//         close(pipefd[0]);

//         // Write the uint8_t array to the pipe
//         if (write(pipefd[1], data, ARRAY_SIZE * sizeof(uint8_t)) == -1) 
//         {
//             perror("write");
//             close(pipefd[1]);
//             exit(EXIT_FAILURE);
//         }

//         // Close the write end of the pipe
//         close(pipefd[1]);

//         // Wait for the child process to finish
//         wait(NULL);
//     } else {  // Child process
//         uint8_t buffer[ARRAY_SIZE];

//         // Close the write end of the pipe
//         close(pipefd[1]);

//         // Read from the pipe
//         if (read(pipefd[0], buffer, ARRAY_SIZE * sizeof(uint8_t)) == -1) 
//         {
//             perror("read");
//             close(pipefd[0]);
//             exit(EXIT_FAILURE);
//         }

//         // Close the read end of the pipe
//         close(pipefd[0]);

//         // Process the data (here we just print it)
//         printf("Child process received data:\n");
//         for (int i = 0; i < ARRAY_SIZE; i++) {
//             printf("%u ", buffer[i]);
//         }
//         printf("\n");
//     }

//     return 0;
// // }

// //i i1 i2 i3 // i4 i5 i6 i7 // i8
// void ifusepipe()
// {
//     for(int i = 0; i < PKG_SIZE; i++)
//     {
//         if(decodeBuffer[i] != content[i])
//         {
//             error_sum++;
//         }
//         else{
//             auto stop =  high_resolution_clock::now();
//             auto duration = duration_cast<microseconds>(stop-start);
//             printf("all :%d  decode error %d time: %f ms \n",read_sum,error_sum,double(duration.count()/1000));
//         }
//     }
// }
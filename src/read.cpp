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
#include <fcntl.h>

#define READ_BUFFER_SIZE 64
#define MAX_BUFFER_SIZE 2048

using namespace std;
using namespace chrono;
using namespace rm_serial_driver;
using namespace newSerialDriver;

uint8_t decodeBuffer[512];
rm_serial_driver::Header header;

/**
 * TODO:
 * 3.adjust the piroty
 * 4.monitor the state of the process.
 */

int read_sum;
int error_sum_header;
int error_sum_payload;
bool crc_ok_header = false;
bool crc_ok        = false;
auto start =  high_resolution_clock::now();
int bag_sum = 0;

void debuginfo()
{
    auto stop =  high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop-start);
    printf("crc rate : %f,bags : %d, error header %d error payload %d time: %f ms \n",
    float(error_sum_header + error_sum_payload)/float(bag_sum)*100,bag_sum,error_sum_header,error_sum_payload,double(duration.count()/1000));
}

Port::PkgState decode(std::deque<uint8_t> & buffer)
{
    int size = buffer.size();
    printf("size : %d \n",size);

    if( size < sizeof(Header) )
        return Port::PkgState::HEADER_INCOMPLETE;

    for(int i = 0; i < size; i++)
    {
        if(buffer[i] == 0xAA)
        {
            std::copy(buffer.begin() + i, buffer.begin()+ i + sizeof(Header), decodeBuffer);
            crc_ok_header = crc16::Verify_CRC16_Check_Sum(decodeBuffer, sizeof(Header) );

            if( !crc_ok_header )
            {
                error_sum_header ++;
                buffer.erase(buffer.begin() + i, buffer.begin() + i + sizeof(Header));
                memset(decodeBuffer,0x00,sizeof(decodeBuffer));
                return Port::PkgState::CRC_HEADER_ERRROR;
            }

            header = arrayToStruct<Header>(decodeBuffer);
            memset(decodeBuffer,0x00,sizeof(decodeBuffer));

            // pkg length = payload(dataLen) + header len (include header crc) + 2crc 
            if( i + (header.dataLen + sizeof(Header) + 2) > size )
            {
                bag_sum ++;
                return Port::PkgState::PAYLOAD_INCOMPLETE;
            }

            printf("id: %d  \n",header.protocolID);
            std::copy(buffer.begin() + i ,buffer.begin() + i + header.dataLen + sizeof(Header) + 2, decodeBuffer);
            crc_ok = crc16::Verify_CRC16_Check_Sum(decodeBuffer,header.dataLen + sizeof(Header) + 2);
            printf("crc ok : %d \n",crc_ok);

            if(!crc_ok)
            {
                error_sum_payload ++;
                memset(decodeBuffer,0x00,sizeof(decodeBuffer));
                buffer.erase(buffer.begin(), buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
                return Port::PkgState::CRC_PKG_ERROR;
            }

            debuginfo(); 
            memset(decodeBuffer,0x00,sizeof(decodeBuffer));
            buffer.erase(buffer.begin(), buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
            bag_sum ++;
            return Port::PkgState::COMPLETE;
        }
    }
}

void receiveProcess(int readPipefd[2])
{
    shared_ptr<SerialConfig> config = make_shared<SerialConfig>(2000000,8,0,SerialConfig::StopBit::TWO,SerialConfig::Parity::NONE);
    shared_ptr<Port>         port   = make_shared<Port>(config);
    port->openPort();
    uint8_t readBuffer[64];
    auto start2 = high_resolution_clock::now();
    int num_per_read = 0;

    while(true)
    {          
        num_per_read = read(port->fd,readBuffer,READ_BUFFER_SIZE); 
        if(num_per_read > 0)
        {
            close(readPipefd[0]);     //close the read
            
            if (write(readPipefd[1], readBuffer, num_per_read * sizeof(uint8_t)) == -1) 
            {
                perror("write");
                close(readPipefd[1]);
                exit(EXIT_FAILURE);
            }

            auto stop2 =  high_resolution_clock::now();
            auto duration2 = duration_cast<microseconds>(stop2-start2);
            read_sum += num_per_read;
        }      
    }
}

void transmitProcess(int writePipefd[2], int writeSize)
{
    auto stop =  high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop-start);

    int num_per_write = 0;
    shared_ptr<SerialConfig> config = make_shared<SerialConfig>(2000000,8,0,SerialConfig::StopBit::TWO,SerialConfig::Parity::NONE);
    shared_ptr<Port>         port   = make_shared<Port>(config);
    if(port->openPort())
    {
       
    }
    else{
        cout << "Open port fail" << endl;
    }
       
    
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
        num_per_write = write(port->fd,data,writeSize);
    }    
}

void mainReceiveFromProcess(int readPipefd[2], std::deque<uint8_t>& buffer)
{
    int per_read = 0;
    uint8_t receiveBuffer[MAX_BUFFER_SIZE];

    close(readPipefd[1]); // close the write

    if ((per_read = read(readPipefd[0], receiveBuffer, READ_BUFFER_SIZE * sizeof(uint8_t))) == -1) 
    {
        perror("read");
        close(readPipefd[0]);
        exit(EXIT_FAILURE);
    }

    cout<<endl;

    buffer.insert(buffer.end(),receiveBuffer,receiveBuffer + per_read);
    decode(buffer);
}


void mainTransmitToProcess(int writePipefd[2])
{
    while (true)
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

        rm_serial_driver::structToArray(twoCRC_ChassisCommand,buff1);
        rm_serial_driver::structToArray(twoCRC_GimbalCommand,buff2);

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

}

int main()
{
    deque<uint8_t> buffer;
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
        //receive from ch343
        printf(" start receive process with id : %d \n",getpid());
        receiveProcess(readPipefd); 
    }
    else
    {
        transmit = fork();

        if(transmit < 0)
        {
            printf("transmit process died... \n");
            exit(1);
        }
        else if (transmit == 0)
        {
            //transmit to ch343
            printf(" start transmit process receivewith id : %d \n", getpid());
            transmitProcess(writePipefd,writeSize);   
        }
        else
        {
            thread test_transmit = thread(mainTransmitToProcess,writePipefd);
            while(true)
            {
                mainReceiveFromProcess(readPipefd,buffer);
            }
        }
    }
    return 0;
}
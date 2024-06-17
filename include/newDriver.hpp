// C++ system
#include <vector>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include "protocol.hpp"
#include "crc.hpp" 

#define ROSCOMM_BUFFER_SIZE 2048
#define BUFFER_SIZE 2048
namespace newSerialDriver
{
class  SerialConfig
{  
public: 

    enum StopBit : uint8_t
    {
        ONE = 0,
        ONE_POINT_FIVE,
        TWO
    };

    enum Parity : uint8_t
    {
        NONE = 0,
        ODD,
        EVEN,
        MARK,
        SPACE
    };

    SerialConfig() = delete;
    SerialConfig(int bps, int databit, bool flow, StopBit stopbits, Parity paritys)
    {
        baudrate = bps;
        databits = databit;
        flowcontrol = flow;
        stopbit = stopbits;
        parity  = paritys;
    }
    ~SerialConfig();
	char* devname = (char*)"/dev/ttyCH343USB0";
    int baudrate = 2000000;
    int databits = 8;
    bool flowcontrol = 0;
    StopBit stopbit = StopBit::ONE;
    Parity  parity  = Parity::NONE;
};

class Port
{
public:

    enum PkgState : uint8_t
    {
        COMPLETE = 0,
        HEADER_INCOMPLETE,
        PAYLOAD_INCOMPLETE,
        CRC_HEADER_ERRROR,
        CRC_PKG_ERROR,
    };

    Port(std::shared_ptr<newSerialDriver::SerialConfig> ptr);
    ~Port();

    // port function 
    bool init();
    int  openPort();
    bool closePort();
    bool reopen();
    bool isPortInit();
    bool isPortOpen();

    // rx tx function
    int  transmit(std::vector<uint8_t> & buff);  
    int  receive();

    void readFun();
    void decodeFun(int ID);
    void decodeThreadFun();
    void putinIndexFun(int size);
    PkgState putoutIndexFun();

    
    // data and debug function
    int& geterrorHeader(){return error_header_count;}
    int& geterrorData(){return error_data_count;}
    int& getdecodeCount(){return decodeCount;}
    int& getputinIndex(){return putinIndex;}
    int& getputoutIndex(){return putoutIndex;}

    rm_serial_driver::TwoCRC_GimbalMsg& getTwoCRC_GimbalMsg(){return twoCRC_GimbalMsg;}
    rm_serial_driver::TwoCRC_SentryGimbalMsg& getTwoCRC_SentryGimbalMsg(){return twoCRC_SentryGimbalMsg;} 

    std::thread  readThread; 
    std::thread  decodeThread;
    int fd;
    
private:

    // port data
    std::shared_ptr<SerialConfig> config;

    int num_per_read        = 0;
    int num_per_write       = 0;

    bool isinit          = false;
    bool isopen          = false;
    

    //rx thread
    PkgState            frameState;
    // state data
    int rxsize              = 0;
    int putinIndex          = -1;
    int putoutIndex         = -1;
    int decodeCount         = 0;
    int error_header_count  = 0;
    int error_data_count    = 0;

    bool crc_ok_header   = false;
    bool crc_ok          = false;
    //rx tx data
    std::vector<uint8_t> transformVector;
    std::vector<uint8_t> decodeVector;
    std::vector<uint8_t> circlur;

    uint8_t readBuffer[BUFFER_SIZE];
    uint8_t RxBuff[ROSCOMM_BUFFER_SIZE];
    uint8_t TxBuff[ROSCOMM_BUFFER_SIZE];
    uint8_t frameBuffer[BUFFER_SIZE];

    rm_serial_driver::Header header;
    rm_serial_driver::TwoCRC_GimbalMsg twoCRC_GimbalMsg;
    rm_serial_driver::TwoCRC_SentryGimbalMsg twoCRC_SentryGimbalMsg;
};

}//newSerialDriver

#pragma once
#include <serial/serial.h>
#include <queue>

class can_frame
{
private:
    /* data */
    uint8_t data[8];
    uint8_t ID[2];
    uint8_t DLC;
    uint8_t Hex[11];    
public:
    can_frame(const uint8_t* id_,const uint8_t dlc_,const uint8_t* data_);
    ~can_frame();
    const uint8_t* getData();
    const uint8_t* getID();
    const uint8_t getDLC();
    const uint8_t* getHex();
    std::string getStr();
};

class Usb_can : serial::Serial
{
private:
    /* data */
    uint8_t frame_buffer[2048];
    std::queue<can_frame*> frame_queue;
    size_t head;
    size_t extract_can_frame();
public:
    /*
    baud_rate(115200) -> CAN port kbps(500)
    parity_bit(ODD) -> switch mode(normal work mode)
    data_bit -> CANFD sending magnification
    stop_bit(1) -> switch state(CAN)
    */
    Usb_can(const std::string &port_name,uint32_t baud_rate,serial::bytesize_t byte_size,serial::stopbits_t stop_bits,serial::parity_t parity_bit,serial::Timeout timeout);
    
    ~Usb_can();
    
    /*
    open the usb port
    */
    void open();
    
    /*
    close the usb port
    delete all the can_frame_ptr
    */
    void close();
    
    /*
    return the port state
    */
    bool isOpen();
    
    /*
    write a standard data frame
    Head    Flag          ID                DLC         Data    Tail
    0xAA    0x55    0x0000~0x07EF       0x00~0x08         -     0xDD
    */
    bool write(const uint8_t* ID,const uint8_t DLC,const uint8_t* data);
    
    /*
    read bytes from usb port ,extract can_frame and
    return the available complete can_frame number
    */
    size_t available();

    /*
    return the latest can_frame
    */
    can_frame* read();
};

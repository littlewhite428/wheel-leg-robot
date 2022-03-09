#include "usb_can.h"

Usb_can::Usb_can(const std::string &port_name,uint32_t baud_rate,serial::bytesize_t byte_size,serial::stopbits_t stop_bits,serial::parity_t parity_bit,serial::Timeout timeout)
{
    serial::Serial(port_name,baud_rate,timeout,byte_size,parity_bit,stop_bits);
    setPort(port_name);
    setBaudrate(baud_rate);
    setBytesize(byte_size);
    setTimeout(timeout);
    setStopbits(stop_bits);
    setParity(parity_bit);
}

Usb_can::~Usb_can()
{
}

bool Usb_can::write(uint8_t* ID,uint8_t DLC,uint8_t* data){
    uint32_t total_frame_size = 6 + DLC;
    uint8_t can_data[total_frame_size];
    can_data[0] = 0xAA;
    can_data[1] = 0x55;
    memcpy(can_data+2,ID,2);
    can_data[4] =  DLC;
    memcpy(can_data+5,data,DLC);
    can_data[total_frame_size-1]=0xDD;
    Serial::write(can_data,total_frame_size);
    return true;

    // uint8_t data_all[8];
    // data_all[0] = 0xAA;
    // data_all[1] = 0x55;
    // data_all[2] = 0x00;
    // data_all[3] = 0x00;
    // data_all[4] = 0x02;
    // data_all[5] = 0xCC;
    // data_all[6] = 0xCC;
    // data_all[7] = 0xDD;
    // Serial::write(data_all,8);
    return true;
}

void Usb_can::open(){
    Serial::open();    
}

bool Usb_can::isOpen(){
    return Serial::isOpen();
}

void Usb_can::close(){
    Serial::close();
}
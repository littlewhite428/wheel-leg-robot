#include "usb_can.h"

Usb_can::Usb_can(const std::string &port_name,uint32_t baud_rate,serial::bytesize_t byte_size,serial::stopbits_t stop_bits,serial::parity_t parity_bit,serial::Timeout timeout)
{
    serial::Serial();
    setPort(port_name);
    setBaudrate(baud_rate);
    setBytesize(byte_size);
    setTimeout(timeout);
    setStopbits(stop_bits);
    setParity(parity_bit);
    head = 0;
}

Usb_can::~Usb_can()
{
    while(!frame_queue.empty()){
        delete frame_queue.front();
        frame_queue.pop();
    }
}

bool Usb_can::write(const uint8_t* ID,const uint8_t DLC,const uint8_t* data){
    if(DLC<1 || DLC >8){
        return false;
    }
    else{
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
    }
}

void Usb_can::open(){
    Serial::open();    
}

bool Usb_can::isOpen(){
    return Serial::isOpen();
}

void Usb_can::close(){
    while(!frame_queue.empty()){
        delete frame_queue.front();
        frame_queue.pop();
    }
    Serial::close();
}
/*
在frame_buffer中提取完整帧数据
返回处理的完整帧数据的字节数
*/
size_t Usb_can::extract_can_frame(){
    int i = 0;
    size_t num = 0;
    for(i=0;i<head;i++){
        if(frame_buffer[i]==0xBB){
            if(frame_buffer[i+5]==0x55){
                uint8_t temp_data_len = frame_buffer[i+8];
                if(frame_buffer[i+8+temp_data_len+1]==0xEE){
                    frame_queue.push(new can_frame(frame_buffer+i+6,temp_data_len,frame_buffer+i+9));
                    i = i + 9 + temp_data_len;
                    num = i + 1;
                }
            }
        }
    }
    return num;
}

/*
从串口缓冲区读取数据，将完整帧提取到队列中
残留帧留在缓冲区，返回当前队列长度
*/
size_t Usb_can::available(){
    size_t available_length = Serial::available();
    if(available_length > 0){
        Serial::read(frame_buffer+head,available_length);
        head += available_length;
        size_t complete_frame_bytes = extract_can_frame();
        memcpy(frame_buffer,frame_buffer + complete_frame_bytes,head - complete_frame_bytes);
        head -= complete_frame_bytes;
    }
    return frame_queue.size();

}
can_frame* Usb_can::read(){
    can_frame* temp_p  = frame_queue.front();
    frame_queue.pop();
    return temp_p;
}


can_frame::can_frame(const uint8_t* id_,const uint8_t dlc_,const uint8_t* data_)
{
    memcpy(ID,id_,2);
    memcpy(data,data_,dlc_);
    DLC = dlc_;
    memcpy(Hex,id_,2);
    Hex[2] = dlc_;
    memcpy(Hex+3,data_,dlc_);
}

can_frame::~can_frame()
{
}
const uint8_t* can_frame::getData(){
    return data;
}
const uint8_t* can_frame::getID(){
    return ID;
}
const uint8_t can_frame::getDLC(){
    return DLC;
}
const uint8_t* can_frame::getHex(){
    return Hex;
}
std::string can_frame::getStr(){
    char temp_str[30];
    sprintf(temp_str,"%02X%02X",Hex[0],Hex[1]);
    sprintf(temp_str+4," %02X ",Hex[2]);
    for(int i=0;i<DLC;i++){
        sprintf(temp_str+8+2*i,"%02X",Hex[i+3]);
    }
    return std::string(temp_str);
}

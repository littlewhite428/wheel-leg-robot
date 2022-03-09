#include <serial/serial.h>
class Usb_can : serial::Serial
{
private:
    /* data */

public:
    /*
    baud_rate(115200) -> CAN port kbps(500)
    parity_bit(ODD) -> switch mode(normal work mode)
    data_bit -> CANFD sending magnification
    stop_bit(1) -> switch state(CAN)
    */
    Usb_can(const std::string &port_name,uint32_t baud_rate,serial::bytesize_t byte_size,serial::stopbits_t stop_bits,serial::parity_t parity_bit,serial::Timeout timeout);
    ~Usb_can();
    void open();
    void close();
    bool isOpen();
    /*
    write a standard data frame
    Head    Flag          ID                DLC         Data    Tail
    0xAA    0x55    0x0000~0x07EF       0x00~0x08         -     0xDD
    */
    bool write(uint8_t* ID,uint8_t DLC,uint8_t* data);
};

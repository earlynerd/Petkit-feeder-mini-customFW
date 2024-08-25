#ifndef _PETKITBUS_H_
#define _PETKITBUS_H_

#include <Arduino.h>

// workflow
// user calls sendcommand api with the command type and any data
// command is serialized and transmitted
// command is copied into buffer and responses arsed to determine of command was acked or not
// if no ack within timeout retry

class PetkitBus
{
public:
    PetkitBus(HardwareSerial* serial);
    void init();
    void beep(uint16_t, uint16_t, uint16_t);
    void blink(uint8_t, uint16_t, uint16_t, uint16_t);
    void open();
    void close();
    void dispense(uint8_t, uint8_t, uint8_t, uint8_t);
    
    
    enum CommandType
    {
        Boot = 0,
        GetStatus,
        StatusReply,
        Config3,
        Config4,
        Config5,
        Config6,
        OpenDoor,
        DoorOpen,
        CloseDoor,
        DoorClosed,
        Dispense,
        DispenseComplete,
        Config13,
        BlinkOrBeep,
        Sleep,
        Unknown16,
        Unknown17,
        Reply18,
        Config19,
        Reply20,
        DispenseData
    };
    struct Frame
    {
        uint32_t timestamp;
        uint8_t length;
        uint8_t type;
        uint8_t sequence;
        //uint8_t payload[257];
        uint16_t crc;
        uint16_t calcCrc;
        uint8_t raw[32];
        bool isValid;
    };
    bool serialLoop(Frame*);
    void printFrame(Frame *frame);
    int sendCommand(CommandType, uint8_t *, size_t);
    bool parseFrame(Frame *frame, uint32_t timeout);

private:
    HardwareSerial* ser;
    
    
    uint16_t crc_tab16[256];
    uint8_t sequence;
    uint16_t crc_16(const unsigned char *input_str, size_t num_bytes, uint16_t start_value);
    void init_crc16_tab(void);
};

#endif
#include "petkitbus.h"
#define CRC_POLY_CCITT 0x1021
#define CRC_START_CCITT_FFFF 0xFFFF


PetkitBus::PetkitBus(HardwareSerial *serial)
{
    ser = serial;
    init_crc16_tab();
    sequence = 0;
}

void PetkitBus::init()
{
    uint8_t cmd19[] = {0x05, 0x7e};
    sendCommand(CommandType::Config19, cmd19, sizeof(cmd19));
    delay(10);

    uint8_t cmd3[] = {0x00, 0x05, 0x00, 0x05};
    sendCommand(CommandType::Config3, cmd3, sizeof(cmd3));
    delay(10);

    uint8_t cmd5[] = {0x00, 0x05};
    sendCommand(CommandType::Config5, cmd5, sizeof(cmd5));
    delay(10);

    uint8_t cmd4[] = {0x00, 0xff, 0x00, 0xff};
    sendCommand(CommandType::Config4, cmd4, sizeof(cmd4));
    delay(10);

    uint8_t cmd6[] = {0xff, 0xff};
    sendCommand(CommandType::Config6, cmd6, sizeof(cmd6));
    delay(10);

    // 00 3C 01 90 F0 12 22 20 1F 4F 01
    uint8_t cmd13[] = {0x00, 0x3c, 0x01, 0x90, 0xf0, 0x12, 0x22, 0x20, 0x1f, 0x4f, 0x01};
    sendCommand(CommandType::Config13, cmd13, sizeof(cmd13));
    delay(10);
}

int PetkitBus::sendCommand(CommandType t, uint8_t *buf, size_t size)
{
    uint8_t command[size + 7] = {0};
    command[0] = 0xaa;
    command[1] = 0xaa;
    command[2] = size + 7;
    command[3] = (int)t;
    command[4] = sequence++;
    for (uint8_t i = 0; i < size; i++)
    {
        command[5 + i] = buf[i];
    }
    uint16_t crc = crc_16(command, size + 5, CRC_START_CCITT_FFFF);
    command[size + 5] = crc >> 8;
    command[size + 6] = crc & 0xff;

    return ser->write(command, size + 7);
}

bool PetkitBus::parseFrame(Frame *frame, uint32_t timeout)
{
    // uint32_t startTime = millis();
    //  uint8_t buf[128];

    ser->setTimeout(timeout);
    if (ser->available())
    {
        if (ser->peek() == 0xaa)
        {
            Frame f = {0};
            ser->read();
            if (ser->peek() == 0xaa)
            {
                f.timestamp = millis();
                f.raw[0] = (uint8_t)0xaa;
                f.raw[1] = (uint8_t)ser->read();
                f.raw[2] = (uint8_t)ser->read();
                f.length = f.raw[2];
                if (f.length > 32)
                    return false;
                ser->readBytes(&f.raw[3], f.length - 3);
                f.type = f.raw[3];
                f.sequence = f.raw[4];
                f.crc = (((uint16_t)f.raw[f.length - 2] << 8) | ((uint16_t)f.raw[f.length - 1]));
                f.raw[f.length] = '\0';
                f.calcCrc = crc_16(f.raw, f.length, CRC_START_CCITT_FFFF);
                if (f.calcCrc == 0)
                {
                    f.isValid = true;
                    memcpy(frame, &f, sizeof(Frame));
                    return true;
                }
                else
                    f.isValid = false;
            }
        }
        else
            ser->read();
    }
    return false;
}

void PetkitBus::beep(uint16_t ontime, uint16_t offtime, uint16_t quantity)
{
    uint8_t command[7];
    command[0] = (uint8_t)3;
    command[1] = (uint8_t)(0xff & (ontime >> 8));
    command[2] = (uint8_t)(ontime & 0xff);
    command[3] = (uint8_t)(0xff & (offtime >> 8));
    command[4] = (uint8_t)(offtime & 0xff);
    command[5] = (uint8_t)(0xff & (quantity >> 8));
    command[6] = (uint8_t)(quantity & 0xff);
    sendCommand(CommandType::BlinkOrBeep, command, sizeof(command));
}

void PetkitBus::blink(uint8_t led, uint16_t ontime, uint16_t offtime, uint16_t quantity)
{
    uint8_t command[7];
    if (led >= 2)
        command[0] = (uint8_t)2;
    else
        command[0] = (uint8_t)1;
    command[1] = (uint8_t)(0xff & (ontime >> 8));
    command[2] = (uint8_t)(ontime & 0xff);
    command[3] = (uint8_t)(0xff & (offtime >> 8));
    command[4] = (uint8_t)(offtime & 0xff);
    command[5] = (uint8_t)(0xff & (quantity >> 8));
    command[6] = (uint8_t)(quantity & 0xff);
    sendCommand(CommandType::BlinkOrBeep, command, sizeof(command));
}

void PetkitBus::open()
{
    uint8_t command = 0x1e;
    sendCommand(CommandType::OpenDoor, &command, sizeof(command));
}

void PetkitBus::close()
{
    uint8_t command = 0x1e;
    sendCommand(CommandType::CloseDoor, &command, sizeof(command));
}

void PetkitBus::dispense(uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4)
{
    uint8_t command[4];
    command[0] = param1;
    command[1] = param2;
    command[2] = param3;
    command[3] = param4;
    sendCommand(CommandType::Dispense, command, sizeof(command));
}

void PetkitBus::init_crc16_tab(void)
{

    uint16_t i;
    uint16_t j;
    uint16_t crc;
    uint16_t c;

    for (i = 0; i < 256; i++)
    {

        crc = 0;
        c = i << 8;

        for (j = 0; j < 8; j++)
        {

            if ((crc ^ c) & 0x8000)
                crc = (crc << 1) ^ CRC_POLY_CCITT;
            else
                crc = crc << 1;

            c = c << 1;
        }

        crc_tab16[i] = crc;
    }

    // crc_tab16_init = true;

} /* init_crcccitt_tab */

uint16_t PetkitBus::crc_16(const unsigned char *input_str, size_t num_bytes, uint16_t start_value)
{

    uint16_t crc;
    const unsigned char *ptr;
    size_t a;

    // if (!crc_tab16_init)
    //   init_crc16_tab();

    crc = start_value;
    ptr = input_str;

    if (ptr != NULL)
        for (a = 0; a < num_bytes; a++)
        {

            crc = (crc << 8) ^ crc_tab16[((crc >> 8) ^ (uint16_t)*ptr++) & 0x00FF];
        }

    return crc;

} /* crc_ccitt_generic */

void PetkitBus::printFrame(Frame *frame)
{
    ser->printf("\ntime:%u, packet type:%hhu, length:%hhu, seq:%hhu, crc:%04X, valid:%hhu,", frame->timestamp, frame->type, frame->length, frame->sequence, frame->crc, frame->isValid);

    if (frame->length >= 8)
    {
        ser->print(" Data:0x");
        for (int i = 5; i < frame->length - 2; i++)
        {
            if (frame->raw[i] < 0xF)
                ser->print("0");
            ser->print(frame->raw[i], HEX);
        }
        ser->print(",");
    }

    else
        ser->print(",");

    switch (frame->type)
    {
    case 0:
    {
        ser->println("boot");
    }
    break;
    case 1:
    {
        ser->println("ack get status");
    }
    break;
    case 2:
    {
        ser->println("status reply");
    }
    break;
    case 3:
    {
        ser->println("ack set config");
    }
    break;
    case 4:
    {
        ser->println("ack set config");
    }
    break;
    case 5:
    {
        ser->println("ack set config");
    }
    break;
    case 6:
    {
        ser->println("ack set config");
    }
    break;
    case 7:
    {
        ser->println("ack open door");
    }
    break;
    case 8:
    {
        ser->println("door open now");
    }
    break;
    case 9:
    {
        ser->println("ack close door");
    }
    break;
    case 10:
    {
        ser->println("door closed now");
    }
    break;
    case 11:
    {
        ser->println("ack dispense");
    }
    break;
    case 12:
    {
        ser->println("dispense info");
    }
    break;
    case 13:
    {
        ser->println("ack set config");
    }
    break;
    case 14:
    {
        ser->println("ack blink/beep");
    }
    break;
    case 15:
    {
        ser->println("ack Sleep");
    }
    break;
    case 16:
    {
        ser->println("Unknown");
    }
    break;
    case 17:
    {
        ser->println("Unknown");
    }
    break;
    case 18:
    {
        ser->println("unknown");
    }
    break;
    case 19:
    {
        ser->println("config");
    }
    break;
    case 20:
    {
        ser->println("config reply");
    }
    break;
    case 21:
    {
        ser->println("dispense complete");
    }
    break;
    }
    // ser->println();
}
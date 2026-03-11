#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10); // CS на пине 10

struct Can_Frames {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
};

unsigned long interval = 10000;
unsigned long last_time_send = 0;

Can_Frames frames[3] = {
    {0x280, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2D, 0x2C}, 8},
    {0x448, {0x83, 0x00, 0x88, 0x1A, 0x70}, 5},
    {0x4A0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8}
};

void setup() {
    Serial.begin(115200);
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    delay(200);
    Serial.println("Starting");     
}

void loop()
{
    unsigned long current_time = micros();

    if (current_time - last_time_send >= interval)
    {
        Frame_Sending();
        last_time_send = current_time;
    }
}

void Frame_Sending()
{
    for (int i = 0; i < 3; i++)
    {
        can_frame frame;

        frame.can_id = frames[i].id;  
        frame.can_dlc = frames[i].len; 
        memcpy(frame.data, frames[i].data, frame.can_dlc);
        mcp2515.sendMessage(&frame);
    }
}
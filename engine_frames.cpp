#include <SPI.h>
#include <mcp2515.h>

#define TOUCH_PIN_E 7

MCP2515 mcp2515(10); // CS на пине 10

struct Can_Frames {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
};

unsigned long interval = 10000;
unsigned long last_time_send = 0;
unsigned long last_press_time = 0;
bool engine_flag = false;

Can_Frames engine_frame_off = {0x280, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x00}, 8};
/*742 rpm*/ //Can_Frames engine_frame_on = {0x280, {0x01, 0x0F, 0x9A, 0x0B, 0x0B, 0x00, 0x11, 0x0B}, 8};

void setup() {
    Serial.begin(115200);

    pinMode(TOUCH_PIN_E, INPUT);

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    delay(200);
    Serial.println("Starting");     
}

void loop()
{
    unsigned long current_time = micros();

    if (digitalRead(TOUCH_PIN_E) == HIGH && (current_time - last_press_time) > 1000000)
    {
        last_press_time = current_time;
        engine_flag = !engine_flag;
    }

    if (current_time - last_time_send >= interval)
    {
        can_frame frame;

        if (engine_flag == false)
        {
            frame.can_id = engine_frame_off.id;  
            frame.can_dlc = engine_frame_off.len; 
            memcpy(frame.data, engine_frame_off.data, frame.can_dlc);
            mcp2515.sendMessage(&frame);
        }
        
        if (engine_flag == true)
        {
            frame.can_id = engine_frame_on.id;  
            frame.can_dlc = engine_frame_on.len; 
            memcpy(frame.data, engine_frame_on.data, frame.can_dlc);
            mcp2515.sendMessage(&frame);
        }
    }
}

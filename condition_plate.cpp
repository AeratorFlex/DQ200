#include <SPI.h>
#include <mcp2515.h>

#define TOUCH_PIN_P 4   //PARKING // выставить пины кнопок 
#define TOUCH_PIN_R 5   //RARE
#define TOUCH_PIN_N 7   //NEITRAL
#define TOUCH_PIN_D 6   //DRIVE
#define TOUCH_PIN_S 7   //SPORT
#define TOUCH_PIN_M 7   //MANUAL
#define TOUCH_PIN_MP 7  //MANUAL UP
#define TOUCH_PIN_MM 7  //MANUAL DOWN
#define TOUCH_PIN_ENGINE 3 //ENGINE START/STOP

/*
D2...MCP2515
D3...ENGINE START/STOP
D4...PARKING
D5...RARE
D6...DRIVE
D7...
D8...
D9...
D10..MCP2515
D11..MCP2515
D12..MCP2515
D13..MCP2515
*/
MCP2515 mcp2515(10);

struct Can_Frames
{
    uint32_t ID;
    uint8_t DATA[8];
    uint8_t len;
};

void setup() {
    Serial.begin(115200);
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    
    pinMode(TOUCH_PIN_P, INPUT);
    pinMode(TOUCH_PIN_R, INPUT);
    pinMode(TOUCH_PIN_N, INPUT);
    pinMode(TOUCH_PIN_D, INPUT);
    pinMode(TOUCH_PIN_S, INPUT);
    pinMode(TOUCH_PIN_M, INPUT);
    pinMode(TOUCH_PIN_MM, INPUT);
    pinMode(TOUCH_PIN_MP, INPUT);
    pinMode(TOUCH_PIN_ENGINE, INPUT);


    delay(200);
    Serial.println("Starting");
}

int flag_selector_state = 0; //по умолчанию 0/паркинг
bool flag_engine = false;

unsigned long selector_frame_interval = 10000;
unsigned long selector_frame_last_send = 0;

unsigned long engine_frame_interval = 10000;
unsigned long engine_frame_last_send = 0;

Can_Frames selector_frame_p = {0x448, {0x82, 0x00, 0x88, 0x1A, 0x70}, 5};
Can_Frames selector_frame_r = {0x448, {0x72, 0x00, 0x88, 0x1A, 0x70}, 5};
Can_Frames selector_frame_n = {0x448, {0x62, 0x00, 0x88, 0x1A, 0x70}, 5};
Can_Frames selector_frame_d = {0x448, {0x52, 0x00, 0x88, 0x1A, 0x70}, 5};
Can_Frames selector_frame_s = {0x448, {0x42, 0x00, 0x88, 0x1A, 0x70}, 5};
Can_Frames selector_frame_m = {0x448, {0x42, 0x00, 0x88, 0x1A, 0x70}, 5};
Can_Frames selector_frame_mp = {0x448, {0x42, 0x00, 0x88, 0x1A, 0x70}, 5};
Can_Frames selector_frame_mm = {0x448, {0x42, 0x00, 0x88, 0x1A, 0x70}, 5};
Can_Frames engine_frame_off = {0x280, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x00}, 8};
Can_Frames engine_frame_on = {0x280, {0x01, 0x0F, 0x9A, 0x0B, 0x0B, 0x00, 0x11, 0x0B}, 8};

void loop()
{
    unsigned long current_time = micros();
    
    static unsigned long last_press_0 = 0;
    static unsigned long last_press_1 = 0;
    static unsigned long last_press_2 = 0;
    static unsigned long last_press_3 = 0;
    static unsigned long last_press_4 = 0;
    static unsigned long last_press_5 = 0;
    static unsigned long last_press_6 = 0;
    static unsigned long last_press_7 = 0;
    static unsigned long last_press_8 = 0;

    if (digitalRead(TOUCH_PIN_P) == HIGH && (current_time - last_press_0) > 1000000)
    {
        last_press_0 = current_time;
        flag_selector_state = 0; 
    }

    if (digitalRead(TOUCH_PIN_R) == HIGH && (current_time - last_press_1) > 1000000)
    {
        last_press_1 = current_time;
        flag_selector_state = 1; 
    }

    if (digitalRead(TOUCH_PIN_N) == HIGH && (current_time - last_press_2) > 1000000)
    {
        last_press_2 = current_time;
        flag_selector_state = 2; 
    }

    if (digitalRead(TOUCH_PIN_D) == HIGH && (current_time - last_press_3) > 1000000)
    {
        last_press_3 = current_time;
        flag_selector_state = 3; 
    }

    if (digitalRead(TOUCH_PIN_S) == HIGH && (current_time - last_press_4) > 1000000)
    {
        last_press_4 = current_time;
        flag_selector_state = 4; 
    }

    if (digitalRead(TOUCH_PIN_M) == HIGH && (current_time - last_press_5) > 1000000)
    {
        last_press_5 = current_time;
        flag_selector_state = 5; 
    }

    if (digitalRead(TOUCH_PIN_MP) == HIGH && (current_time - last_press_6) > 1000000)
    {
        last_press_6 = current_time;
        flag_selector_state = 6; 
    }

    if (digitalRead(TOUCH_PIN_MM) == HIGH && (current_time - last_press_7) > 1000000)
    {
        last_press_7 = current_time;
        flag_selector_state = 7; 
    }

    static unsigned long last_press_8 = 0;

    if (digitalRead(TOUCH_PIN_ENGINE) == HIGH && (current_time - last_press_8) > 1000000)
    {
        last_press_8 = current_time;
        if (flag_engine == false)
        {
            flag_engine = true;
        }
        else
        {
            flag_engine = false;
        }
    }

    if (current_time - selector_frame_last_send >= selector_frame_interval)
    {
        Selector_Frame(current_time);
    }
   
    if (current_time - engine_frame_last_send >= engine_frame_interval)
    {
        Engine_Frame();
        engine_frame_last_send = current_time;
    }
    
}

void Engine_Frame()
{
    can_frame frame;

    switch (flag_engine)
    {
    case true:
        frame.can_id = engine_frame_on.ID;
        frame.can_dlc = engine_frame_on.len;
        memcpy(frame.data, engine_frame_on.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
        break;
    
    case false:
        frame.can_id = engine_frame_off.ID;
        frame.can_dlc = engine_frame_off.len;
        memcpy(frame.data, engine_frame_off.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
        break;
    }
}

void Selector_Frame(unsigned long current_time)
{
    can_frame frame;  

    switch (flag_selector_state)
    {
    case 0:
        frame.can_id = selector_frame_p.ID;
        frame.can_dlc = selector_frame_p.len;
        memcpy(frame.data, selector_frame_p.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
        selector_frame_last_send = current_time;
        break;

    case 1:
        frame.can_id = selector_frame_r.ID;
        frame.can_dlc = selector_frame_r.len;
        memcpy(frame.data, selector_frame_r.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
        selector_frame_last_send = current_time;
        break;

    case 2:
        frame.can_id = selector_frame_n.ID;
        frame.can_dlc = selector_frame_n.len;
        memcpy(frame.data, selector_frame_n.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
        selector_frame_last_send = current_time;
        break;

    case 3:
        frame.can_id = selector_frame_d.ID;
        frame.can_dlc = selector_frame_d.len;
        memcpy(frame.data, selector_frame_d.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
        selector_frame_last_send = current_time;
        break;

    case 4:
        frame.can_id = selector_frame_s.ID;
        frame.can_dlc = selector_frame_s.len;
        memcpy(frame.data, selector_frame_s.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
        selector_frame_last_send = current_time;
        break;

    case 5:
        frame.can_id = selector_frame_m.ID;
        frame.can_dlc = selector_frame_m.len;
        memcpy(frame.data, selector_frame_m.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
        selector_frame_last_send = current_time;
        break;

    case 6:
        frame.can_id = selector_frame_mp.ID;
        frame.can_dlc = selector_frame_mp.len;
        memcpy(frame.data, selector_frame_mp.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
        selector_frame_last_send = current_time;
        break;

    case 7:
        frame.can_id = selector_frame_mm.ID;
        frame.can_dlc = selector_frame_mm.len;
        memcpy(frame.data, selector_frame_mm.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
        selector_frame_last_send = current_time;
        break;
    }
}
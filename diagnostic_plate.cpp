#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>

#define TOUCH_PIN_1 3 // diagnostic connection/disconnection
#define TOUCH_PIN_2 4 // start_adaptation
#define TOUCH_PIN_3 5 // fault_reading
#define TOUCH_PIN_4 6 // fault_delete
#define TOUCH_PIN_5 7 // engine start/stop

MCP2515 mcp2515(10);
LiquidCrystal_I2C lcd(0x27, 16, 2);

struct Can_Frames
{
    uint32_t ID;
    uint8_t DATA[8];
    uint8_t len;
};

void setup()
{
    Timer1.initialize(10000);  // 10 000 микросекунд = 10 мс
    Timer1.attachInterrupt(Send_Three_Frames);

    lcd.init();
    lcd.backlight();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CAN Data:");

    Serial.begin(115200);

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    pinMode(TOUCH_PIN_1, INPUT_PULLUP);
    pinMode(TOUCH_PIN_2, INPUT_PULLUP);

    delay(200);
    Serial.println("Starting");
}

bool flag_diagnostic_connection = false;
bool flag_adaptation = false;
int flag_connection_step = 0; // какой этап начала диагностического сеанса начат
int flag_adaptation_step = 0;
int model_number[10] = {};
int software_version[4] = {};
int part_number[9] = {};

unsigned long function_timeout = 5000000;

unsigned long wakeup_CAN_frame_interval = 75000;
int Wakeup_CAN_Frame_counter = 0;
unsigned long wakeup_last_send = 0;

unsigned long try_1_interval = 75000;
unsigned long try_1_last_send = 0;

unsigned long try_2_interval = 250000;
unsigned long try_2_last_send = 0;

unsigned long try_3_interval = 10000;
unsigned long try_3_last_send = 0;

unsigned long start_session_frame_interval = 15000; // может быть 10000 нужно будет попробовать
unsigned long start_session_frame_last_send = 0;

unsigned long installation_session_frame_interval = 150000;
unsigned long installation_session_frame_last_send = 0;

unsigned long identification_request_frame_interval = 100000;
unsigned long identification_request_frame_last_send = 0;

unsigned long some_request_frame_interval = 200000; // was 100000
unsigned long some_request_frame_last_send = 0;

unsigned long A3_frame_interval = 500000;
unsigned long A3_frame_last_send = 0;

unsigned long A3_frame_2_interval = 140000;
unsigned long A3_frame_2_last_send = 0;

unsigned long part_number_request_frame_interval = 10000;
unsigned long part_number_request_frame_last_send = 0;

unsigned long adaptation_frame_interval = 150000;
unsigned long adaptation_frame_last_send = 0;

volatile bool engine_flag = false;

Can_Frames wakeup_CAN_frame = {0x200, {0x1F, 0xC0, 0x00, 0x10, 0x00, 0x03, 0x01}, 7}; // 10 раз с задержкой 75 мс
Can_Frames try_1 = {0x710, {0x02, 0x10, 0x03, 0x55, 0x55, 0x55, 0x55, 0x55}, 8};
Can_Frames try_2 = {0x200, {0x02, 0xC0, 0x00, 0x10, 0x00, 0x03, 0x01}, 7};
Can_Frames try_3 = {0x202, {0x00, 0xD0, 0x00, 0x03, 0x60, 0x07, 0x01}, 7};
Can_Frames start_session_frame = {0x760, {0xA0, 0x0F, 0x8A, 0xFF, 0x32, 0xFF}, 6};
Can_Frames installation_session_frame = {0x760, {0x10, 0x00, 0x02, 0x10, 0x89}, 5};
Can_Frames B_frame = {0x760, {0xB0}, 1};
Can_Frames identification_request_frame = {0x760, {0x11, 0x00, 0x02, 0x1A, 0x9B}, 5};
Can_Frames some_request_frame = {0x760, {0x12, 0x00, 0x04, 0x31, 0xB8, 0x00, 0x00}, 7};
Can_Frames A3_frame = {0x760, {0xA3}, 1};
Can_Frames A8_frame = {0x760, {0xA8}, 1};
Can_Frames part_number_request_frame = {0x760, {0x13, 0x00, 0x02, 0x1A, 0x91}, 5};
Can_Frames system_preparation_frame = {0x760, {0x14, 0x00, 0x04, 0x31, 0xB8, 0x00, 0x3C}, 7};
Can_Frames adaptation_frame = {0x760, {0x15, 0x00, 0x04, 0x31, 0xBA, 0x00, 0x3C}, 7};
Can_Frames fault_codes_request = {0x760, {0x18, 0x00, 0x04, 0x18, 0x00, 0xFF, 0x00}, 7};
Can_Frames fault_codes_delete= {0x760, {0x1A, 0x00, 0x04, 0x18, 0x00, 0xFF, 0x00}, 7};

Can_Frames engine_frame_off = {0x280, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x00}, 8};
Can_Frames engine_frame_on = {0x280, {0x01, 0x0F, 0x9A, 0x0B, 0x0B, 0x00, 0x11, 0x0B}, 8};  //742
Can_Frames frames[2] = {
    {0x448, {0x83, 0x00, 0x88, 0x1A, 0x70}, 5},
    {0x4A0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8}
};

void loop()
{
    unsigned long current_time = micros();

    //подключение диагностики
    static unsigned long last_press_1 = 0;
    if (digitalRead(TOUCH_PIN_1) == LOW && (current_time - last_press_1) > 1000000)
    {
        last_press_1 = current_time;

        if (flag_diagnostic_connection == false)
        {
            flag_diagnostic_connection = true;
            B_frame.DATA[0] = 0xB0;          // B-ACK сбрасывается в ноль при повторном подключении
            Wakeup_CAN_Frame_counter = 0;    // СБРОСИТЬ счётчик!
            wakeup_last_send = current_time; // Начать отсчёт!
        }
        else
        {
            flag_diagnostic_connection = false;
            flag_connection_step = 0;
            Wakeup_CAN_Frame_counter = 0;
            lcd.clear();
            A8_Frame();
        }
    }

    //получение кодов неисправностей
    static unsigned long last_press_3 = 0;
    if (digitalRead(TOUCH_PIN_3) == LOW && (current_time - last_press_3) > 1000000)
    {
        last_press_3 = current_time;
        Fault_Codes_Request();
    }


    //удаление кодов неисправностей
    static unsigned long last_press_4 = 0;
    if (digitalRead(TOUCH_PIN_4) == LOW && (current_time - last_press_4) > 1000000)
    {
        last_press_4 = current_time;
        Fault_Codes_Delete();
    }
    
    //запуск адаптации
    static unsigned long last_press_2 = 0;
    if (digitalRead(TOUCH_PIN_2) == LOW && (current_time - last_press_2) > 1000000)
    {
        last_press_2 = current_time;
        flag_adaptation = true;
    }

    //запуск или остановка двигателя
    static unsigned long last_press_5 = 0;
    if(digitalRead(TOUCH_PIN_5) == LOW && (current_time - last_press_5) > 1000000)
    {
        last_press_5 = current_time;
        engine_flag = !engine_flag;
    }

    if (flag_diagnostic_connection == true && flag_connection_step != 10)
    {
        switch (flag_connection_step)
        {
        case 0:
            if (current_time - wakeup_last_send >= wakeup_CAN_frame_interval)
            {
                Wakeup_CAN_Frame();
                wakeup_last_send = current_time;
                if (++Wakeup_CAN_Frame_counter >= 10)
                {
                    flag_connection_step = 1; // Завершили этап 0
                    try_1_last_send = current_time;
                }
            }
            break;

        case 1:
            if (current_time - try_1_last_send >= try_1_interval)
            {
                Try_1();
                try_1_last_send = current_time;
                flag_connection_step = 2;
                try_2_last_send = current_time;
            }
            break;

        case 2:
            if (current_time - try_2_last_send >= try_2_interval)
            {
                Try_2();
                try_2_last_send = current_time;
                flag_connection_step = 3;
                try_3_last_send = current_time;
            }
            break;

        case 3:
            if (current_time - try_3_last_send >= try_3_interval)
            {
                Try_3();
                try_3_last_send = current_time;
                flag_connection_step = 4;
                start_session_frame_last_send = current_time;
            }
            break;

        case 4:
            if (current_time - start_session_frame_last_send >= start_session_frame_interval)
            {
                Start_Session_Frame();
                start_session_frame_last_send = current_time;
                flag_connection_step = 5;
                installation_session_frame_last_send = current_time;
            }
            break;

        case 5:
            if (current_time - installation_session_frame_last_send >= installation_session_frame_interval)
            {
                Installation_Session_Frame();
                installation_session_frame_last_send = current_time;
                flag_connection_step = 6;
                identification_request_frame_last_send = current_time;
            }
            break;

        case 6:
            if (current_time - identification_request_frame_last_send >= identification_request_frame_interval)
            {
                Identification_Request_Frame();
                installation_session_frame_last_send = current_time;
                flag_connection_step = 7;
                some_request_frame_last_send = current_time;
            }
            break;

        case 7:
            if (current_time - some_request_frame_last_send >= some_request_frame_interval)
            {
                Some_Request_Frame();
                some_request_frame_last_send = current_time;
                flag_connection_step = 8;
                A3_frame_2_last_send = current_time;
            }
            break;

        case 8:
            if (current_time - A3_frame_2_last_send >= A3_frame_2_interval)
            {
                A3_Frame();
                A3_frame_2_last_send = current_time;
                flag_connection_step = 9;
                part_number_request_frame_last_send = current_time;
            }
            break;

        case 9:
            if (current_time - part_number_request_frame_last_send >= part_number_request_frame_interval)
            {
                Part_Number_Request_Frame();
                part_number_request_frame_last_send = current_time;
                flag_connection_step = 10;
                A3_frame_last_send = current_time;
            }
            break;

        default:
            break;
        }
    }

    if (flag_adaptation == true && flag_diagnostic_connection == true)
    {
        switch (flag_adaptation_step)
        {
        case 0:
        {
            System_Preparation_Frame();
            flag_adaptation_step = 1;
            adaptation_frame_last_send = current_time;
        }
        break;

        case 1:
        {
            if (current_time - adaptation_frame_last_send >= adaptation_frame_interval)
            {
                Adaptation_Frame();
                adaptation_frame_last_send = current_time;
            }
        }

        default:
            break;
        }
    }
    else
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("No connection");
        lcd.setCursor(1, 0);
        lcd.print("Wait for 5 sec");
    }

    if (flag_diagnostic_connection == true && flag_connection_step == 10)
    {
        if (current_time - A3_frame_last_send >= A3_frame_interval)
        {
            A3_Frame();
            A3_frame_last_send = current_time;
        }
    }
}

void Wakeup_CAN_Frame()
{
    can_frame frame;
    frame.can_id = wakeup_CAN_frame.ID;
    frame.can_dlc = wakeup_CAN_frame.len;
    memcpy(frame.data, wakeup_CAN_frame.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);
}

void Try_1()
{
    can_frame frame;
    frame.can_id = try_1.ID;
    frame.can_dlc = try_1.len;
    memcpy(frame.data, try_1.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);
}

void Try_2()
{
    can_frame frame;
    frame.can_id = try_2.ID;
    frame.can_dlc = try_2.len;
    memcpy(frame.data, try_2.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);
}

void Try_3()
{
    can_frame frame;
    frame.can_id = try_3.ID;
    frame.can_dlc = try_3.len;
    memcpy(frame.data, try_3.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);
}

void Start_Session_Frame()
{
    can_frame frame;
    frame.can_id = start_session_frame.ID;
    frame.can_dlc = start_session_frame.len;
    memcpy(frame.data, start_session_frame.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);

    unsigned long timeout_start = micros();
    bool timeout_occurred = false;

    while (!timeout_occurred)
    {
        if (micros() - timeout_start >= function_timeout)
        {
            timeout_occurred = true;
            flag_diagnostic_connection = false;
            break;
        }

        can_frame frame;

        if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK)
        {
            if (frame.can_id == 0x300 && frame.can_dlc == 6)
            {
                if (frame.data[3] == 0xFF && frame.data[4] == 0x54 && frame.data[5] == 0xFF)
                {
                    break;
                }
            }
        }
    }
}

void Installation_Session_Frame()
{
    can_frame frame;
    frame.can_id = installation_session_frame.ID;
    frame.can_dlc = installation_session_frame.len;
    memcpy(frame.data, installation_session_frame.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);

    int recieved_frames_counter = 0;
    unsigned long timeout_start = micros();
    bool timeout_occurred = false;

    while (!timeout_occurred)
    {
        if (micros() - timeout_start >= function_timeout)
        {
            timeout_occurred = true;
            flag_diagnostic_connection = false;
            return;
        }

        can_frame frame;

        mcp2515.readMessage(&frame);
        if (frame.can_id == 0x300 && frame.can_dlc != 1)
        {
            recieved_frames_counter++;
            if (frame.data[3] == 0x50 && frame.data[4] == 0x89)
            {
                break;
            }
        }
    }

    uint8_t lower_nibble = B_frame.DATA[0] & 0x0F;
    lower_nibble = (lower_nibble + recieved_frames_counter) % 16;
    B_frame.DATA[0] = 0xB0 | lower_nibble;

    can_frame frame_B;
    frame_B.can_id = B_frame.ID;
    frame_B.can_dlc = B_frame.len;
    memcpy(frame_B.data, B_frame.DATA, frame_B.can_dlc);
    mcp2515.sendMessage(&frame_B);
}

void Identification_Request_Frame()
{
    can_frame frame;
    frame.can_id = identification_request_frame.ID;
    frame.can_dlc = identification_request_frame.len;
    memcpy(frame.data, identification_request_frame.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);

    int recieved_frames_counter = 0;
    can_frame f_part_frame;
    can_frame s_part_frame;
    can_frame t_part_frame;

    unsigned long timeout_start = micros();
    bool timeout_occurred = false;

    while (!timeout_occurred)
    {
        if (micros() - timeout_start >= function_timeout)
        {
            timeout_occurred = true;
            flag_diagnostic_connection = false;
            return;
        }

        can_frame frame;

        mcp2515.readMessage(&frame);
        if (frame.can_id == 0x300 && frame.can_dlc != 1)
        {
            recieved_frames_counter++;
            if (frame.data[0] == 0x21)
            {
                f_part_frame = frame;
            }
            if (frame.data[0] == 0x22)
            {
                s_part_frame = frame;
            }
            if (frame.data[0] == 0x23)
            {
                t_part_frame = frame;
            }
            delay(5);

            if (frame.can_dlc == 2)
            {
                uint8_t lower_nibble = B_frame.DATA[0] & 0x0F;
                lower_nibble = (lower_nibble + recieved_frames_counter) % 16;
                B_frame.DATA[0] = 0xB0 | lower_nibble;

                can_frame frame_B;
                frame_B.can_id = B_frame.ID;
                frame_B.can_dlc = B_frame.len;
                memcpy(frame_B.data, B_frame.DATA, frame_B.can_dlc);
                mcp2515.sendMessage(&frame_B);
                break;
            }
        }
    }
    model_number[0] = f_part_frame.data[5];
    model_number[1] = f_part_frame.data[6];
    model_number[2] = f_part_frame.data[7];
    model_number[3] = s_part_frame.data[1];
    model_number[4] = s_part_frame.data[2];
    model_number[5] = s_part_frame.data[3];
    model_number[6] = s_part_frame.data[4];
    model_number[7] = s_part_frame.data[5];
    model_number[8] = s_part_frame.data[6];
    model_number[9] = s_part_frame.data[7];

    software_version[0] = t_part_frame.data[3];
    software_version[1] = t_part_frame.data[4];
    software_version[2] = t_part_frame.data[5];
    software_version[3] = t_part_frame.data[6];

    lcd.clear();
    for (int i = 0; i < 10; i++)
    {
        lcd.setCursor(i, 0);
        lcd.print((char)model_number[i]);
    }

    lcd.setCursor(11, 0);

    for (int i = 0; i < 4; i++)
    {
        lcd.setCursor((i + 11), 0);
        lcd.print((char)software_version[i]);
    }
}

void Some_Request_Frame()
{
    can_frame frame;
    frame.can_id = some_request_frame.ID;
    frame.can_dlc = some_request_frame.len;
    memcpy(frame.data, some_request_frame.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);

    int recieved_frames_counter = 0;

    unsigned long timeout_start = micros();
    bool timeout_occurred = false;

    while (!timeout_occurred)
    {
        if (micros() - timeout_start >= function_timeout)
        {
            timeout_occurred = true;
            flag_diagnostic_connection = false;
            break;
        }

        can_frame frame;

        mcp2515.readMessage(&frame);
        if (frame.can_id == 0x300 && frame.can_dlc != 1)
        {
            recieved_frames_counter++;
            delay(5);

            if (frame.data[6] == 0x01 && frame.data[7] == 0x18)
            {
                uint8_t lower_nibble = B_frame.DATA[0] & 0x0F;
                lower_nibble = (lower_nibble + recieved_frames_counter) % 16;
                B_frame.DATA[0] = 0xB0 | lower_nibble;

                can_frame frame_B;
                frame_B.can_id = B_frame.ID;
                frame_B.can_dlc = B_frame.len;
                memcpy(frame_B.data, B_frame.DATA, frame_B.can_dlc);
                mcp2515.sendMessage(&frame_B);
                break;
            }
        }
    }
}

void A3_Frame()
{
    can_frame frame;
    frame.can_id = A3_frame.ID;
    frame.can_dlc = A3_frame.len;
    memcpy(frame.data, A3_frame.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);

    unsigned long timeout_start = micros();
    bool timeout_occurred = false;

    while (!timeout_occurred)
    {
        if (micros() - timeout_start >= function_timeout)
        {
            timeout_occurred = true;
            flag_diagnostic_connection = false;
            break;
        }

        can_frame frame;

        if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK)
        {
            if (frame.can_id == 0x300 && frame.can_dlc == 6)
            {
                if (frame.data[3] == 0xFF && frame.data[4] == 0x54 && frame.data[5] == 0xFF)
                {
                    break;
                }
            }
        }
    }
}

void Part_Number_Request_Frame()
{
    can_frame frame;
    frame.can_id = part_number_request_frame.ID;
    frame.can_dlc = part_number_request_frame.len;
    memcpy(frame.data, part_number_request_frame.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);

    int recieved_frames_counter = 0;
    can_frame f_part_frame;
    can_frame s_part_frame;

    unsigned long timeout_start = micros();
    bool timeout_occurred = false;

    while (!timeout_occurred)
    {
        if (micros() - timeout_start >= function_timeout)
        {
            timeout_occurred = true;
            flag_diagnostic_connection = false;
            return;
        }

        if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) // вот тут нужно будет пересмотреть механизм копирования, возможно,
        {                                                     // вместо копирования указателей можно будет сразу копировать в массив
            if (frame.can_id == 0x300 && frame.can_dlc != 1)  // и соответственно в предидущей функции так же сделать
            {
                recieved_frames_counter++;

                if (frame.data[0] == 0x2B)
                {
                    f_part_frame = frame;
                }
                if (frame.data[0] == 0x2C)
                {
                    s_part_frame = frame;
                }

                if (frame.data[0] == 0x1E)
                {
                    delay(5);
                    uint8_t lower_nibble = B_frame.DATA[0] & 0x0F;
                    lower_nibble = (lower_nibble + recieved_frames_counter) % 16;
                    B_frame.DATA[0] = 0xB0 | lower_nibble;

                    can_frame frame_B;
                    frame_B.can_id = B_frame.ID;
                    frame_B.can_dlc = B_frame.len;
                    memcpy(frame_B.data, B_frame.DATA, frame_B.can_dlc);
                    mcp2515.sendMessage(&frame_B);
                    break;
                }
            }
        }
    }

    part_number[0] = f_part_frame.data[6];
    part_number[1] = f_part_frame.data[7];
    part_number[2] = s_part_frame.data[1];
    part_number[3] = s_part_frame.data[2];
    part_number[4] = s_part_frame.data[3];
    part_number[5] = s_part_frame.data[4];
    part_number[6] = s_part_frame.data[5];
    part_number[7] = s_part_frame.data[6];
    part_number[8] = s_part_frame.data[7];

    for (int i = 0; i < 9; i++)
    {
        lcd.setCursor(i, 1);
        lcd.print((char)part_number[i]);
    }
}

void System_Preparation_Frame()
{
    can_frame frame;

    frame.can_id = system_preparation_frame.ID;
    frame.can_dlc = system_preparation_frame.len;
    memcpy(frame.data, system_preparation_frame.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);

    int recieved_frames_counter = 0;

    unsigned long timeout_start = micros();
    bool timeout_occurred = false;

    while (!timeout_occurred)
    {
        if (micros() - timeout_start >= function_timeout)
        {
            timeout_occurred = true;
            flag_diagnostic_connection = false;
            break;
        }

        can_frame frame;

        mcp2515.readMessage(&frame);
        if (frame.can_dlc == 7 && frame.data[3] == 0x71)
        {
            recieved_frames_counter++;

            uint8_t lower_nibble = B_frame.DATA[0] & 0x0F;
            lower_nibble = (lower_nibble + recieved_frames_counter) % 16;
            B_frame.DATA[0] = 0xB0 | lower_nibble;

            can_frame frame_B;
            frame_B.can_id = B_frame.ID;
            frame_B.can_dlc = B_frame.len;
            memcpy(frame_B.data, B_frame.DATA, frame_B.can_dlc);
            mcp2515.sendMessage(&frame_B);
            break;
        }
    }
}

void Adaptation_Frame()
{
    can_frame frame;
    uint8_t status_1 = 0x00;
    uint8_t status_2 = 0x00;
    int changes_counter = 0;

    frame.can_id = adaptation_frame.ID;
    frame.can_dlc = adaptation_frame.len;
    memcpy(frame.data, adaptation_frame.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);

    int recieved_frames = 0;

    unsigned long timeout_start = micros();
    bool timeout_occurred = false;

    while (!timeout_occurred)
    {
        if (micros() - timeout_start >= function_timeout)
        {
            timeout_occurred = true;
            flag_diagnostic_connection = false;
            return;
        }

        can_frame frame;

        mcp2515.readMessage(&frame);
        if (frame.can_dlc != 1)
        {
            status_1 = status_2;
            status_2 = frame.data[6];
            if (status_2 != status_1)
            {
                changes_counter++;
                if (changes_counter == 3)
                {
                    Serial.println("4 0 0");
                    Serial.println("Start engine");
                }
            }

            recieved_frames++;
            if (recieved_frames == 3)
            {
                uint8_t lower_nibble = B_frame.DATA[0] & 0x0F;
                lower_nibble = (lower_nibble + recieved_frames) % 16;
                B_frame.DATA[0] = 0xB0 | lower_nibble;

                can_frame frame_B;
                frame_B.can_id = B_frame.ID;
                frame_B.can_dlc = B_frame.len;
                memcpy(frame_B.data, B_frame.DATA, frame_B.can_dlc);
                mcp2515.sendMessage(&frame_B);
                break;
            }
        }
    }

    adaptation_frame.DATA[0] += 0x1;
    if (adaptation_frame.DATA[0] == 0x20)
    {
        adaptation_frame.DATA[0] = 0x10;
    }
    delay(5); // нужна чтобы буфер успевал чиститься и я не читал один и тот же фрейм из буфера дважды
}

void A8_Frame()
{
    can_frame frame;
    frame.can_id = A8_frame.ID;
    frame.can_dlc = A8_frame.len;
    memcpy(frame.data, A8_frame.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);
}

void Fault_Codes_Request()
{
    can_frame frame;
    frame.can_id = fault_codes_request.ID;
    frame.can_dlc = fault_codes_request.len;
    memcpy(frame.data, fault_codes_request.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);


}

void Fault_Codes_Delete()
{
    can_frame frame;
    frame.can_id = fault_codes_delete.ID;
    frame.can_dlc = fault_codes_delete.len;
    memcpy(frame.data, fault_codes_delete.DATA, frame.can_dlc);
    mcp2515.sendMessage(&frame);
}

void Send_Three_Frames()
{
    can_frame frame;
    
    if (engine_flag) {
        frame.can_id = engine_frame_on.ID;
        frame.can_dlc = engine_frame_on.len;
        memcpy(frame.data, engine_frame_on.DATA, frame.can_dlc);
    } else {
        frame.can_id = engine_frame_off.ID;
        frame.can_dlc = engine_frame_off.len;
        memcpy(frame.data, engine_frame_off.DATA, frame.can_dlc);
    }
    mcp2515.sendMessage(&frame);
    
    for (int i = 0; i < 2; i++) {
        frame.can_id = frames[i].ID;
        frame.can_dlc = frames[i].len;
        memcpy(frame.data, frames[i].DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
    }
}
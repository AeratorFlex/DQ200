    #include <SPI.h>
    #include <mcp2515.h>

    MCP2515 mcp2515(10);

    struct Can_Frames
    {
        uint32_t ID;
        uint8_t DATA[8];
        uint8_t len;
    };

    Can_Frames tester = {0x777, {0xFF, 0xFF, 0xFF}, 3};
    Can_Frames first_frame = {0x200, {0x1F, 0xC0, 0x00, 0x10, 0x00, 0x03, 0x01}, 7};
    Can_Frames first_diagnostic = {0x710, {0x02, 0x10, 0x03, 0x55, 0x55, 0x55, 0x55, 0x55}, 8};
    Can_Frames second_frame = {0x200, {0x02, 0xC0, 0x00, 0x10, 0x00, 0x03, 0x01}, 7};
    Can_Frames third_frame = {0x202, {0x00, 0xD0, 0x00, 0x03, 0x60, 0x07, 0x01}, 7};
    Can_Frames second_diagnostic= {0x760, {0xA0, 0x0F, 0x8A, 0xFF, 0x32, 0xFF}, 6};
    Can_Frames start_session = {0x760, {0x10, 0x00, 0x02, 0x10, 0x89}, 5};
    Can_Frames frame_A3 = {0x760, {0xA3}, 1};
    Can_Frames frame_B1 = {0x760, {0xB1}, 1};
    Can_Frames frame_B9 = {0x760, {0xB9}, 1};
    Can_Frames frame_BB = {0x760, {0xBB}, 1};
    Can_Frames frame_BF = {0x760, {0xBF}, 1};
    Can_Frames identification_frame = {0x760, {0x11, 0x00, 0x02, 0x1A, 0x9B}, 5};
    Can_Frames first_frame_adaptation = {0x760, {0x12, 0x00, 0x04, 0x31, 0xB8, 0x00, 0x00}, 7};
    Can_Frames second_frame_adaptation = {0x760, {0x13, 0x00, 0x02, 0x1A, 0x91}, 5};
    Can_Frames main_frame_adaptation = {0x760, {0x14, 0x00, 0x04, 0x31, 0xB8, 0x00, 0x3C}, 7};
    Can_Frames B_ACK_frame = {0x760, {0xB0}, 1};
    Can_Frames first_condition_frame = {0x280, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2D, 0x2C}, 8};
    Can_Frames second_condition_frame = {0x448, {0x83, 0x00, 0x88, 0x1A, 0x70}, 5};
    Can_Frames third_condition_frame = {0x4A0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8};

    uint8_t expected_response_starting_session[] = {0x10, 0x00, 0x02, 0x50, 0x89};
    uint8_t expected_ident_response[] = {0x18, 0x20};

    unsigned long last_send_time_A3 = 0;
    unsigned long last_send_time_adaptation = 0;
    const int send_interval_A3 = 500000;
    const int send_interval_adaptation = 150000;

    void setup() {
        Serial.begin(115200);
        
        mcp2515.reset();
        mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
        mcp2515.setNormalMode();
        
        delay(200);
        Serial.println("Starting");
        
        start_diagnostic_session();

        reading_answer_diagnostic(0x300, expected_response_starting_session, 5);
        delay(90);
        //test_frame();
        Identification_frame();
        
        Reading_answer_ident(0x300, expected_ident_response, 2);
        delay(90);

        First_Frame_adaptation();

        Reading_F_F_adapt();
        delay(100);

        frame_A3_sending();

        Reading_A3_response();

        Reading_S_F_adapt();
        
        delay(500);

        frame_A3_sending();
    }

    void loop()
    {
        unsigned long current_time = micros();  
        
        if (current_time - last_send_time_A3 >= send_interval_A3) 
        { 
            frame_A3_sending();
            last_send_time_A3 = current_time;
        }

        if (current_time - last_send_time_adaptation >= send_interval_adaptation)
        {
        frame_adaptation_sending();
        adaptation_status_reading();
        last_send_time_adaptation = current_time; 
        }
        
        
    }

    void start_diagnostic_session()
    {
        for (int i = 0; i < 10; i++)
        {
            can_frame frame_1;
            frame_1.can_id = first_frame.ID;
            frame_1.can_dlc = first_frame.len;
            memcpy(frame_1.data, first_frame.DATA, frame_1.can_dlc);
            mcp2515.sendMessage(&frame_1);
            delay(75);
        }
        
        can_frame frame_2;
        frame_2.can_id = first_diagnostic.ID;
        frame_2.can_dlc = first_diagnostic.len;
        memcpy(frame_2.data, first_diagnostic.DATA, frame_2.can_dlc);
        mcp2515.sendMessage(&frame_2);
        delay(250);

        can_frame frame_3;
        frame_3.can_id = second_frame.ID;
        frame_3.can_dlc = second_frame.len;
        memcpy(frame_3.data, second_frame.DATA, frame_3.can_dlc);
        mcp2515.sendMessage(&frame_3);
        delay(10);

        can_frame frame_4;
        frame_4.can_id = third_frame.ID;
        frame_4.can_dlc = third_frame.len;
        memcpy(frame_4.data, third_frame.DATA, frame_4.can_dlc);
        mcp2515.sendMessage(&frame_4);
        delay(10);

        can_frame frame_5;
        frame_5.can_id = second_diagnostic.ID;
        frame_5.can_dlc = second_diagnostic.len;
        memcpy(frame_5.data, second_diagnostic.DATA, frame_5.can_dlc);
        mcp2515.sendMessage(&frame_5);
        delay(150);

        can_frame frame_6;
        frame_6.can_id = start_session.ID;
        frame_6.can_dlc = start_session.len;
        memcpy(frame_6.data, start_session.DATA, frame_6.can_dlc);
        mcp2515.sendMessage(&frame_6);
        delay(75);
    }

    void frame_A3_sending()
    {
        can_frame frame;
        frame.can_id = frame_A3.ID;
        frame.can_dlc = frame_A3.len;
        memcpy(frame.data, frame_A3.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
    }

    void reading_answer_diagnostic(uint32_t expectedID, uint8_t expectedData[], uint8_t expectedLen)
    {
        while (true)
        {
            can_frame frame;

            mcp2515.readMessage(&frame);
            if(frame.can_id == expectedID)
            {
                if (frame.data[3] == 0x50 && frame.data[4] == 0x89)
                {
                B1_frame();
                return; 
                }
                
            }

        }
        
    }

    void B1_frame()
    {
        can_frame frame;
        frame.can_id = frame_B1.ID;
        frame.can_dlc = frame_B1.len;
        memcpy(frame.data, frame_B1.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
    }

    void Identification_frame()
    {
        can_frame frame;
        frame.can_id = identification_frame.ID;
        frame.can_dlc = identification_frame.len;
        memcpy(frame.data, identification_frame.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
    }

    void Reading_answer_ident(uint32_t expectedID, uint8_t expectedData[], uint8_t expectedLen)
    {
        while (true)
        {
            can_frame frame;

            mcp2515.readMessage(&frame);
            if (frame.can_id == expectedID && frame.can_dlc == expectedLen)
            {
                if (frame.data[0] == 0x18 && frame.data[1] == 0x20)
                {
                    B9_frame();
                    return;
                }
                
            }
            
        }
        
    }

    void B9_frame()
    {
        can_frame frame;
        frame.can_id = frame_B9.ID;
        frame.can_dlc = frame_B9.len;
        memcpy(frame.data, frame_B9.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
    }

    void First_Frame_adaptation()
    {
        can_frame frame;
        frame.can_id = first_frame_adaptation.ID;
        frame.can_dlc = first_frame_adaptation.len;
        memcpy(frame.data, first_frame_adaptation.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
    }

    void Reading_F_F_adapt()
    {
        while (true)
        {
            can_frame frame;

            mcp2515.readMessage(&frame);
            if (frame.can_id == 0x300)
            {
                if (frame.data[6] == 0x01 && frame.data[7] == 0x18)
                {
                    BB_frame();
                    return;
                }
                
            }
            
        }
        
    }

    void BB_frame()
    {
        can_frame frame;
        frame.can_id = frame_BB.ID;
        frame.can_dlc = frame_BB.len;
        memcpy(frame.data, frame_BB.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
    }

    void Reading_A3_response()
    {
        while (true)
        {
            can_frame frame;

            mcp2515.readMessage(&frame);
            if (frame.can_id == 0x300)
            {
                if (frame.data[3] == 0xFF && frame.data[4] == 0x54 && frame.data[5] == 0xFF)
                {
                    Second_Frame_Adaptation();
                    return;
                }
                
            }
            
        }
        
    }

    void Second_Frame_Adaptation()
    {
        can_frame frame;
        frame.can_id = second_frame_adaptation.ID;
        frame.can_dlc = second_frame_adaptation.len;
        memcpy(frame.data, second_frame_adaptation.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
    }

    void Reading_S_F_adapt()
    {
        while (true)
        {
            can_frame frame;

            mcp2515.readMessage(&frame);
            if (frame.can_id == 0x300)
            {
                if (frame.data[6] == 0x20 && frame.data[7] == 0xFF)
                {
                    BF_frame();
                    delay(400);
                    frame_A3_sending();
                    return;
                }
                
            }
            
        }
        
    }

    void BF_frame()
    {
        can_frame frame;
        frame.can_id = frame_BF.ID;
        frame.can_dlc = frame_BF.len;
        memcpy(frame.data, frame_BF.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
    }

    void frame_adaptation_sending()
    {
        can_frame frame;
        frame.can_id = main_frame_adaptation.ID;
        frame.can_dlc = main_frame_adaptation.len;
        memcpy(frame.data, main_frame_adaptation.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);

        main_frame_adaptation.DATA[0] += 0x1;
        if (main_frame_adaptation.DATA[0] == 0x20)
        {
            main_frame_adaptation.DATA[0] = 0x10;
        }
        
    }

    void adaptation_status_reading()
    {
        while (true)
        {
            can_frame frame;

            mcp2515.readMessage(&frame);
            if (frame.can_id == 0x300)
            {
                if (frame.data[3] == 0x25 && frame.data[4] == 0x00 && frame.data[5] == 0x00)
                {
                    B_ACK_Frame();
                    return;
                }
                
            }
            
        }
        
    }

    void B_ACK_Frame()
    {
        can_frame frame;
        frame.can_id = B_ACK_frame.ID;
        frame.can_dlc = B_ACK_frame.len;
        memcpy(frame.data, B_ACK_frame.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);

        uint8_t lower_nibble = B_ACK_frame.DATA[0] & 0x0F;  
        lower_nibble = (lower_nibble + 3) % 16;             
        
        B_ACK_frame.DATA[0] = 0xB0 | lower_nibble;
    }

    void test_frame()
    {
        can_frame frame;
        frame.can_id = tester.ID;
        frame.can_dlc = tester.len;
        memcpy(frame.data, tester.DATA, frame.can_dlc);
        mcp2515.sendMessage(&frame);
    }
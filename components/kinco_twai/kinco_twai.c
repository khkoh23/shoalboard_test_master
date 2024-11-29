#include "kinco_twai.h"

void kinco_twai_init (const uint8_t tx, const uint8_t rx) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        //printf("Driver installed\n");
    } else {
        //printf("Failed to install driver\n");
        return;
    }
    if (twai_start() == ESP_OK) {
        //printf("Driver started\n");
    } else {
        //printf("Failed to start driver\n");
        return;
    }
}

/* enable:0x2F/0x0F   disable:0x06   error reset:0x86   absolute positioning:0x103F
relative positioning:0x4F-->0x5F   homing:0x0F->0x1F
bit15:Manufacture4   bit14:Manufacture3   bit13:Manufacture2   bit12:Manufacture1
bit11:Manufacture0   bit10:Reserved1   bit9:Reserved0   bit8:Halt 
bit7:Fault_reset   bit6:Related_abs   bit5:Immed_change   bit4:Set_point
bit3:Enable_operation   bit2:Quick_stop   bit1:Enable_voltage   bit0:Switch_on
*/
void kinco_twai_setControlword (const uint8_t id, const uint16_t msg) { //6040 
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x40, 0x60, 0x00, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*
bit15:Reference_found   bit14:Commutation_found   bit13:Following_error   bit12:Setpoint_ack
bit11:Intlim_active   bit10:Target_reached   bit9:Remote   bit8:Manufacture0 
bit7:Warning   bit6:Switchon_disabled   bit5:Quick_stop   bit4:Voltage_enable
bit3:Fault   bit2:Operation_enable   bit1:Switched_on   bit0:Ready_on
*/
uint16_t kinco_twai_getStatusword (const uint8_t id) { //6041
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {
            uint8_t d1 = receive_message.data[4];
            uint8_t d2 = receive_message.data[5];
            return (d2*256 + d1);
        }
        else return 0;
    }
    else return 0;
}

/*1:position control   3:speed control   6:Homing mode   7:Interpolation mode
8:CSP    9:CSV   10: CST   4:Torque control   -4:Pulse train control   -3:Speed control
*/
void kinco_twai_setOperationMode (const uint8_t id, const int8_t msg) { //6060 
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x60, 0x60, 0x00, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        //printf("Message queued for transmission\n");
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {
            //printf("Message received\n");
        } 
        else {
            //printf("Failed to receive message\n");
            //return;
        }
    }
    else {
        //printf("Failed to queue message for transmission\n");
    } 
}

/*If "Enable" function is configured to Din, when the related Din Internal is 1, this
OD value would be set to "Controlword(6040.00)"
*/
void kinco_twai_setDinControlword (const uint8_t id, const uint16_t msg) { //2020:0F
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x20, 0x20, 0x0F, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*actual position of motor
*/
int32_t kinco_twai_getPosActual (const uint8_t id) { //6063
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x40, 0x63, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {
            uint8_t d1 = receive_message.data[4];
            uint8_t d2 = receive_message.data[5];
            uint8_t d3 = receive_message.data[6];
            uint8_t d4 = receive_message.data[7];
            return (d4*16777216 + d3*65536 + d2*256 + d1);
        }
        else return 0;
    }
    else return 0;
}

/*real speed after filter, sample time 250us
*/
int32_t kinco_twai_getSpeedReal (const uint8_t id) { //606C
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {
            uint8_t d1 = receive_message.data[4];
            uint8_t d2 = receive_message.data[5];
            uint8_t d3 = receive_message.data[6];
            uint8_t d4 = receive_message.data[7];
            return (d4*16777216 + d3*65536 + d2*256 + d1);
        }
        else return 0;
    }
    else return 0;
}

/*Invert motion. 0:CCW is positive direction   1: CW is positive direction
*/
void kinco_twai_setInvertDir (const uint8_t id, const uint8_t msg) { //607E
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x7E, 0x60, 0x00, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

/*(Mode1)
*/
void kinco_twai_setTargetPosition (const uint8_t id, const int32_t inc) { //607A 
    int32_t dec = inc; 
    uint8_t d1 = (dec & 0x000000FF) >> 0; 
    uint8_t d2 = (dec & 0x0000FF00) >> 8; 
    uint8_t d3 = (dec & 0x00FF0000) >> 16; 
    uint8_t d4 = (dec & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x7A, 0x60, 0x00, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*(Mode1)
DEC=(rpm*512*encoder_resolution)/1875
*/
void kinco_twai_setProfileSpeed (const uint8_t id, const double rpm) { //6081 Unsigned32
    if (rpm<0) return;
    uint32_t dec = rpm*2730.666666667;
    uint8_t d1 = (dec & 0x000000FF) >> 0; 
    uint8_t d2 = (dec & 0x0000FF00) >> 8; 
    uint8_t d3 = (dec & 0x00FF0000) >> 16; 
    uint8_t d4 = (dec & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x81, 0x60, 0x00, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*(Mode3,-3)
DEC=(rpm*512*encoder_resolution)/1875
*/
void kinco_twai_setTargetSpeed (const uint8_t id, const double rpm) { //60FF Integer32
    if (rpm>3000 || rpm<-3000) return;
    int32_t dec = rpm*2730.666666667; 
    uint8_t d1 = (dec & 0x000000FF) >> 0; 
    uint8_t d2 = (dec & 0x0000FF00) >> 8; 
    uint8_t d3 = (dec & 0x00FF0000) >> 16; 
    uint8_t d4 = (dec & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0xFF, 0x60, 0x00, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*Default:5000
*/
void kinco_twai_setMaxSpeedRPM (const uint8_t id, const uint16_t msg) { //6080
    uint32_t rpm = msg*1;
    uint8_t d1 = (rpm & 0x000000FF) >> 0; 
    uint8_t d2 = (rpm & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x80, 0x60, 0x00, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*(Mode1,3)
Default: 610.352
DEC=(rps/s*65536*encoder_resolution)/4000000
*/
void kinco_twai_setProfileAcc (const uint8_t id, const double rpsps) { //6083 Unsigned32
    uint32_t dec = rpsps*163.84;
    uint8_t d1 = (dec & 0x000000FF) >> 0; 
    uint8_t d2 = (dec & 0x0000FF00) >> 8; 
    uint8_t d3 = (dec & 0x00FF0000) >> 16; 
    uint8_t d4 = (dec & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x83, 0x60, 0x00, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*(Mode1,3)
Default: 610.352
DEC=(rps/s*65536*encoder_resolution)/4000000
*/
void kinco_twai_setProfileDec (const uint8_t id, const double rpsps) { //6084 Unsigned32
    uint32_t dec = rpsps*163.84;
    uint8_t d1 = (dec & 0x000000FF) >> 0; 
    uint8_t d2 = (dec & 0x0000FF00) >> 8; 
    uint8_t d3 = (dec & 0x00FF0000) >> 16; 
    uint8_t d4 = (dec & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x84, 0x60, 0x00, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*The velocity loop kp is Kvp[x], x=PI_pointer(60F9.28) (Range:1-32767 DEC)
*/
void kinco_twai_setKvp (const uint8_t id, const uint16_t msg) { //60F9:01
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0xF9, 0x60, 0x01, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*The velocity loop kp is Kvi[x], x=PI_pointer(60F9.28) (Range:1-1023)
*/
void kinco_twai_setKvi (const uint8_t id, const uint16_t msg) { //60F9:02
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0xF9, 0x60, 0x02, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*Ki2 of speed loop (Range:0-32767)
It is 1/31 of the normal kvi, using for high resolution encoder
*/
void kinco_twai_setKvi32 (const uint8_t id, const uint16_t msg) { //60F9:07
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0xF9, 0x60, 0x07, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*Bandwidth of speed feedback filter (Default:7 DEC, Range:0-45)
BW = Speed_Fb_N*20 + 100[Hz]
*/
void kinco_twai_setSpeedFbN (const uint8_t id, const uint8_t msg) { //60F9:05
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0xF9, 0x60, 0x05, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

/*Mode of speed feedback (Default:1)
0:2nd order FB LPF
1:No FB LPF
2:Observer FB
4:1st order FB LPF
10:2nd LPF + SPD_CMD FT
11:SPD_CMD FT
12:SPD_CMD FT + Observer
14:1st LPF + Observer
Bit7: 1:use 8k speed loop sampling and 2k position loop sampling frequency
Bit7: 0:use 4k speed loop sampling and 1k position loop sampling frequency 
*/
void kinco_twai_setSpeedMode (const uint8_t id, const uint8_t msg) { //60F9:06
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0xF9, 0x60, 0x06, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

/*Bandwidth of current command filter (Default:1 DEC, Range:1-127)
BW=2546/Output_Filter_N[Hz]
*/
void kinco_twai_setOutputFilterN (const uint8_t id, const uint8_t msg) { //60F9:15
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0xF9, 0x60, 0x15, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

/*Limiter of velocity control PI loop's integral part
*/
void kinco_twai_setKviSumLimit (const uint8_t id, const int32_t msg) { //60F9:08
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0xF9, 0x60, 0x08, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*Kp of position loop (Default:10 Hz, Range:0-327)
Each DEC is 0.01Hz
*/
void kinco_twai_setKpp (const uint8_t id, const int16_t msg) { //60FB:01
    uint16_t gain = msg*100;
    uint8_t d1 = (gain & 0x000000FF) >> 0; 
    uint8_t d2 = (gain & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0xFB, 0x60, 0x01, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*Velocity feedforward of position loop (Default: 100%, Range:0-100)
0 means no feedforward, 100 means 100% feed forward
*/
void kinco_twai_setKVelocityFF (const uint8_t id, const int16_t msg) { //60FB:02
    int16_t percent = msg*256*0.01;
    uint8_t d1 = (percent & 0x000000FF) >> 0; 
    uint8_t d2 = (percent & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0xFB, 0x60, 0x02, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*Acceleration feedforward of position loop (Range:0-32767 DEC)
Note only CD3 accept the unit "%", set by the "%" must be base on the right auto-tuning result.
Otherwise the value show by "%" could be not right
*/
void kinco_twai_setKAccFF (const uint8_t id, const int16_t msg) { //60FB:03
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0xFB, 0x60, 0x03, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*Average filter parameter (Default:2, Range:1-255)
Edit when it's not enabled!
*/
void kinco_twai_setPosFilterN (const uint8_t id, const uint16_t msg) { //60FB:05
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0xFB, 0x60, 0x05, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*Applied to set the value of maximal following error (Default:10000 DEC)
If exceed will give alarm 020.0
*/
void kinco_twai_setMaxFollowingError (const uint8_t id, const uint32_t msg) { //6065
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x65, 0x60, 0x00, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*Define function for Din1
0001:drive enable   0002:drive_reset error   0004:operation mode selection   0008:Kvi off
0010:position +ve lim   0020:posiion -ve lim   0040:homing signal   0080:invert direction
0100:Din velocity index0   0200:Din velocity index1   0400:Din position index0   0800:Din position index1
1000:quick stop   2000:start homing   4000:activate command (activate position command, controlword=0x2F->0x3F)
8001:Din velocity index2   8002: Din position index2   8004:Multifunction0   8008: Multifunction1
8010:Multifunction2   8020:gain switch0   8040:gain switch1   8080:MaxCur switch
8100:motor error   8200:pre enable   8400:fast capture 1   8800:fast capture 2
9001:postable cond0   9002:postable cond1   9004:start postable   9008:postable idx0
9010:postable idx1   9020:postable idx2   9040:abortpostable
A001:reset count   A002:halt   A004:positive jog   A008:negative jog
A010:brake release ok   A020:brake hold ok   A040:pulse adjust pos   A080:pulse adjust neg
*/
void kinco_twai_setDin1Function (const uint8_t id, const uint16_t msg) { //2010:03
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x10, 0x20, 0x03, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*Define function for Din2
0001:drive enable   0002:drive_reset error   0004:operation mode selection   0008:Kvi off
0010:position +ve lim   0020:posiion -ve lim   0040:homing signal   0080:invert direction
0100:Din velocity index0   0200:Din velocity index1   0400:Din position index0   0800:Din position index1
1000:quick stop   2000:start homing   4000:activate command (activate position command, controlword=0x2F->0x3F)
8001:Din velocity index2   8002: Din position index2   8004:Multifunction0   8008: Multifunction1
8010:Multifunction2   8020:gain switch0   8040:gain switch1   8080:MaxCur switch
8100:motor error   8200:pre enable   8400:fast capture 1   8800:fast capture 2
9001:postable cond0   9002:postable cond1   9004:start postable   9008:postable idx0
9010:postable idx1   9020:postable idx2   9040:abortpostable
A001:reset count   A002:halt   A004:positive jog   A008:negative jog
A010:brake release ok   A020:brake hold ok   A040:pulse adjust pos   A080:pulse adjust neg
*/
void kinco_twai_setDin2Function (const uint8_t id, const uint16_t msg) { //2010:04
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x10, 0x20, 0x04, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*Define the polarity of DIN signal: 0 is normally closed, 1 is normally open
bit0:Din1 
bit1:Din2
bit2:Din3
...     (default 0xFF)
bit7:Din8
*/
void kinco_twai_setDinPolarity (const uint8_t id, const uint16_t msg) { // 2010:01
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x10, 0x20, 0x01, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*bit0 to bit7 map to Din1 to Din8
*/
void kinco_twai_setDinSimulate (const uint8_t id, const uint16_t msg) { //2010:02
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x10, 0x20, 0x02, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*
bit0:Extended error (refer to 2602:00)   bit1:Encoder ABZ / not connected 
bit2:Encoder UVW / encoder internal   bit3:Encoder counting / encoder crc
bit4:Driver temperature    bit5:Over voltage
bit6:Under voltage   bit7:Over current
bit8:Chop resistor   bit9:Position following
bit10:Low logic voltage   bit11:Motor or driver iit
bit12:Over frequency   bit13:Motor temperature
bit14:Motor commutation   bit15:EEPROM checksum fault
*/
uint16_t kinco_twai_getErrorState (const uint8_t id) { //2601
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x40, 0x01, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {
            uint8_t d1 = receive_message.data[4];
            uint8_t d2 = receive_message.data[5];
            return (d2*256 + d1);
        }
        else return 0;
    }
    else return 0;
}

/*quick stop mode (Controlword.bits.Quick_stop=0, e.g. ControlWord 0x000F -> 0x000B)
    1:Enable_operation 0:Quick_stop 1:Enable_voltage 1:Switch_on
0:stop without control
1:stop by using ramp, then switch off
2:stop by using quick stop deceleration, then switch off
5:stop with profie deceleration, stay in quick stop active
6:stop with quick stop deceleration, stay in quick stop active
18: using motor winding for brake even encoder is wrong
*/
void kinco_twai_setQuickStopMode (const uint8_t id, const int16_t msg) { //605A
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x5A, 0x60, 0x00, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*shutdown stop mode (Controlword = 0x06) 
    0:Enable_operation 1:Quick_stop 1:Enable_voltage 0:Switch_on
0:stop without control
1:stop by using ramp, then switch off
2:stop by using quick stop deceleration, then switch off
*/
void kinco_twai_setShutdownStopMode (const uint8_t id, const int16_t msg) { //605B
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x5B, 0x60, 0x00, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*disable stop mode (Controlword.bit.Enable operation=0, e.g. Controlword = 0x0F -> 0x07)
    0:Enable_operation 1:Quick_stop 1:Enable_voltage 1:Switch_on
0:stop without control
1:stop by using ramp, then switch off
2:stop by using quick stop deceleration, then switch off
18:using motor winding for brake even encoder is wrong
*/
void kinco_twai_setDisableStopMode (const uint8_t id, const int16_t msg) { //605C
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x5C, 0x60, 0x00, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*halt mode (Controlword.bit.Halt=1, e.g. Controlword = 0x000F -> 0x010F)
1:stop by current ramp
2:stop by quick stop deceleration
*/
void kinco_twai_setHaltMode (const uint8_t id, const int16_t msg) { //605D
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x5D, 0x60, 0x00, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*fault stop mode 
0:stop without control
1:stop by using ramp, then switch off
2:stop by using quick stop deceleration, then switch off
18:using motor winding for brake even encoder is wrong
*/
void kinco_twai_setFaultStopMode (const uint8_t id, const int16_t msg) { //605E
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x5E, 0x60, 0x00, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*Deceleration for quick stop
*/
void kinco_twai_setQuickStopDec (const uint8_t id, const uint32_t msg) { //6085
    uint32_t rpsps = msg*163.84;
    uint8_t d1 = (rpsps & 0x000000FF) >> 0; 
    uint8_t d2 = (rpsps & 0x0000FF00) >> 8; 
    uint8_t d3 = (rpsps & 0x00FF0000) >> 16; 
    uint8_t d4 = (rpsps & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x85, 0x60, 0x00, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setRx1Id (const uint8_t id, const uint32_t msg) { //1400:01
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x00, 0x14, 0x01, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setRx1Transmission (const uint8_t id, const uint8_t msg) { //1400:02
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x00, 0x14, 0x02, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

void kinco_twai_setRx1InhibitTime (const uint8_t id, const uint16_t msg) { //1400:03
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x00, 0x14, 0x03, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setGroupRx1Pdo (const uint8_t id, const uint8_t msg) { //1600:00
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x00, 0x16, 0x00, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

void kinco_twai_setRx1Pdo1 (const uint8_t id, const uint32_t msg) { //1600:01
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x00, 0x16, 0x01, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setRx1Pdo2 (const uint8_t id, const uint32_t msg) { //1600:02
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x00, 0x16, 0x02, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*This RPDO_01 has been configured as COB-ID:0x20A, transmission type:default (0xFE)
Both node 1 (L) and node 2 (R): 0x60400010 (2-Byte Controlword)
*/
void kinco_twai_Rx1Pdo (const uint32_t cobid, const uint16_t msg) { 
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = cobid, .data_length_code = 2, .data = {d1, d2} };
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {}
    else {}
}

void kinco_twai_setRx2Id (const uint8_t id, const uint32_t msg) { //1401:01
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x01, 0x14, 0x01, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setRx2Transmission (const uint8_t id, const uint8_t msg) { //1401:02
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x01, 0x14, 0x02, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

void kinco_twai_setRx2InhibitTime (const uint8_t id, const uint16_t msg) { //1401:03
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x01, 0x14, 0x03, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setGroupRx2Pdo (const uint8_t id, const uint8_t msg) { //1601:00
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x01, 0x16, 0x00, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

void kinco_twai_setRx2Pdo1 (const uint8_t id, const uint32_t msg) { //1601:01
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x01, 0x16, 0x01, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setRx2Pdo2 (const uint8_t id, const uint32_t msg) { //1601:02
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x01, 0x16, 0x02, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*This RPDO_02 has been configured as COB-ID:0x30A, transmission type:default (0xFE)
For node 1 (L): 0x60FF0020 (4-Byte Target Speed) + 0x60C10220 (4-Byte dummy)
For node 2 (R): 0x60C10220 (4-Byte dummy) + 0x60FF0020 (4-Byte Target Speed)
*/
void kinco_twai_Rx2Pdo (const uint32_t cobid, const double rpm1, const double rpm2) { 
    if (rpm1>3000 || rpm1<-3000) return;
    if (rpm2>3000 || rpm2<-3000) return;
    int32_t dec1 = rpm1*2730.666666667; 
    uint8_t d1 = (dec1 & 0x000000FF) >> 0; 
    uint8_t d2 = (dec1 & 0x0000FF00) >> 8; 
    uint8_t d3 = (dec1 & 0x00FF0000) >> 16; 
    uint8_t d4 = (dec1 & 0xFF000000) >> 24; 
    int32_t dec2 = rpm2*2730.666666667; 
    uint8_t d5 = (dec2 & 0x000000FF) >> 0; 
    uint8_t d6 = (dec2 & 0x0000FF00) >> 8; 
    uint8_t d7 = (dec2 & 0x00FF0000) >> 16; 
    uint8_t d8 = (dec2 & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = cobid, .data_length_code = 8, .data = {d1, d2, d3, d4, d5, d6, d7, d8} };
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {}
    else {}
}

void kinco_twai_setRx3Id (const uint8_t id, const uint32_t msg) { //1402:01
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x02, 0x14, 0x01, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setRx3Transmission (const uint8_t id, const uint8_t msg) { //1402:02
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x02, 0x14, 0x02, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

void kinco_twai_setRx3InhibitTime (const uint8_t id, const uint16_t msg) { //1402:03
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x02, 0x14, 0x03, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setGroupRx3Pdo (const uint8_t id, const uint8_t msg) { //1602:00
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x02, 0x16, 0x00, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

void kinco_twai_setRx3Pdo1 (const uint8_t id, const uint32_t msg) { //1602:01
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x02, 0x16, 0x01, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setRx3Pdo2 (const uint8_t id, const uint32_t msg) { //1602:02
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x02, 0x16, 0x02, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*This RPDO_03 has following separate configuration, transmission type:default (0xFE)
For node 1 (L): COB-ID:0x401, 0x607A0020 (4-Byte TargetPosition) + 0x60810020 (4-Byte ProfileSpeed)
For node 2 (R): COB-ID:0x402, 0x607A0020 (4-Byte TargetPosition) + 0x60810020 (4-Byte ProfileSpeed)
*/
void kinco_twai_Rx3Pdo (const uint32_t cobid, const int32_t inc, const double rpm) { 
    if (rpm<0) return;
    int32_t dec1 = inc; 
    uint8_t d1 = (dec1 & 0x000000FF) >> 0; 
    uint8_t d2 = (dec1 & 0x0000FF00) >> 8; 
    uint8_t d3 = (dec1 & 0x00FF0000) >> 16; 
    uint8_t d4 = (dec1 & 0xFF000000) >> 24; 
    int32_t dec2 = rpm*2730.666666667; 
    uint8_t d5 = (dec2 & 0x000000FF) >> 0; 
    uint8_t d6 = (dec2 & 0x0000FF00) >> 8; 
    uint8_t d7 = (dec2 & 0x00FF0000) >> 16; 
    uint8_t d8 = (dec2 & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = cobid, .data_length_code = 8, .data = {d1, d2, d3, d4, d5, d6, d7, d8} };
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {}
    else {}
}

void kinco_twai_setRx4Id (const uint8_t id, const uint32_t msg) { //1403:01
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x03, 0x14, 0x01, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setRx4Transmission (const uint8_t id, const uint8_t msg) { //1403:02
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x03, 0x14, 0x02, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

void kinco_twai_setRx4InhibitTime (const uint8_t id, const uint16_t msg) { //1403:03
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x03, 0x14, 0x03, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setGroupRx4Pdo (const uint8_t id, const uint8_t msg) { //1603:00
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x03, 0x16, 0x00, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

void kinco_twai_setRx4Pdo1 (const uint8_t id, const uint32_t msg) { //1603:01
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x03, 0x16, 0x01, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setRx4Pdo2 (const uint8_t id, const uint32_t msg) { //1603:02
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x03, 0x16, 0x02, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*This RPDO_04 has following separate configuration, transmission type:default (0xFE) 
For node 1 (L): COB-ID:0x501, 0x60830020 (4-Byte ProfileAcc) + 0x60840020 (4-Byte ProfileDec)
For node 2 (R): COB-ID:0x502, 0x60830020 (4-Byte ProfileAcc) + 0x60840020 (4-Byte ProfileDec)
*/
void kinco_twai_Rx4Pdo (const uint32_t cobid, const double rpsps1, const double rpsps2) { 
    uint32_t dec1 = rpsps1*163.84;
    uint8_t d1 = (dec1 & 0x000000FF) >> 0; 
    uint8_t d2 = (dec1 & 0x0000FF00) >> 8; 
    uint8_t d3 = (dec1 & 0x00FF0000) >> 16; 
    uint8_t d4 = (dec1 & 0xFF000000) >> 24; 
    uint32_t dec2 = rpsps2*163.84;
    uint8_t d5 = (dec2 & 0x000000FF) >> 0; 
    uint8_t d6 = (dec2 & 0x0000FF00) >> 8; 
    uint8_t d7 = (dec2 & 0x00FF0000) >> 16; 
    uint8_t d8 = (dec2 & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = cobid, .data_length_code = 8, .data = {d1, d2, d3, d4, d5, d6, d7, d8} };
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {}
    else {}
}

void kinco_twai_setTx1Id (const uint8_t id, const uint32_t msg) { //1800:01
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x00, 0x18, 0x01, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setTx1Transmission (const uint8_t id, const uint8_t msg) { //1800:02
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x00, 0x18, 0x02, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

void kinco_twai_setTx1InhibitTime (const uint8_t id, const uint16_t msg) { //1800:03
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x00, 0x18, 0x03, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setTx1EventTimer (const uint8_t id, const uint16_t msg) { //1800:05
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x00, 0x18, 0x05, d1, d2, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setGroupTx1Pdo (const uint8_t id, const uint8_t msg) { //1A00:00
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x00, 0x1A, 0x00, d1, 0x00, 0x00, 0x00} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        else {}
    }
    else {} 
}

void kinco_twai_setTx1Pdo1 (const uint8_t id, const uint32_t msg) { //1A00:01
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x00, 0x1A, 0x01, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setTx1Pdo2 (const uint8_t id, const uint32_t msg) { //1A00:02
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x00, 0x1A, 0x02, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setTx1Pdo3 (const uint8_t id, const uint32_t msg) { //1A00:03
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x00, 0x1A, 0x03, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setEcanSyncPeriod (const uint8_t id, const uint32_t msg) { //1006:00
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x06, 0x10, 0x00, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

void kinco_twai_setSyncId (const uint8_t id, const uint32_t msg) { //1005:00
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    uint8_t d3 = (msg & 0x00FF0000) >> 16; 
    uint8_t d4 = (msg & 0xFF000000) >> 24; 
    twai_message_t transmit_message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x05, 0x10, 0x00, d1, d2, d3, d4} };
    twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        else {}
    }
    else {}
}

/*SYNChronization protocol
*/
void kinco_twai_Sync (void) {
    twai_message_t transmit_message = {.identifier = 0x80, .rtr = 0, .data_length_code = 0 };
    //twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        //if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {}
        //else {}
    }
    else {}
}

/*Network ManagemenT protocol
0x01:operational mode
0x02:stop mode
0x80:pre-operational mode
0x81:reset-application mode
0x82:reset-communication mode
*/
void kinco_twai_setNmt (const uint8_t id, const uint8_t msg) { 
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t transmit_message = {.identifier = 0x0, .rtr = 0, .data_length_code = 2, .data = {d1, id} };
    //twai_message_t receive_message;
    if (twai_transmit(&transmit_message, pdMS_TO_TICKS(transmit_wait)) == ESP_OK) {
        //if (twai_receive(&receive_message, pdMS_TO_TICKS(receive_wait)) == ESP_OK) {} 
        //else {}
    }
    else {} 
}

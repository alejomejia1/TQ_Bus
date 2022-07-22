#include <Arduino.h>
#include "torqeedo.h"

#define TORQEEDO_MESSAGE_LEN_MAX                        35      // messages are no more than 35 bytes
#define TORQEEDO_SERIAL_BAUD                            19200   // communication is always at 19200
#define TORQEEDO_PACKET_HEADER                          0xAC    // communication packet header
#define TORQEEDO_PACKET_FOOTER                          0xAD    // communication packet footer
#define TORQEEDO_PACKET_ESCAPE                          0xAE    // escape character for handling occurrences of header, footer and this escape bytes in original message
#define TORQEEDO_PACKET_ESCAPE_MASK                     0x80    // byte after ESCAPE character should be XOR'd with this value
#define TORQEEDO_LOG_TRQD_INTERVAL_MS                   5000    // log TRQD message at this interval in milliseconds
#define TORQEEDO_SEND_MOTOR_SPEED_INTERVAL_MS           100     // motor speed sent at 10hz if connected to motor
#define TORQEEDO_SEND_MOTOR_STATUS_REQUEST_INTERVAL_MS  400     // motor status requested every 0.4sec if connected to motor
#define TORQEEDO_SEND_MOTOR_PARAM_REQUEST_INTERVAL_MS   400     // motor param requested every 0.4sec if connected to motor
#define TORQEEDO_BATT_TIMEOUT_MS                        5000    // battery info timeouts after 5 seconds
#define TORQEEDO_REPLY_TIMEOUT_MS                       25      // stop waiting for replies after 25ms
#define TORQEEDO_ERROR_REPORT_INTERVAL_MAX_MS           10000   // errors reported to user at no less than once every 10 seconds

#define THROTLE_PIN 23
#define MIX_PIN 19


void TorqeedoMotor::begin(uint8_t ser,uint8_t tx, uint8_t rx, uint8_t rts, uint8_t onoff)
{
    _rts = rts;
    _onoff = onoff;
    _ser = ser;
    _type == ConnectionType::TYPE_TILLER;
    _starttime = millis();
    _order = 0;

    pinMode(_rts, OUTPUT); 
    pinMode(_onoff, OUTPUT);


    if(_ser==1) {
        Serial1.begin(19200,SERIAL_8N1, tx, rx);
        digitalWrite(_rts,0);
    } 

    if (_ser==2) {
        Serial2.begin(19200,SERIAL_8N1, tx, rx);
        digitalWrite(_rts,0);
        
    }
    
}

void TorqeedoMotor::On()
{
    digitalWrite(_onoff, HIGH);
    delay(500); // Wait 500 ms
    digitalWrite(_onoff, LOW);
    _startup_timer = millis();
}

void TorqeedoMotor::Off()
{
    digitalWrite(_onoff, HIGH);
    delay(6000);
    digitalWrite(_onoff, LOW);
}

int16_t TorqeedoMotor::getOrder() {
    int16_t channel;
    int16_t channel1;

    channel = pulseIn(THROTLE_PIN,HIGH);
    // channel1 = pulseIn(MIX_PIN,HIGH);


    // channel1 = 1500;

    int16_t throttleOrder = map(channel, 998, 2000, -1000, 1000);
    int16_t mixOrder = map(channel1, 998, 2000, -1000, 1000);

    if (throttleOrder > -60 && throttleOrder < 60) throttleOrder = 0;

    // Serial.print("Pulse:"); Serial.println(throttleOrder);
    // Serial.print("Mix:"); Serial.println(mixOrder);
    return throttleOrder;
}

void TorqeedoMotor::loop(int16_t throttleOrder) 
{
    _initialised = true;
    _throttleOrder = throttleOrder;

    // 1ms loop delay
    delay(1);

    // if (_send_motor_speed) {
    //     send_motor_speed_cmd();
    //     _send_motor_speed = false;
    //     log_update = true;
    // }

    // check if transmit pin should be unset
    check_for_send_end();

    // check for timeout waiting for reply
    check_for_reply_timeout();

    // parse incoming characters
    if (_ser == 1) {
        uint32_t nbytes = Serial1.available();
        while (nbytes-- > 0) {
            int16_t b = Serial1.read();
            if (b >= 0 ) {
                if (parse_byte((uint8_t)b)) {
                    // complete message received, parse it!
                    parse_message();
                    // clear wait-for-reply because if we are waiting for a reply, this message must be it
                    set_reply_received();
                }
            }
        }
    } else {
        uint32_t nbytes = Serial2.available();
        while (nbytes-- > 0) {
            int16_t b = Serial2.read();
            if (b >= 0 ) {
                if (parse_byte((uint8_t)b)) {
                    // complete message received, parse it!
                    parse_message();
                    // clear wait-for-reply because if we are waiting for a reply, this message must be it
                    set_reply_received();
                }
            }
        }
    }


    // send motor speed
    bool log_update = false;
    // if (true) {
    if (safe_to_send()) {
        uint32_t now_ms = millis();

        // if connected to motor
        if (_type == ConnectionType::TYPE_MOTOR) {
            
        }

        
        // send motor speed
        // if (true) {
        if (_send_motor_speed) {
            send_motor_speed_cmd();
            _send_motor_speed = false;
            log_update = true;
        }
    }
    

    
}

static const uint8_t crc8_table_maxim[] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};


uint8_t TorqeedoMotor::crc8_maxim(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0x0;

    while (length--) {
        crc = crc8_table_maxim[crc ^ *data];
        data++;
    }

    return crc;
}

// /*
//  CRC8-Maxim implementation based on FastCRC library
//  see https://github.com/FrankBoesing/FastCRC
//  */
// static const uint8_t crc8_table_maxim[] = {
//     0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
//     0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
//     0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
//     0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
//     0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
//     0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
//     0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
//     0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
//     0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
//     0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
//     0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
//     0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
//     0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
//     0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
//     0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
//     0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
//     0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
//     0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
//     0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
//     0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
//     0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
//     0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
//     0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
//     0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
//     0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
//     0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
//     0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
//     0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
//     0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
//     0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
//     0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
//     0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
// };

// uint8_t TorqeedoMotor::crc8_maxim(const uint8_t *data, uint16_t length)
// {
//     uint16_t crc = 0x0;

//     while (length--) {
//         crc = crc8_table_maxim[crc ^ *data];
//         data++;
//     }

//     return crc;
// }


// report changes in error codes to user
void TorqeedoMotor::report_error_codes()
{
    // skip reporting if we have already reported status very recently
    const uint32_t now_ms = millis();

    // skip reporting if no changes in flags and already reported within 10 seconds
    const bool flags_changed = (_display_system_state_flags_prev.value != _display_system_state.flags.value) ||
                               (_display_system_state_master_error_code_prev != _display_system_state.master_error_code) ||
                               (_motor_status_prev.status_flags_value != _motor_status.status_flags_value) ||
                               (_motor_status_prev.error_flags_value != _motor_status.error_flags_value);
    if (!flags_changed && ((now_ms - _last_error_report_ms) < TORQEEDO_ERROR_REPORT_INTERVAL_MAX_MS)) {
        return;
    }

    // report display system errors
    const char* msg_prefix = "Torqeedo:";
    if (_display_system_state.flags.set_throttle_stop) {
        Serial.print(msg_prefix);
        Serial.println(" zero throttle required");
    }
    if (_display_system_state.flags.temp_warning) {
        Serial.print(msg_prefix);
        Serial.println(" high temp");
    }
    if (_display_system_state.flags.temp_warning) {
        Serial.print(msg_prefix);
        Serial.println(" batt nearly empty");
    }
    if (_display_system_state.master_error_code > 0) {
        const char *error_string = map_master_error_code_to_string(_display_system_state.master_error_code);
        if (error_string != nullptr) {
            Serial.print(" err: ");
            Serial.print(msg_prefix);
            Serial.print(" ");
            Serial.print(_display_system_state.master_error_code);
            Serial.println(error_string);
        } else {
            Serial.print(" err: ");
            Serial.print(msg_prefix);
            Serial.print(" ");
            Serial.println(_display_system_state.master_error_code);
        }
    }

    // report motor status errors
    if (_motor_status.error_flags.overcurrent) {
        Serial.print(msg_prefix);
        Serial.println("overcurrent");
    }
    if (_motor_status.error_flags.blocked) {
        Serial.print(msg_prefix);
        Serial.println("prop blocked");
    }
    if (_motor_status.error_flags.overvoltage_static || _motor_status.error_flags.overvoltage_current) {
        Serial.print(msg_prefix);
        Serial.println(" high voltage");
    }
    if (_motor_status.error_flags.undervoltage_static || _motor_status.error_flags.undervoltage_current) {
        Serial.print(msg_prefix);
        Serial.println(" low voltage");
    }
    if (_motor_status.error_flags.overtemp_motor || _motor_status.error_flags.overtemp_pcb) {
        Serial.print(msg_prefix);
        Serial.println(" high temp");
    }
    if (_motor_status.error_flags.timeout_rs485) {
        Serial.print(msg_prefix);
        Serial.println(" comm timeout");
    }
    if (_motor_status.error_flags.temp_sensor_error) {
        Serial.print(msg_prefix);
        Serial.println(" temp sensor err");
    }
    if (_motor_status.error_flags.tilt) {
        Serial.print(msg_prefix);
        Serial.print(" tilted");
    }

    // display OK if all errors cleared
    const bool prev_errored = (_display_system_state_flags_prev.value != 0) ||
                              (_display_system_state_master_error_code_prev != 0) ||
                              (_motor_status_prev.error_flags_value != 0);

    const bool now_errored = (_display_system_state.flags.value != 0) ||
                             (_display_system_state.master_error_code != 0) ||
                             (_motor_status.error_flags_value != 0);

    if (!now_errored && prev_errored) {
        Serial.print(msg_prefix);
        Serial.print(" OK");
    }

    // record change in state and reporting time
    _display_system_state_flags_prev.value = _display_system_state.flags.value;
    _display_system_state_master_error_code_prev = _display_system_state.master_error_code;
    _motor_status_prev = _motor_status;
    _last_error_report_ms = now_ms;
}

// get latest battery status info.  returns true on success and populates arguments
bool TorqeedoMotor::get_batt_info(float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining)
{

    // use battery info from display_system_state if available (tiller connection)
    if ((millis() - _display_system_state.last_update_ms) <= TORQEEDO_BATT_TIMEOUT_MS) {
        voltage = _display_system_state.batt_voltage;
        current_amps = _display_system_state.batt_current;
        temp_C = max(_display_system_state.temp_sw, _display_system_state.temp_rp);
        pct_remaining = _display_system_state.batt_charge_pct;
        return true;
    }

    return false;
}

// get battery capacity.  returns true on success and populates argument
bool TorqeedoMotor::get_batt_capacity_Ah(uint16_t &amp_hours)
{
    if (_display_system_setup.batt_capacity == 0) {
        return false;
    }
    amp_hours = _display_system_setup.batt_capacity;
    return true;
}



// process a single byte received on serial port
// return true if a complete message has been received (the message will be held in _received_buff)
bool TorqeedoMotor::parse_byte(uint8_t b)
{
    bool complete_msg_received = false;

    switch (_parse_state) {
    case ParseState::WAITING_FOR_HEADER:
        if (b == TORQEEDO_PACKET_HEADER) {
            _parse_state = ParseState::WAITING_FOR_FOOTER;
        }
        _received_buff_len = 0;
        _parse_escape_received = false;
        break;
    case ParseState::WAITING_FOR_FOOTER:
        if (b == TORQEEDO_PACKET_FOOTER) {
            _parse_state = ParseState::WAITING_FOR_HEADER;

            // check message length
            if (_received_buff_len == 0) {
                _parse_error_count++;
                break;
            }

            // check crc
            const uint8_t crc_expected = crc8_maxim(_received_buff, _received_buff_len-1);
            if (_received_buff[_received_buff_len-1] != crc_expected) {
                _parse_error_count++;
                break;
            }

            complete_msg_received = true;
            // Serial.println(_received_buff_len);

        } else {
            // escape character handling
            if (_parse_escape_received) {
                b ^= TORQEEDO_PACKET_ESCAPE_MASK;
                _parse_escape_received = false;
            } else if (b == TORQEEDO_PACKET_ESCAPE) {
                // escape character received, record this and ignore this byte
                _parse_escape_received = true;
                break;
            }
            // add to buffer
            _received_buff[_received_buff_len] = b;
            _received_buff_len++;
            if (_received_buff_len > TORQEEDO_MESSAGE_LEN_MAX) {
                // message too long
                _parse_state = ParseState::WAITING_FOR_HEADER;
                _parse_error_count++;
            }
        }
        break;
    }

    return complete_msg_received;
}

// mark reply received. should be called whenever a message is received regardless of whether we are actually waiting for a reply
void TorqeedoMotor::set_reply_received()
{
    _reply_wait_start_ms = 0;
}


// process message held in _received_buff
void TorqeedoMotor::parse_message()
{
    // message address (i.e. target of message)
    const MsgAddress msg_addr = (MsgAddress)_received_buff[0];
    
    // Serial.println("Parsing msg");

    //handle messages sent to "remote" (aka tiller)
    // if ((_type == ConnectionType::TYPE_TILLER) && (msg_addr == MsgAddress::REMOTE1)) {
    //     RemoteMsgId msg_id = (RemoteMsgId)_received_buff[1];
    //     Serial.println("Speed request Msg");
    //     if (msg_id == RemoteMsgId::REMOTE) {
    //         // request received to send updated motor speed
    //         _send_motor_speed = true;
    //     }
    //     return;
    // }

    if ((msg_addr == MsgAddress::REMOTE1)) {
        RemoteMsgId msg_id = (RemoteMsgId)_received_buff[1];
        // Serial.print(millis());
        // Serial.print(" Speed request Msg Motor ");
        // Serial.println(_ser);
        

        if (msg_id == RemoteMsgId::REMOTE) {
            // request received to send updated motor speed
            if (_received_buff_len == 3) {
                // Serial.println(_received_buff_len);
                _send_motor_speed = true;
            }
        }
        return;
    }
    

    // handle messages sent to Display
    if ((msg_addr == MsgAddress::LCD)) {
        DisplayMsgId msg_id = (DisplayMsgId)_received_buff[1];
        switch (msg_id) {
        case DisplayMsgId::SYSTEM_STATE :
            if (_received_buff_len == 30) {
                
                _display_system_state.flags.value = UINT16_VALUE(_received_buff[2], _received_buff[3]);
                _display_system_state.master_state = _received_buff[4]; // deprecated
                _display_system_state.master_error_code = _received_buff[5];
                _display_system_state.motor_voltage = UINT16_VALUE(_received_buff[6], _received_buff[7]) * 0.01;
                _display_system_state.motor_current = UINT16_VALUE(_received_buff[8], _received_buff[9]) * 0.1;
                _display_system_state.motor_power = UINT16_VALUE(_received_buff[10], _received_buff[11]);
                _display_system_state.motor_rpm = (int16_t)UINT16_VALUE(_received_buff[12], _received_buff[13]);
                _display_system_state.motor_pcb_temp = _received_buff[14];
                _display_system_state.motor_stator_temp = _received_buff[15];
                _display_system_state.batt_charge_pct = _received_buff[16];
                _display_system_state.batt_voltage = UINT16_VALUE(_received_buff[17], _received_buff[18]) * 0.01;
                _display_system_state.batt_current = UINT16_VALUE(_received_buff[19], _received_buff[20]) * 0.1;
                _display_system_state.gps_speed = UINT16_VALUE(_received_buff[21], _received_buff[22]);
                _display_system_state.range_miles = UINT16_VALUE(_received_buff[23], _received_buff[24]);
                _display_system_state.range_minutes = UINT16_VALUE(_received_buff[25], _received_buff[26]);
                _display_system_state.temp_sw = _received_buff[27];
                _display_system_state.temp_rp = _received_buff[28];
                _display_system_state.last_update_ms = millis();
                
                // // update esc telem sent to ground station
                const uint8_t esc_temp = max(_display_system_state.temp_sw, _display_system_state.temp_rp);
                const uint8_t motor_temp = max(_display_system_state.motor_pcb_temp, _display_system_state.motor_stator_temp);
                // update_esc_telem(_display_system_state.motor_rpm,
                //         _display_system_state.motor_voltage,
                //         _display_system_state.motor_current,
                //         esc_temp,
                //         motor_temp);

                // Serial.print("Display Motor State Msg: ");
                // Serial.print("Voltage: ");Serial.print(_display_system_state.motor_voltage);Serial.print(" ,");
                Serial.print(millis());
                Serial.print(" Mot # "); Serial.print(_ser);
                Serial.print(" Orden: "); Serial.print(_throttleOrder);
                Serial.print(" RPM: ");Serial.print(_display_system_state.motor_rpm);Serial.print(" ,");
                Serial.print("Temp Pcb Motor: ");Serial.print(_display_system_state.motor_pcb_temp);Serial.print(" ");
                Serial.print("Temp Stator Motor: ");Serial.print(_display_system_state.motor_stator_temp);Serial.print(" ");
                Serial.print("Bat Charge %: ");Serial.print(_display_system_state.batt_charge_pct);Serial.print(" ");
                Serial.print("Range Min: ");Serial.print(_display_system_state.range_minutes);Serial.println("");
                if(_display_system_state.master_error_code > 0) {
                    map_master_error_code_to_string(_display_system_state.master_error_code);
                }
                
                // report_error_codes();

                // Serial.println();    
            } else {
                // unexpected length
                _parse_error_count++;
            }
            break;
        case DisplayMsgId::SYSTEM_SETUP:
            if (_received_buff_len == 13) {
                // fill in display system setup
                _display_system_setup.flags = _received_buff[2];
                _display_system_setup.motor_type = _received_buff[3];
                _display_system_setup.motor_sw_version = UINT16_VALUE(_received_buff[4], _received_buff[5]);
                _display_system_setup.batt_capacity = UINT16_VALUE(_received_buff[6], _received_buff[7]);
                _display_system_setup.batt_charge_pct = _received_buff[8];
                _display_system_setup.batt_type = _received_buff[9];
                _display_system_setup.master_sw_version =  UINT16_VALUE(_received_buff[10], _received_buff[11]);

                // Serial.println("Display Setup Msg :");
                // Serial.print(millis());
                // Serial.print(" Mot # "); Serial.print(_ser);
                // Serial.print(" Bat Level: ");Serial.print(_display_system_setup.batt_charge_pct);Serial.print("");
                // Serial.print(" Bat Cap: ");Serial.print(_display_system_setup.batt_capacity);Serial.println("");
                // // Serial.print("Bat Type: ");Serial.print(_display_system_setup.batt_type);Serial.println("");

                
            } else {
                // unexpected length
                Serial.println("Error parsing Msg");
                _parse_error_count++;
            }
            break;
        default:
            // ignore message
            break;
        }
        return;
    }


    
    // Serial.print("-");

    // handle reply from motor
    if ( (msg_addr == MsgAddress::BUS_MASTER)) {
        // replies strangely do not return the msgid so we must have stored it
        MotorMsgId msg_id = (MotorMsgId)_reply_msgid;
        

        switch (msg_id) {

        case MotorMsgId::PARAM:
            if (_received_buff_len == 15) {
                _motor_param.rpm = (int16_t)UINT16_VALUE(_received_buff[2], _received_buff[3]);
                _motor_param.power = UINT16_VALUE(_received_buff[4], _received_buff[5]);
                _motor_param.voltage = UINT16_VALUE(_received_buff[6], _received_buff[7]) * 0.01;
                _motor_param.current = UINT16_VALUE(_received_buff[8], _received_buff[9]) * 0.1;
                _motor_param.pcb_temp = (int16_t)UINT16_VALUE(_received_buff[10], _received_buff[11]) * 0.1;
                _motor_param.stator_temp = (int16_t)UINT16_VALUE(_received_buff[12], _received_buff[13]) * 0.1;
                _motor_param.last_update_ms = millis();
  
                Serial.print(" Motor RPM: ");Serial.print(_motor_param.rpm);Serial.print(" ,");
                Serial.print(" Temp PCB: ");Serial.print(_motor_param.pcb_temp);Serial.print(" ,");
                Serial.print(" Temp Stator: ");Serial.print(_motor_param.stator_temp);Serial.print(" ,");

            } else {
                // unexpected length
                _parse_error_count++;
            }
            break;

        case MotorMsgId::STATUS:
            if (_received_buff_len == 6) {
                _motor_status.status_flags_value = _received_buff[2];
                _motor_status.error_flags_value = UINT16_VALUE(_received_buff[3], _received_buff[4]);

                // report any errors
                report_error_codes();
            } else {
                // unexpected length
                _parse_error_count++;
            }
            break;

        case MotorMsgId::INFO:
        case MotorMsgId::DRIVE:
        case MotorMsgId::CONFIG:
            // we do not process these replies
            break;

        default:
            // ignore unknown messages
            break;
        }
    }
}

// Convert 2 Hex bytes received to unsigned int 16 bits
uint16_t TorqeedoMotor::UINT16_VALUE(uint8_t byteH, uint8_t byteL)
{ 
    unsigned long MSB = byteH << 8;
    unsigned long LSB = byteL;
    uint16_t result = MSB + LSB;
    return result;
}

// Example "Remote (0x01)" reply message to allow tiller to control motor speed
// Byte     Field Definition    Example Value   Comments
// ---------------------------------------------------------------------------------
// byte 0   Header              0xAC
// byte 1   TargetAddress       0x00            see MsgAddress enum
// byte 2   Message ID          0x00            only master populates this. replies have this set to zero
// byte 3   Flags               0x05            bit0=pin present, bit2=motor speed valid
// byte 4   Status              0x00            0x20 if byte3=4, 0x0 is byte3=5
// byte 5   Motor Speed MSB     ----            Motor Speed MSB (-1000 to +1000)
// byte 6   Motor Speed LSB     ----            Motor Speed LSB (-1000 to +1000)
// byte 7   CRC-Maxim           ----            CRC-Maxim value
// byte 8   Footer              0xAD
//
// example message when rotating tiller handle forwards:  "AC 00 00 05 00 00 ED 95 AD FF"    (+237)
// example message when rotating tiller handle backwards: "AC 00 00 05 00 FF AE 2C 0C AD FF" (-82)

// send a motor speed command as a value from -1000 to +1000
// value is taken directly from SRV_Channel
// for tiller connection this sends the "Remote (0x01)" message
// for motor connection this sends the "Motor Drive (0x82)" message
void TorqeedoMotor::send_motor_speed_cmd()
{

    if (millis() - _startup_timer < 5000) {
        Serial.println("Initial speed always 0");
        _motor_speed_desired = 0;
    } else {
        _motor_speed_desired = _throttleOrder;
    }
   
    
    // Serial.println("Sending speed command");
    // updated limited motor speed
    int16_t mot_speed_limited = calc_motor_speed_limited(_motor_speed_desired);

    // Construct Speed Command Message
    uint8_t mot_speed_cmd_buff[] = {(uint8_t)MsgAddress::BUS_MASTER, 0x0, 0x5, 0x0, highByte(mot_speed_limited), lowByte(mot_speed_limited)};
    
    // send a message
    if (safe_to_send()) {
        if (send_message(mot_speed_cmd_buff, sizeof(mot_speed_cmd_buff))) {
            // Serial.print(millis());
            // Serial.println(" Speed cmd snd");
            _last_send_motor_ms = millis();
        }

    }
}


void TorqeedoMotor::send_tiller_init()
{

    uint8_t message_buff[] = {0x01, 0x01};
    // send a message
    if (send_message(message_buff, sizeof(message_buff))) {
        Serial.println("Tiller initial cmd");
        _last_send_motor_ms = millis();
    }

}



// calculate the limited motor speed that is sent to the motors
// desired_motor_speed argument and returned value are in the range -1000 to 1000
int16_t TorqeedoMotor::calc_motor_speed_limited(int16_t desired_motor_speed)
{
    const uint32_t now_ms = millis();

    // update dir_limit flag for forward-reverse transition delay
    const bool dir_delay_active = (_dir_delay > 0);
    if (!dir_delay_active) {
        // allow movement in either direction
        _dir_limit = 0;
    } else {
        // by default limit motor direction to previous iteration's direction
        if (_motor_speed_limited > 0) {
            _dir_limit = 1;
        } else if (_motor_speed_limited < 0) {
            _dir_limit = -1;
        } else {
            // motor speed is zero
            if ((_motor_speed_zero_ms != 0) && ((now_ms - _motor_speed_zero_ms) > (_dir_delay * 1000))) {
                // delay has passed so allow movement in either direction
                _dir_limit = 0;
                _motor_speed_zero_ms = 0;
            }
        }
    }

    // calculate upper and lower limits for forward-reverse transition delay
    int16_t lower_limit = -1000;
    int16_t upper_limit = 1000;
    if (_dir_limit < 0) {
        upper_limit = 0;
    }
    if (_dir_limit > 0) {
        lower_limit = 0;
    }

    // calculate dt since last update
    float dt = (now_ms - _motor_speed_limited_ms) * 0.001f;
    if (dt > 1.0) {
        // after a long delay limit motor output to zero to avoid sudden starts
        lower_limit = 0;
        upper_limit = 0;
    }
    _motor_speed_limited_ms = now_ms;

    // apply slew limit
    if (_slew_time > 0) {
       const float chg_max = 1000.0 * dt / _slew_time;
       _motor_speed_limited = constrain(desired_motor_speed, _motor_speed_limited - chg_max, _motor_speed_limited + chg_max);
    } else {
        // no slew limit
        _motor_speed_limited = desired_motor_speed;
    }

    // apply upper and lower limits
    _motor_speed_limited = constrain(_motor_speed_limited, lower_limit, upper_limit);

    // record time motor speed becomes zero
    if (_motor_speed_limited == 0) {
        if (_motor_speed_zero_ms == 0) {
            _motor_speed_zero_ms = now_ms;
        }
    } else {
        // clear timer
        _motor_speed_zero_ms = 0;
    }

    return (int16_t)_motor_speed_limited;
}

// send a message to the motor with the specified message contents
// msg_contents should not include the header, footer or CRC
// returns true on success
bool TorqeedoMotor::send_message(const uint8_t msg_contents[], uint8_t num_bytes)
{
    // Serial.print("Sending command ");
    // buffer for outgoing message
    uint8_t send_buff[TORQEEDO_MESSAGE_LEN_MAX];
    uint8_t send_buff_num_bytes = 0;

    // calculate crc
    const uint8_t crc = crc8_maxim(msg_contents, num_bytes);

    // add header
    send_buff[send_buff_num_bytes++] = TORQEEDO_PACKET_HEADER;

    // add contents
    for (uint8_t i=0; i<num_bytes; i++) {
        if (!add_byte_to_message(msg_contents[i], send_buff, sizeof(send_buff), send_buff_num_bytes)) {
            _parse_error_count++;
            return false;
        }
    }

    // add crc
    if (!add_byte_to_message(crc, send_buff, sizeof(send_buff), send_buff_num_bytes)) {
        _parse_error_count++;
        return false;
    }

    // add footer
    if (send_buff_num_bytes >= sizeof(send_buff)) {
        _parse_error_count++;
        return false;
    }
    send_buff[send_buff_num_bytes++] = TORQEEDO_PACKET_FOOTER;
    send_buff[send_buff_num_bytes++] = '\xFF';


    // for (uint8_t i=0; i<send_buff_num_bytes ; i++) {
    //     Serial.print(send_buff[i], HEX);
    //     Serial.print(" ");
    // }   
    // Serial.println("");
    
    // set send pin
    send_start();

    // write message
    if (_ser == 1) {
        Serial1.write(send_buff, send_buff_num_bytes);
    } else {
        Serial2.write(send_buff, send_buff_num_bytes); 
    }
    
    
    // record start and expected delay to send message
    _send_start_us = micros();
    _send_delay_us = calc_send_delay_us(send_buff_num_bytes);

    // for (uint8_t i=0; i<send_buff_num_bytes ; i++) {
    //     Serial.print(send_buff[i], HEX);
    //     Serial.print(" ");
    // }   
    // Serial.println("");
    // delay(6);
    // digitalWrite(_rts,0);

    return true;
}

// calculate delay require to allow bytes to be sent
uint32_t TorqeedoMotor::calc_send_delay_us(uint8_t num_bytes)
{
    // baud rate of 19200 bits/sec
    // total number of bits = 10 x num_bytes
    // convert from seconds to micros by multiplying by 1,000,000
    // plus additional 300us safety margin
    const uint32_t delay_us = 1e6 * num_bytes * 11 / TORQEEDO_SERIAL_BAUD + 300;
    return delay_us;
}

// add a byte to a message buffer including adding the escape character (0xAE) if necessary
// this should only be used when adding the contents to the buffer, not the header and footer
// num_bytes is updated to the next free byte
bool TorqeedoMotor::add_byte_to_message(uint8_t byte_to_add, uint8_t msg_buff[], uint8_t msg_buff_size, uint8_t &num_bytes) const
{
    bool escape_required = (byte_to_add == TORQEEDO_PACKET_HEADER ||
                            byte_to_add == TORQEEDO_PACKET_FOOTER ||
                            byte_to_add == TORQEEDO_PACKET_ESCAPE);

    // check if we have enough space
    if (num_bytes + (escape_required ? 2 : 1) >= msg_buff_size) {
        return false;
    }

    // add byte
    if (escape_required) {
        msg_buff[num_bytes++] = TORQEEDO_PACKET_ESCAPE;
        msg_buff[num_bytes++] = byte_to_add ^ TORQEEDO_PACKET_ESCAPE_MASK;
    } else {
        msg_buff[num_bytes++] = byte_to_add;
    }
    return true;
}

// set DE Serial CTS pin to enable sending commands to motor
void TorqeedoMotor::send_start()
{
    // set gpio pin or serial port's CTS pin
    digitalWrite(_rts, 1);
    delay(1);
}

// set DE Serial CTS pin to enable sending commands to motor
void TorqeedoMotor::debug(uint16_t buffer_size)
{
    // set gpio pin or serial port's CTS pin
    for (int i = 0; i < buffer_size; i++) {
                Serial.print(_received_buff[i], HEX);
                Serial.print(" ");
        }
    Serial.println();
}

// record msgid of message to wait for and set timer for timeout handling
void TorqeedoMotor::set_expected_reply_msgid(uint8_t msg_id)
{
    _reply_msgid = msg_id;
    _reply_wait_start_ms = millis();
}

// check for timeout after sending and unset pin if required
void TorqeedoMotor::check_for_send_end()
{
    if (_send_delay_us == 0) {
        // not sending
        return;
    }

    if (micros() - _send_start_us < _send_delay_us) {
        // return if delay has not yet elapsed
        return;
    }

    _send_delay_us = 0;

    // unset gpio or serial port's CTS pin
    digitalWrite(_rts, 0);
}

// check for timeout waiting for reply message
void TorqeedoMotor::check_for_reply_timeout()
{
    // return immediately if not waiting for reply
    if (_reply_wait_start_ms == 0) {
        return;
    }
    if (millis() - _reply_wait_start_ms > TORQEEDO_REPLY_TIMEOUT_MS) {
        _reply_wait_start_ms = 0;
        _parse_error_count++;
    }
}

// returns a human-readable string corresponding the the passed-in
// master error code (see page 93 of https://media.torqeedo.com/downloads/manuals/torqeedo-Travel-manual-DE-EN.pdf)
// If no conversion is available then nullptr is returned
const char * TorqeedoMotor::map_master_error_code_to_string(uint8_t code)
{
    switch (code) {
    case 2:
        return "stator high temp";
    case 5:
        return "propeller blocked";
    case 6:
        return "motor low voltage";
    case 7:
        return "motor high current";
    case 8:
        return "pcb temp high";
    case 21:
        return "tiller cal bad";
    case 22:
        return "mag bad";
    case 23:
        return "range incorrect";
    case 30:
        return "motor comm error";
    case 32:
        return "tiller comm error";
    case 33:
        return "general comm error";
    case 41:
    case 42:
        return "charge voltage bad";
    case 43:
        return "battery flat";
    case 45:
        return "battery high current";
    case 46:
        return "battery temp error";
    case 48:
        return "charging temp error";
    }

    return nullptr;
}
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



void TorqeedoMotor::begin(uint8_t ser,uint8_t tx, uint8_t rx, uint8_t rts, uint8_t onoff)
{
    if(ser==1) {
        Serial.println("Serial1 port selected");
        Serial1.begin(19200,SERIAL_8N1, tx, rx);
        digitalWrite(_rts,1);
        Serial1.print("Serial1 port selected");
        digitalWrite(_rts,0);
    } 

    if (ser==2) {
        Serial2.begin(19200,SERIAL_8N1, tx, rx);
    }

    _rts = rts;
    _onoff = onoff;
    _type == ConnectionType::TYPE_TILLER;
    
    pinMode(_rts, OUTPUT); 
    pinMode(_onoff, OUTPUT);
}

void TorqeedoMotor::On()
{
    digitalWrite(_onoff, HIGH);
    delay(500); // Wait 500 ms
    digitalWrite(_onoff, LOW);
}

void TorqeedoMotor::Off()
{
    digitalWrite(_onoff, HIGH);
    delay(6000);
    digitalWrite(_onoff, LOW);
}

void TorqeedoMotor::loop() 
{
    // check if transmit pin should be unset
    // check_for_send_end();

    // parse incoming characters
    uint32_t nbytes = Serial1.available();
    // Serial.print("# bytes : " );
    // Serial.println(nbytes);
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

    // send motor speed
    bool log_update = false;
    // if (safe_to_send()) {

        // if connected to motor
        if (_type == ConnectionType::TYPE_MOTOR) {
            
        }

        // send motor speed
        if (_send_motor_speed) {
            send_motor_speed_cmd();
            _send_motor_speed = false;
            log_update = true;
        }
    // }
}


/*
 CRC8-Maxim implementation based on FastCRC library
 see https://github.com/FrankBoesing/FastCRC
 */
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
            //  debug(_received_buff_len);

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

    // handle messages sent to "remote" (aka tiller)
    // if ((_type == ConnectionType::TYPE_TILLER) && (msg_addr == MsgAddress::REMOTE1)) {
    //     RemoteMsgId msg_id = (RemoteMsgId)_received_buff[1];
    //     Serial.println("Speed request Msg");
    //     if (msg_id == RemoteMsgId::REMOTE) {
    //         // request received to send updated motor speed
    //         _send_motor_speed = true;
    //     }
    //     return;
    // }

    if (msg_addr == MsgAddress::REMOTE1) {
        RemoteMsgId msg_id = (RemoteMsgId)_received_buff[1];
        // Serial.println("Speed request Msg");
        if (msg_id == RemoteMsgId::REMOTE) {
            // request received to send updated motor speed
            _send_motor_speed = true;
        }
        return;
    }
    

    // handle messages sent to Display
    // if ((_type == ConnectionType::TYPE_TILLER) && (msg_addr == MsgAddress::LCD)) {
    //     DisplayMsgId msg_id = (DisplayMsgId)_received_buff[1];
    //     switch (msg_id) {
    //     case DisplayMsgId::SYSTEM_STATE :
    //         if (_received_buff_len == 30) {
    //             Serial.println("Dispay State Msg");
    //         } else {
    //             // unexpected length
    //             _parse_error_count++;
    //         }
    //         break;
    //     case DisplayMsgId::SYSTEM_SETUP:
    //         if (_received_buff_len == 13) {
    //             Serial.println("Dispay Setup Msg");
                
    //         } else {
    //             // unexpected length
    //             Serial.println("Error parsing Msg");
    //             _parse_error_count++;
    //         }
    //         break;
    //     default:
    //         // ignore message
    //         break;
    //     }
    //     return;
    // }

    // // handle reply from motor
    // if ((_type == ConnectionType::TYPE_MOTOR) && (msg_addr == MsgAddress::BUS_MASTER)) {
    //     // replies strangely do not return the msgid so we must have stored it
    //     MotorMsgId msg_id = (MotorMsgId)_reply_msgid;
    //     switch (msg_id) {

    //     case MotorMsgId::PARAM:
    //         if (_received_buff_len == 15) {
                
    //         } else {
    //             // unexpected length
    //             _parse_error_count++;
    //         }
    //         break;

    //     case MotorMsgId::STATUS:
    //         if (_received_buff_len == 6) {
                
    //         } else {
    //             // unexpected length
    //             _parse_error_count++;
    //         }
    //         break;

    //     case MotorMsgId::INFO:
    //     case MotorMsgId::DRIVE:
    //     case MotorMsgId::CONFIG:
    //         // we do not process these replies
    //         break;

    //     default:
    //         // ignore unknown messages
    //         break;
    //     }
    // }
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
// example message when rotating tiller handle forwards:  "AC 00 00 05 00 00 ED 95 AD"    (+237)
// example message when rotating tiller handle backwards: "AC 00 00 05 00 FF AE 2C 0C AD" (-82)

// send a motor speed command as a value from -1000 to +1000
// value is taken directly from SRV_Channel
// for tiller connection this sends the "Remote (0x01)" message
// for motor connection this sends the "Motor Drive (0x82)" message
void TorqeedoMotor::send_motor_speed_cmd()
{
    _motor_speed_desired = 500;
    // Serial.println("Sending speed command");
    // updated limited motor speed
    int16_t mot_speed_limited = calc_motor_speed_limited(_motor_speed_desired);

    // by default use tiller connection command
    uint8_t mot_speed_cmd_buff[] = {(uint8_t)MsgAddress::BUS_MASTER, 0x0, 0x5, 0x0, highByte(_motor_speed_desired), lowByte(_motor_speed_desired)};

    // update message if using motor connection
    if (_type == ConnectionType::TYPE_MOTOR) {
        const uint8_t motor_power = (uint8_t)constrain(_motor_power, 0, 100);
        mot_speed_cmd_buff[0] = (uint8_t)MsgAddress::MOTOR;
        mot_speed_cmd_buff[1] = (uint8_t)MotorMsgId::DRIVE;
        mot_speed_cmd_buff[2] = (mot_speed_limited == 0 ? 0 : 0x01) | (_motor_clear_error ? 0x04 : 0);  // 1:enable motor, 2:fast off, 4:clear error
        mot_speed_cmd_buff[3] = mot_speed_limited == 0 ? 0 : motor_power;   // motor power from 0 to 100


        // reset motor clear error request
        _motor_clear_error = false;
    }

    // for (uint8_t i=0; i<sizeof(mot_speed_cmd_buff) ; i++) {
    //     Serial.print(mot_speed_cmd_buff[i], HEX);
    //     Serial.print(" ");
    // }   
    // Serial.println("");
    
    // send a message
    if (send_message(mot_speed_cmd_buff, sizeof(mot_speed_cmd_buff))) {
        // record time of send for health reporting
        // set_expected_reply_msgid((uint8_t)msg_id);
    }
}

// calculate the limited motor speed that is sent to the motors
// desired_motor_speed argument and returned value are in the range -1000 to 1000
int16_t TorqeedoMotor::calc_motor_speed_limited(int16_t desired_motor_speed)
{

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

    // for (uint8_t i=0; i<send_buff_num_bytes ; i++) {
    //     Serial.print(send_buff[i], HEX);
    //     Serial.print(" ");
    // }   
    // Serial.println("");

    // set send pin
    
    send_buff[5] = 0xff;
    send_buff[6] = 0xe5;
    send_buff[7] = 0xd6;
    // write message
    
    
    send_start();
    Serial1.write(send_buff, send_buff_num_bytes);
    
    for (uint8_t i=0; i<send_buff_num_bytes ; i++) {
        Serial.print(send_buff[i], HEX);
        Serial.print(" ");
    }   
    Serial.println("");
    digitalWrite(_rts,0);

    return true;
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
}

// check for timeout after sending and unset pin if required
void TorqeedoMotor::check_for_send_end()
{
    // unset gpio or serial port's CTS pin
    digitalWrite(_rts, 0);
}
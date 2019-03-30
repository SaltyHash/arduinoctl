/* arduinoctl.ino
 * 
 * Library to allow control of an Arduino via a corresponding Python script.
 */


#include <Servo.h>


#define COMMAND_SET_PIN_MODE_OUTPUT 0x00
#define COMMAND_SET_PIN_MODE_INPUT  0x01
#define COMMAND_SET_PIN_MODE_PULLUP 0x02

#define COMMAND_DIGITAL_READ        0x03
#define COMMAND_DIGITAL_WRITE_LOW   0x04
#define COMMAND_DIGITAL_WRITE_HIGH  0x05

#define COMMAND_ANALOG_READ         0x06
#define COMMAND_SET_PWM             0x07

#define COMMAND_PLAY_TONE           0x08
#define COMMAND_PLAY_TONE_TIMED     0x09
#define COMMAND_STOP_TONE           0x0A

#define COMMAND_SHIFT_IN_MSB_FIRST  0x0B
#define COMMAND_SHIFT_IN_LSB_FIRST  0x0C
#define COMMAND_SHIFT_OUT_MSB_FIRST 0x0D
#define COMMAND_SHIFT_OUT_LSB_FIRST 0x0E

#define COMMAND_SET_AREF_DEFAULT    0x0F
#define COMMAND_SET_AREF_INTERNAL   0x10
#define COMMAND_SET_AREF_1V1        0x11
#define COMMAND_SET_AREF_2V56       0x12
#define COMMAND_SET_AREF_EXTERNAL   0x13

// Extended functions

#define COMMAND_ATTACH_SERVO        0x14
#define COMMAND_DETACH_SERVO        0x15
#define COMMAND_SET_SERVO_ANGLE     0x16
#define COMMAND_SET_SERVO_TIME      0x17

#define COMMAND_DIGITAL_READ_RANGE  0x18
#define COMMAND_DIGITAL_WRITE_RANGE 0x19

#define COMMAND_RESET               0x1A

#define COMMAND_ACK 0xAA


#define SERVO_CHANNELS 48
Servo SERVOS[SERVO_CHANNELS];


void setup() {
    Serial.begin(115200);
}

void loop() {
    // Receive command
    const int command = read_byte();
    
    // Now do it
    switch (command) {
        case COMMAND_ACK:
            write_ack();
            break;

        // Pin modes
        case COMMAND_SET_PIN_MODE_OUTPUT:
            pinMode(read_byte(), OUTPUT);
            write_ack();
            break;
        case COMMAND_SET_PIN_MODE_INPUT:
            pinMode(read_byte(), INPUT);
            write_ack();
            break;
        case COMMAND_SET_PIN_MODE_PULLUP:
            pinMode(read_byte(), INPUT_PULLUP);
            write_ack();
            break;

        // Digital IO
        case COMMAND_DIGITAL_READ:
            write_byte(digitalRead(read_byte()) == HIGH ? 0xFF : 0x00);
            break;
        case COMMAND_DIGITAL_WRITE_LOW:
            digitalWrite(read_byte(), LOW);
            write_ack();
            break;
        case COMMAND_DIGITAL_WRITE_HIGH:
            digitalWrite(read_byte(), HIGH);
            write_ack();
            break;

        // Analog IO
        case COMMAND_ANALOG_READ:
            command_analog_read();
            break;
        case COMMAND_SET_PWM:
            command_set_pwm();
            break;

        // Tones
        case COMMAND_PLAY_TONE:
            command_play_tone();
            break;
        case COMMAND_PLAY_TONE_TIMED:
            command_play_tone_timed();
            break;
        case COMMAND_STOP_TONE:
            noTone(read_byte());
            write_ack();
            break;

        // Shift in/out
        case COMMAND_SHIFT_IN_MSB_FIRST:
            command_shift_in(MSBFIRST);
            break;
        case COMMAND_SHIFT_IN_LSB_FIRST:
            command_shift_in(LSBFIRST);
            break;
        case COMMAND_SHIFT_OUT_MSB_FIRST:
            command_shift_out(MSBFIRST);
            break;
        case COMMAND_SHIFT_OUT_LSB_FIRST:
            command_shift_out(LSBFIRST);
            break;

        // Analog references
        case COMMAND_SET_AREF_DEFAULT:
            analogReference(DEFAULT);
            write_ack();
            break;
        case COMMAND_SET_AREF_INTERNAL:
#ifdef INTERNAL
            analogReference(INTERNAL);
#endif
            write_ack();
            break;
        case COMMAND_SET_AREF_1V1:
#ifdef INTERNAL1V1
            analogReference(INTERNAL1V1);
#endif
            write_ack();
            break;
        case COMMAND_SET_AREF_2V56:
#ifdef INTERNAL2V56
            analogReference(INTERNAL2V56);
#endif
            write_ack();
            break;
        case COMMAND_SET_AREF_EXTERNAL:
            analogReference(EXTERNAL);
            write_ack();
            break;

        // Extended functionality

        // Servos
        case COMMAND_ATTACH_SERVO:
            command_attach_servo();
            break;
        case COMMAND_DETACH_SERVO:
            SERVOS[read_byte()].detach();
            write_ack();
            break;
        case COMMAND_SET_SERVO_ANGLE:
            command_set_servo_angle();
            break;
        case COMMAND_SET_SERVO_TIME:
            command_set_servo_time();
            break;

        // Digital read/write range
        case COMMAND_DIGITAL_READ_RANGE:
            command_digital_read_range();
            break;
        case COMMAND_DIGITAL_WRITE_RANGE:
            command_digital_write_range();
            break;

        case COMMAND_RESET:
            command_reset();
            break;
    }
}

void command_analog_read() {
    const int value = analogRead(read_byte());
    write_byte(value >> 8);    // Send MSB
    write_byte(value);         // Send LSB
}

void command_set_pwm() {
    const int pin = read_byte();
    analogWrite(pin, read_byte());
    write_ack();
}

void command_play_tone() {
    const int pin = read_byte();

    int freq = read_byte() << 8;
    freq    |= read_byte();

    tone(pin, freq);
    write_ack();
}

void command_play_tone_timed() {
    const int pin = read_byte();

    int freq = read_byte() << 8;
    freq    |= read_byte();

    int duration = read_byte() << 24;
    duration |= read_byte() << 16;
    duration |= read_byte() << 8;
    duration |= read_byte();

    tone(pin, freq, duration);
    write_ack();
}

void command_shift_in(const int bit_order) {
    // TODO: Set pins as inputs/outputs?
    const int data_pin  = read_byte();
    const int clock_pin = read_byte();
    const int byte_cnt  = read_byte() + 1;

    // Set clock pin to be an output (not setting data pin to input because it could be either INPUT or INPUT_PULLUP).
    pinMode(clock_pin, OUTPUT);

    // Send back each byte as it comes in
    for (int i = 0; i < byte_cnt; i++) {
        write_byte(shiftIn(data_pin, clock_pin, bit_order));
    }
}

void command_shift_out(const int bit_order) {
    // TODO: Set pins as outputs?
    const int data_pin  = read_byte();
    const int clock_pin = read_byte();
    const int byte_cnt  = read_byte() + 1;

    // Set pins to be outputs
    pinMode(data_pin, OUTPUT);
    pinMode(clock_pin, OUTPUT);

    // Receive all the data first, to prevent buffer overrun
    byte data[byte_cnt] = {};
    for (int i = 0; i < byte_cnt; i++) {
        data[i] = read_byte();
    }

    // Send all the data out
    for (int i = 0; i < byte_cnt; i++) {
        shiftOut(data_pin, clock_pin, bit_order, data[i]);
    }

    write_ack();
}

void command_attach_servo() {
    const int channel = read_byte();
    const int pin     = read_byte();
    
    Servo servo = SERVOS[channel];
    if (servo.attached())
        servo.detach();
    servo.attach(pin);
    
    write_ack();
}

void command_set_servo_angle() {
    const int channel = read_byte();
    const int angle   = read_byte();

    SERVOS[channel].write(angle);

    write_ack();
}

void command_set_servo_time() {
    const int channel = read_byte();
    int time_us = read_byte() << 8;
    time_us    |= read_byte();

    SERVOS[channel].writeMicroseconds(time_us);

    write_ack();
}

void command_digital_read_range() {
    const int start_pin = read_byte();
    const int pin_count = read_byte() + 1;
    const int stop_pin  = start_pin + pin_count - 1;
    
    byte states;
    for (int i = 0; i < pin_count; i++) {
//    for (int pin = start_pin; pin <= stop_pin; pin++) {
        const int pin = start_pin + i;
        const byte pin_index = i % 8;
        
        if (pin_index == 0)
            states = 0;

        states |= digitalRead(pin) << (7 - pin_index);
        
        if (pin_index == 7 || pin == stop_pin)
            write_byte(states);
    }
}

void command_digital_write_range() {
    const int start_pin = read_byte();
    const int pin_count = read_byte() + 1;
    const int stop_pin  = start_pin + pin_count - 1;
    const int byte_cnt  = ((pin_count - 1) / 8) + 1;

    int pin = start_pin;
    for (int byte_i = 0; byte_i < byte_cnt; byte_i++) {
        const int data = read_byte();

        for (int bit_index = 7; bit_index >= 0; bit_index--) {
            digitalWrite(pin, data & (1 << bit_index));

            if (pin == stop_pin)
                break;
            pin++;
        }
    }
    
    write_ack();
}

void command_reset() {
    // Reset pins
    for (int pin = 0; pin < NUM_DIGITAL_PINS; pin++) {
        pinMode(pin, INPUT);
        digitalWrite(pin, LOW);
        noTone(pin);
    }

    // Reset analog reference
    analogReference(DEFAULT);

    // Reset servos
    for (int channel = 0; channel < SERVO_CHANNELS; channel++)
        SERVOS[channel].detach();

    write_ack();
}

/* Reads a single byte from Serial and returns it, blocking until a byte can be read. */
int read_byte() {
    while (Serial.available() < 1);
    return Serial.read();
}

void write_ack() {
    write_byte(COMMAND_ACK);
}

/* Blocks until the byte of data has been written to the Serial port. */
void write_byte(const int data) {
    while (Serial.write(data) == 0);
    Serial.flush();
}

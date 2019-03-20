# arduinoctl
Control an Arduino with Python.

### Usage
1. Use the Arduino IDE to load the `arduinoctl.ino` file into your Arduino.
2. In your Python code, import the particular Arduino model you have from the arduinoctl module.
3. Start using it!

### Example
```python
from arduinoctl import ArduinoUno

with ArduinoUno('/dev/ttyACM0') as arduino:
    # Set pin 2 to be an output
    arduino.set_pin_mode(2, arduino.PinMode.OUTPUT)
    
    # Set pin 2 high
    arduino.digital_write(2, True)
    
    # Read pin 3
    print('Pin 3 =', arduino.digital_read(3))
    
    # Get analog voltage from channel A3
    print('A3 Volts =', arduino.analog_read_voltage(3))
    
    # Set PWM on pin 9 to 50% duty cycle
    arduino.set_pwm_duty_cycle(9, 0.5)
    
    # Play a 440Hz tone on pin 10 for 500 milliseconds
    arduino.play_tone(10, 440, 500)
    
    # Shift data out over pins 4 (data) and 5 (clock) in MSB-first order
    arduino.shift_out(4, 5, True, b'Hello, world!')
    
    # Shift 32 bytes of data in over pins 6 (data) and 5 (clock) in LSB-first order
    data = arduino.shift_in(6, 5, False, 32)
    
    # Attach a servo on pin 15 to servo channel 0 so it can be controlled
    arduino.attach_servo(0, 15)
    arduino.set_servo_angle(90)
    ...
    arduino.set_servo_angle(180)
    
# The serial connection to the Arduino is automatically closed when exiting the `with` block
```

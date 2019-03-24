import unittest
from random import randint, seed

from arduinoctl import ArduinoUno, PinMode, AnalogReference

seed()


def randbool():
    """Returns True or False, chosen randomly."""
    return bool(randint(0, 1))


class TestArduinoctl(unittest.TestCase):
    ARDUINO_CLASS = ArduinoUno
    ARDUINO_TTY = '/dev/ttyACM0'
    DEBUG = False

    def setUp(self):
        self.arduino = self.ARDUINO_CLASS(self.ARDUINO_TTY, timeout=3)
        self.arduino.debug = self.DEBUG
        self.arduino.reset()

    def tearDown(self):
        self.arduino.reset()
        self.arduino.close()

    def test_shift_in(self):
        data_pin, clock_pin = 9, 10
        msb_first = randbool()
        byte_cnt = 1024

        self.arduino.set_pin_mode(data_pin, PinMode.OUTPUT)
        self.arduino.digital_write(data_pin, False)

        self.arduino.set_pin_mode(clock_pin, PinMode.OUTPUT)

        data = self.arduino.shift_in(data_pin, clock_pin, msb_first, byte_cnt)
        self.assertEqual(len(data), byte_cnt, 'Received a different number of bytes than requested.')

    def test_shift_out(self):
        data_pin, clock_pin = 9, 10
        msb_first = randbool()
        byte_cnt = 2014

        data = bytes(randint(0, 255) for _ in range(byte_cnt))

        self.arduino.set_pin_mode(data_pin, PinMode.OUTPUT)
        self.arduino.set_pin_mode(clock_pin, PinMode.OUTPUT)

        self.arduino.shift_out(9, 10, msb_first, data)

    def test_set_analog_reference(self):
        max_voltage = 5.0

        for (ref_source, ref_voltage) in (
                (AnalogReference.EXTERNAL, max_voltage),
                (AnalogReference.INTERNAL, None)
        ):
            self.arduino.set_analog_reference(ref_source, ref_voltage)

            voltage = self.arduino.analog_read_voltage(0)
            self.assertGreaterEqual(voltage, 0)
            self.assertLessEqual(voltage, max_voltage)

    def test_range(self):
        start_pin = 3
        pin_count = 17

        # Build list of random pin states
        states = []
        while all(states) or not any(states):
            states = [randbool() for _ in range(pin_count)]

        # Set pins to be outputs
        for pin in range(start_pin, start_pin + pin_count):
            self.arduino.set_pin_mode(pin, PinMode.OUTPUT)

        # Set pin states
        self.arduino.digital_write_range(start_pin, states)

        # Get pin states
        actual_states = self.arduino.digital_read_range(start_pin, pin_count)

        # Make sure they're the same
        self.assertListEqual(actual_states, states, 'Actual pin states did not match desired pin states.')


if __name__ == '__main__':
    unittest.main()

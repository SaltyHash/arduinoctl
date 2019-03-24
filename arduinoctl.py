"""
Control an Arduino.

TODO:
- Does using even a single servo "break" the normal PWM functionality for other pins on the same timer?
"""

from abc import ABC, abstractmethod
from enum import Enum, IntEnum
from itertools import islice
from math import ceil
from threading import RLock
from typing import ByteString, Mapping, Optional, Set, List, Collection

import serial


def get_byte(value: int, byte: int) -> int:
    return (value >> (8 * byte)) & 0xFF


class Arduino(ABC):
    BAUD_RATES = (300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200)

    class _SerialCommands(IntEnum):
        SET_PIN_MODE_OUTPUT = 0x00
        SET_PIN_MODE_INPUT = 0x01
        SET_PIN_MODE_INPUT_PULLUP = 0x02

        DIGITAL_READ = 0x03
        DIGITAL_WRITE_LOW = 0x04
        DIGITAL_WRITE_HIGH = 0x05

        ANALOG_READ = 0x06
        SET_PWM = 0x07

        PLAY_TONE = 0x08
        PLAY_TONE_TIMED = 0x09
        STOP_TONE = 0x0A

        SHIFT_IN_MSB_FIRST = 0x0B
        SHIFT_IN_LSB_FIRST = 0x0C
        SHIFT_OUT_MSB_FIRST = 0x0D
        SHIFT_OUT_LSB_FIRST = 0x0E

        SET_AREF_DEFAULT = 0x0F
        SET_AREF_INTERNAL = 0x10
        SET_AREF_1V1 = 0x11
        SET_AREF_2V56 = 0x12
        SET_AREF_EXTERNAL = 0x13

        # Extended functions

        ATTACH_SERVO = 0x14
        DETACH_SERVO = 0x15
        SET_SERVO_ANGLE = 0x16
        SET_SERVO_TIME = 0x17

        # [start pin] [pin count] --> [ceil(pin_count / 8) bytes]
        DIGITAL_READ_RANGE = 0x18
        DIGITAL_WRITE_RANGE = 0x19

        RESET = 0x1A

        ACK = 0xAA

    class AnalogReference(Enum):
        """
        Arduino AVR Boards (Uno, Mega, etc.):

        - DEFAULT: The default analog reference of 5 volts (on 5V Arduino boards) or 3.3 volts (on 3.3V Arduino boards).
        - INTERNAL: A built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328P and 2.56 volts on the
                    ATmega8 (not available on the Arduino Mega).
        - INTERNAL_1V1: A built-in 1.1V reference (Arduino Mega only).
        - INTERNAL_2V56: A built-in 2.56V reference (Arduino Mega only).
        - EXTERNAL: The voltage applied to the AREF pin (0 to 5V only) is used as the reference.

        Documentation: https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
        """

        DEFAULT = 0
        INTERNAL = 1
        INTERNAL_1V1 = 2
        INTERNAL_2V56 = 3
        EXTERNAL = 4

    class PinMode(Enum):
        OUTPUT = 0
        INPUT = 1
        INPUT_PULLUP = 2

    _ANALOG_REFERENCE_TO_SERIAL_COMMAND = {
        AnalogReference.DEFAULT: _SerialCommands.SET_AREF_DEFAULT,
        AnalogReference.INTERNAL: _SerialCommands.SET_AREF_INTERNAL,
        AnalogReference.INTERNAL_1V1: _SerialCommands.SET_AREF_1V1,
        AnalogReference.INTERNAL_2V56: _SerialCommands.SET_AREF_2V56,
        AnalogReference.EXTERNAL: _SerialCommands.SET_AREF_EXTERNAL
    }

    @property
    @abstractmethod
    def name(self) -> str:
        """Name of the particular Arduino product."""
        ...

    @property
    @abstractmethod
    def max_analog_channel(self) -> int:
        """Highest allowed analog input channel."""
        ...

    @property
    @abstractmethod
    def allowed_analog_references(self) -> Mapping[AnalogReference, float]:
        """Map of allowed analog references and their corresponding reference voltage."""
        ...

    @property
    @abstractmethod
    def pwm_pins(self) -> Set[int]:
        """Set of pins capable of PWM output."""
        ...

    @property
    @abstractmethod
    def servo_channels(self) -> int:
        """Number of servo channels available."""
        ...

    def __init__(
            self,
            tty: str,
            baud_rate: int = BAUD_RATES[-1],
            timeout: Optional[float] = None
    ):
        if baud_rate not in self.BAUD_RATES:
            raise ValueError(f'Baud rate {baud_rate} is invalid. Valid baud rates: {self.BAUD_RATES}')

        self.debug = False

        self._conn_lock = RLock()
        self._conn = serial.Serial(tty, baud_rate, timeout=timeout)
        self._conn.reset_input_buffer()
        self._conn.reset_output_buffer()
        self._sync()

        self._analog_references = dict(self.allowed_analog_references)
        self._curr_analog_reference = Arduino.AnalogReference.DEFAULT

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def _read_arduino(self, byte_count: int) -> bytes:
        """
        :raises TimeoutError: Connection timed out waiting to read the specified number of bytes. Input buffer is reset.
        """
        assert byte_count > 0

        with self._conn_lock:
            data = self._conn.read(byte_count)
            actual_byte_count = len(data)
            if actual_byte_count != byte_count:
                self._conn.reset_input_buffer()
                raise TimeoutError(f'Tried to read {byte_count} bytes, but only got {actual_byte_count}.')

        if self.debug:
            print(f'Recv {len(data):3}:', ' '.join(hex(datum) for datum in data))

        return data

    def _recv_ack(self) -> None:
        with self._conn_lock:
            data = self._read_arduino(1)[0]
            if data != Arduino._SerialCommands.ACK:
                raise UnexpectedResponseError(hex(Arduino._SerialCommands.ACK), hex(data))

    def _sync(self) -> None:
        timeout = self._conn.timeout
        try:
            self._conn.timeout = 1
            while True:
                self._write_arduino(self._SerialCommands.ACK)
                try:
                    self._recv_ack()
                    return
                except (TimeoutError, UnexpectedResponseError):
                    pass
        finally:
            self._conn.timeout = timeout

    @staticmethod
    def _validate_pin(pin: int) -> None:
        if pin is None or pin < 0:
            raise ValueError(f'Pin {pin} is invalid; must be >= 0.')

    @staticmethod
    def _validate_range(name, value, min_=None, max_=None) -> None:
        if min_ is None and max_ is None:
            raise ValueError('min_ and max_ cannot both be undefined.')
        if min_ is not None and value < min_:
            raise ValueError(f'Invalid value {value} for {name}; must be >= {min_}.')
        if max_ is not None and value > max_:
            raise ValueError(f'Invalid value {value} for {name}; must be <= {max_}.')

    def _validate_servo_channel(self, channel: int) -> None:
        max_channel = self.servo_channels - 1
        if channel < 0 or channel > max_channel:
            raise ValueError(f'Invalid servo channel {channel}; must be in range [0, {max_channel}].')

    def _write_arduino(self, *data: int) -> None:
        if self.debug:
            print(f'Send {len(data):3}:', ' '.join(hex(datum) for datum in data))

        with self._conn_lock:
            self._conn.write(bytes(data))
            self._conn.flush()

    def close(self) -> None:
        if self._conn and self._conn.is_open:
            self._conn.close()
            self._conn = None

    def reset(self) -> None:
        """
        Resets the Arduino's pins, analog reference, and servo channels to default states.

        This is NOT the same as pressing the RESET button, but it tries to accomplish effectively the same thing.
        """

        with self._conn_lock:
            self._write_arduino(Arduino._SerialCommands.RESET)
            self._recv_ack()

    def digital_read(self, pin: int) -> bool:
        self._validate_pin(pin)

        with self._conn_lock:
            self._write_arduino(Arduino._SerialCommands.DIGITAL_READ, pin)
            return self._read_arduino(1)[0] != 0

    def digital_read_range(self, start_pin: int, pin_count: int) -> List[bool]:
        self._validate_pin(start_pin)
        self._validate_range('pin count', pin_count, 0, 256)
        if pin_count == 0:
            # Weird, but okay
            return []

        with self._conn_lock:
            self._write_arduino(
                Arduino._SerialCommands.DIGITAL_READ_RANGE,
                start_pin,
                pin_count - 1
            )

            data = self._read_arduino(ceil(pin_count / 8))

        states = []
        for byte in data:
            states += [bool(byte & (1 << bit)) for bit in range(7, -1, -1)]

        return states[:pin_count]

    def digital_write(self, pin: int, value: bool) -> None:
        self._validate_pin(pin)

        with self._conn_lock:
            self._write_arduino(
                Arduino._SerialCommands.DIGITAL_WRITE_HIGH if value else Arduino._SerialCommands.DIGITAL_WRITE_LOW,
                pin
            )
            self._recv_ack()

    def digital_write_range(self, start_pin: int, states: Collection[bool]):
        self._validate_pin(start_pin)

        pin_count = len(states)
        self._write_arduino(
            Arduino._SerialCommands.DIGITAL_WRITE_RANGE,
            start_pin,
            pin_count - 1
        )

        data = 0
        for i, state in enumerate(states):
            bit_index = (7 - i) % 8

            if bit_index == 7:
                data = 0

            if state:
                data |= 1 << bit_index

            is_last_bit = i == (pin_count - 1)
            if bit_index == 0 or is_last_bit:
                self._write_arduino(data)

                if is_last_bit:
                    break

        self._recv_ack()

    def analog_read(self, channel: int) -> float:
        """
        :return: Analog voltage reading in the range [0.0, 1.0].
        """

        # Validate the channel
        if channel < 0 or channel > self.max_analog_channel:
            raise ValueError(
                f'Invalid channel {channel}; must be in the range [0, {self.max_analog_channel}] for the {self.name}.'
            )

        # Get analog data from Arduino
        with self._conn_lock:
            self._write_arduino(Arduino._SerialCommands.ANALOG_READ, channel)
            data = self._read_arduino(2)

        # MSB is first
        data = (data[0] << 8) | data[1]
        # Only keep the bottom 10 bits
        data &= 0x03FF
        # Scale to [0.0, 1.0]
        return data / 1023

    def analog_read_voltage(self, channel: int) -> float:
        """
        :return: Real analog voltage reading.
        """
        return self.get_analog_reference_voltage() * self.analog_read(channel)

    def set_pwm_duty_cycle(self, pin: int, duty_cycle: float) -> None:
        """
        The frequency of the PWM signal on most pins is approximately 490 Hz. On the Uno and similar boards, pins 5 and
        6 have a frequency of approximately 980 Hz.

        :param pin: The pin on which to output the PWM signal. Must be in the set of pwm_pins.
        :param duty_cycle: PWM duty cycle in the range [0.0, 1.0], where 0.0 is always off, and 1.0 is always on.
        """
        if pin not in self.pwm_pins:
            raise ValueError(
                f'Pin {pin} is not a valid PWM pin for the {self.name}; valid pins are: {sorted(self.pwm_pins)}'
            )

        # Clamp duty cycle to [0.0, 1.0]
        duty_cycle = min(max(duty_cycle, 0), 1)
        duty_cycle = int(round(duty_cycle * 255))

        with self._conn_lock:
            self._write_arduino(Arduino._SerialCommands.SET_PWM, pin, duty_cycle)
            self._recv_ack()

    def get_analog_reference(self) -> AnalogReference:
        return self._curr_analog_reference

    def get_analog_reference_voltage(self) -> float:
        return self._analog_references[self.get_analog_reference()]

    def play_tone(self, pin: int, frequency_hz: int, duration_ms: Optional[int] = None) -> None:
        self._validate_pin(pin)
        self._validate_range('frequency_hz', frequency_hz, min_=31, max_=2 ** 16 - 1)
        if duration_ms is not None:
            self._validate_range('duration_ms', duration_ms, min_=0, max_=2 ** 32 - 1)

        with self._conn_lock:
            if duration_ms is None:
                self._write_arduino(
                    Arduino._SerialCommands.PLAY_TONE,
                    pin,

                    # 2 bytes for frequency_hz
                    get_byte(frequency_hz, 1),
                    get_byte(frequency_hz, 0)
                )
            else:
                self._write_arduino(
                    Arduino._SerialCommands.PLAY_TONE_TIMED,
                    pin,

                    # 2 bytes for frequency_hz
                    get_byte(frequency_hz, 1),
                    get_byte(frequency_hz, 0),

                    # 4 bytes for duration_ms
                    get_byte(duration_ms, 3),
                    get_byte(duration_ms, 2),
                    get_byte(duration_ms, 1),
                    get_byte(duration_ms, 0)
                )
            self._recv_ack()

    def set_analog_reference(self, reference: AnalogReference, voltage: float = None) -> None:
        # Make sure the analog reference mode is supported
        if reference not in self._analog_references:
            raise ValueError(f'Analog reference {reference.name} is not supported by the {self.name}.')

        # Check external reference and voltage
        references = Arduino.AnalogReference
        if reference == references.EXTERNAL:
            # Perform voltage sanity checks
            if voltage is None:
                raise ValueError(f'Voltage must be defined for {reference.name} reference.')
            if voltage < 0 or voltage > 5:
                raise ValueError(f'Voltage must be in range [0, 5].')
        elif voltage is not None:
            raise ValueError(
                f'Voltage must only be defined for {references.EXTERNAL.name} reference, '
                f'not for {reference.name} reference.'
            )

        # Set the analog reference on the Arduino
        with self._conn_lock:
            self._write_arduino(self._ANALOG_REFERENCE_TO_SERIAL_COMMAND[reference])
            self._recv_ack()

        # Updating these variables after updating the Arduino just in case something goes wrong with the Arduino
        self._curr_analog_reference = reference
        if reference == references.EXTERNAL:
            # Set the EXTERNAL voltage reference
            self._analog_references[references.EXTERNAL] = voltage

    def set_pin_mode(self, pin: int, mode: PinMode) -> None:
        self._validate_pin(pin)

        commands = Arduino._SerialCommands
        with self._conn_lock:
            if mode == Arduino.PinMode.INPUT:
                self._write_arduino(commands.SET_PIN_MODE_INPUT)
            elif mode == Arduino.PinMode.INPUT_PULLUP:
                self._write_arduino(commands.SET_PIN_MODE_INPUT_PULLUP)
            elif mode == Arduino.PinMode.OUTPUT:
                self._write_arduino(commands.SET_PIN_MODE_OUTPUT)
            else:
                raise ValueError(f'Invalid pin mode {mode}.')
            self._write_arduino(pin)

            self._recv_ack()

    def shift_in(self, data_pin: int, clock_pin: int, msb_first: bool, byte_cnt: int = 1) -> bytearray:
        self._validate_pin(data_pin)
        self._validate_pin(clock_pin)
        if data_pin == clock_pin:
            raise ValueError(f'Data and clock pins must be different; they were both set to {data_pin}.')
        if byte_cnt <= 0:
            raise ValueError(f'Invalid byte count {byte_cnt}; must be > 0.')

        max_chunk_size = 256
        command = Arduino._SerialCommands.SHIFT_IN_MSB_FIRST if msb_first else \
            Arduino._SerialCommands.SHIFT_IN_LSB_FIRST
        data = bytearray(byte_cnt)

        with self._conn_lock:
            i = 0
            while i < byte_cnt:
                chunk_size = min(max_chunk_size, byte_cnt - i)
                self._write_arduino(command, data_pin, clock_pin, chunk_size - 1)
                data[i:i + chunk_size] = self._read_arduino(chunk_size)
                i += chunk_size

        return data

    def shift_out(self, data_pin: int, clock_pin: int, msb_first: bool, data: ByteString) -> None:
        self._validate_pin(data_pin)
        self._validate_pin(clock_pin)
        if data_pin == clock_pin:
            raise ValueError(f'Data and clock pins must be different; they were both set to {data_pin}.')

        max_chunk_size = 256
        command = Arduino._SerialCommands.SHIFT_OUT_MSB_FIRST if msb_first else \
            Arduino._SerialCommands.SHIFT_OUT_LSB_FIRST
        data_iter = iter(data)

        with self._conn_lock:
            while True:
                chunk = list(islice(data_iter, max_chunk_size))
                if not chunk:
                    break

                self._write_arduino(command, data_pin, clock_pin, len(chunk) - 1, *chunk)
                self._recv_ack()

    def stop_tone(self, pin: int) -> None:
        self._validate_pin(pin)

        with self._conn_lock:
            self._write_arduino(Arduino._SerialCommands.STOP_TONE, pin)
            self._recv_ack()

    def attach_servo(self, channel: int, pin: int) -> None:
        self._validate_servo_channel(channel)
        self._validate_pin(pin)

        with self._conn_lock:
            self._write_arduino(Arduino._SerialCommands.ATTACH_SERVO, channel, pin)
            self._recv_ack()

    def detach_servo(self, channel: int) -> None:
        self._validate_servo_channel(channel)

        with self._conn_lock:
            self._write_arduino(Arduino._SerialCommands.DETACH_SERVO, channel)
            self._recv_ack()

    def set_servo_angle(self, channel: int, angle: int) -> None:
        self._validate_servo_channel(channel)
        self._validate_range('servo angle', angle, 0, 180)

        with self._conn_lock:
            self._write_arduino(Arduino._SerialCommands.SET_SERVO_ANGLE, channel, angle)
            self._recv_ack()

    def set_servo_time(self, channel: int, time_us: int) -> None:
        self._validate_servo_channel(channel)
        self._validate_range('time_us', time_us, 544, 2400)

        with self._conn_lock:
            self._write_arduino(
                Arduino._SerialCommands.SET_SERVO_TIME,
                channel,
                get_byte(time_us, 1),
                get_byte(time_us, 0)
            )


class ArduinoMega2560(Arduino):
    @property
    def name(self) -> str:
        return 'Arduino Mega 2560'

    @property
    def max_analog_channel(self) -> int:
        return 14

    @property
    def allowed_analog_references(self) -> Mapping[Arduino.AnalogReference, float]:
        return {
            Arduino.AnalogReference.DEFAULT: 5.0,
            Arduino.AnalogReference.INTERNAL_1V1: 1.1,
            Arduino.AnalogReference.INTERNAL_2V56: 2.56,
            Arduino.AnalogReference.EXTERNAL: None
        }

    @property
    def pwm_pins(self) -> Set[int]:
        return {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45, 46}

    @property
    def servo_channels(self) -> int:
        return 48


class ArduinoMicro(Arduino):
    @property
    def name(self) -> str:
        return 'Arduino Micro'

    @property
    def max_analog_channel(self) -> int:
        return 11

    @property
    def allowed_analog_references(self) -> Mapping[Arduino.AnalogReference, float]:
        return {
            Arduino.AnalogReference.DEFAULT: 5.0,
            Arduino.AnalogReference.INTERNAL: 1.1,
            Arduino.AnalogReference.EXTERNAL: None
        }

    @property
    def pwm_pins(self) -> Set[int]:
        return {3, 5, 6, 9, 10, 11}

    @property
    def servo_channels(self) -> int:
        return 12


class ArduinoNano(Arduino):
    @property
    def name(self) -> str:
        return 'Arduino Nano'

    @property
    def max_analog_channel(self) -> int:
        return 7

    @property
    def allowed_analog_references(self) -> Mapping[Arduino.AnalogReference, float]:
        return {
            Arduino.AnalogReference.DEFAULT: 5.0,
            Arduino.AnalogReference.INTERNAL: 1.1,
            Arduino.AnalogReference.EXTERNAL: None
        }

    @property
    def pwm_pins(self) -> Set[int]:
        return {3, 5, 6, 9, 10, 11}

    @property
    def servo_channels(self) -> int:
        return 12


class ArduinoUno(Arduino):
    @property
    def name(self) -> str:
        return 'Arduino UNO'

    @property
    def max_analog_channel(self) -> int:
        return 5

    @property
    def allowed_analog_references(self) -> Mapping[Arduino.AnalogReference, float]:
        return {
            Arduino.AnalogReference.DEFAULT: 5.0,
            Arduino.AnalogReference.INTERNAL: 1.1,
            Arduino.AnalogReference.EXTERNAL: None
        }

    @property
    def pwm_pins(self) -> Set[int]:
        return {3, 5, 6, 9, 10, 11}

    @property
    def servo_channels(self) -> int:
        return 12


class ArduinoError(Exception):
    """Base class for Arduino exceptions."""
    pass


class NotSupportedError(ArduinoError):
    """Raised when a function is not supported by a particular Arduino."""
    pass


class UnexpectedResponseError(ArduinoError):
    """Raised when an unexpected response is received from the Arduino."""

    def __init__(self, expected, actual):
        ArduinoError.__init__(
            self,
            f'Received unexpected response from Arduino (expected: {repr(expected)}; actual: {repr(actual)}).'
        )

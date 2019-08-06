try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
except ImportError:
    ImportError("RPi.GPIO module not found")

try:
    from time import monotonic
except ImportError:
    from time import time as monotonic

import time
import warnings
import spidev


class NRF24:
    max_rf_channel =    127
    max_payload_size =  32

    # -----------------------------------------------------------------------
    # Register Map Table
    CONFIG =        0x00         # Configuration Register
    EN_AA =         0x01         # Enhanced ShockBurst - Enable Auto Acknowledgment
    EN_RXADDR =     0x02         # Enabled RX Addresses
    SETUP_AW =      0x03         # Setup of Address Widths
    SETUP_RETR =    0x04         # Setup of Automatic Retransmission
    RF_CH =         0x05         # RF Channel
    RF_SETUP =      0x06         # RF Setup Register
    STATUS =        0x07         # Status Register
    OBSERVE_TX =    0x08         # Transmit observe register
    RPD =           0x09         # Received Power Detector

    RX_ADDR_P0 = 0x0A   # Receive address data pipe 0
    RX_ADDR_P1 = 0x0B   # Receive address data pipe 1
    RX_ADDR_P2 = 0x0C   # Receive address data pipe 2
    RX_ADDR_P3 = 0x0D   # Receive address data pipe 3
    RX_ADDR_P4 = 0x0E   # Receive address data pipe 4
    RX_ADDR_P5 = 0x0F   # Receive address data pipe 5

    TX_ADDR = 0x10      # Transmit address

    RX_PW_P0 = 0x11     # Setting data bytes in RX payload in data pipe Px
    RX_PW_P1 = 0x12
    RX_PW_P2 = 0x13
    RX_PW_P3 = 0x14
    RX_PW_P4 = 0x15
    RX_PW_P5 = 0x16

    FIFO_STATUS =   0x17
    DYNPD =         0x1C    # Enable dynamic payload length
    FEATURE =       0x1D    # Feature Register
    # -----------------------------------------------------------------------
    # CONFIG bits
    MASK_RX_DR =            0b01000000  # Mask interrupt caused by RX_DR
    MASK_TX_DS =            0b00100000  # Mask interrupt caused by TX_DS
    MASK_MAX_RT =           0b00010000  # Mask interrupt caused by MAX_RT

    PWR_UP =                0b00000010  # Power Up
    PRIM_RX =               0b00000001  # RX/TX control

    # Cyclic Redundancy Check
    CRC_DISABLED =  0x00
    CRC_1 =         0b00001000  # CRC length = 1 byte
    CRC_2 =         0b00001100  # CRC length = 2 byte

    EN_CRC =        0b00001000  # Enable CRC
    CRCO =          0b00000100  # CRC encoding scheme

    # -----------------------------------------------------------------------
    # SETUP_AW bits - address width
    AW_ERROR =      0b00000000  # Illegal
    AW_3 =          0b00000001  # 3 bytes width
    AW_4 =          0b00000010  # 4 bytes
    AW_5 =          0b00000011  # 5 bytes

    # -----------------------------------------------------------------------
    # RF_SETUP
    CONT_WAVE =             0b10000000  # Enables continuous carrier transmit
    RF_DR_LOW =             0b00100000
    PLL_LOCK =              0b00010000  # Force PLL lock Signal
    RF_DR_HIGH =            0b00001000

    # For binary arythmetic - PA Level
    RF_PWR_LOW =            0b00000010
    RF_PWR_HIGH =           0b00000100

    # Radio Air Data Rate (Bit Rate)
    BR_1Mbps =   0b00
    BR_2Mbps =   0b01
    BR_250kbps = 0b11

    # Power Amplifier - RF output power , DC current Consumption
    PA_MAX =    0b11   # 0dBm , 11.3 mA
    PA_HIGH =   0b10   # -6dBm , 9 mA
    PA_LOW =    0b01   # -12dBM , 7.5 mA
    PA_MIN =    0b00   # -18dBM , 7 mA

    # -----------------------------------------------------------------------
    # STATUS
    RX_DR =                 0b01000000  # Clear bit for RX_DR
    TX_DS =                 0b00100000  # Clear bit for TX_DS
    MAX_RT =                0b00010000  # Clear bit for MAX_RT

    # -----------------------------------------------------------------------
    # FIFO STATUS
    RX_EMPTY =              0b00000010  # RX FIFO full
    # -----------------------------------------------------------------------
    # FEATURE
    EN_DPL =                0b00000100  # Enable Dynamic Payload Length
    EN_ACK_PAY =            0b00000010  # Enable Payloads With ACK
    EN_DYN_ACK =            0b00000001  # Enable the W_TX_PAYLOAD_NO_ACK command

    # -----------------------------------------------------------------------
    # SPI Commands
    R_REGISTER =            0b00000000  # Read command and status registers
    W_REGISTER =            0b00100000  # Write command and status registers
    R_RX_PAYLOAD =          0b01100001  # Read RX-payload
    W_TX_PAYLOAD =          0b10100000  # Write TX-payload
    FLUSH_TX =              0b11100001  # Flush TX FIFO
    FLUSH_RX =              0b11100010  # Flush RX FIFO
    REUSE_TX_PL =           0b11100011  # Reuse last transmitted payload
    R_RX_PL_WID =           0b01100000  # Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO
    W_ACK_PAYLOAD =         0b10101000  # (RX mode) Write Payload to be transmitted together with ACK packet on Pipe
    W_TX_PAYLOAD_NO_ACK =   0b10110000  # (TX mode) Disables AUTOACK on specific packet
    NOP =                   0b11111111  # No Operation

    REGISTER_MASK = 0b00011111          # For Register Map Address in W_REGISTER

    # -----------------------------------------------------------------------

    @staticmethod
    def _convert_to_byte_list(data):     # do also for strings?
        # Converts data to list of bytes
        if isinstance(data, int):
            temp = [data]
        else:
            temp = [int(var) for var in data]
        for byte in temp:
            if 0 > byte > 255:
                raise ValueError("Cannot convert value: {}".format(byte))
        return temp

    def _data_rate_bits(self):
        # Interpretation of Data Rate bits in kbps
        bits = {
            0b00: 1000,
            0b01: 2000,
            0b11: 250,
        }
        rate_bits = bits.get(self.data_rate)
        if rate_bits is None:
            raise ValueError("Such Data Rate does not exist ")
        else:
            return rate_bits

    def __init__(self, spi_bus=None, spi_device=None, ce_pin=None, irq_pin=None):
        self.spidev = None

        self.ce_pin = ce_pin
        self.irq_pin = irq_pin

        self.channel = 76
        self.data_rate = NRF24.BR_1Mbps
        self.payload_size = 5

        self.retries = 3
        self.delay = 0.00025    # Do not change
        self.crc_length = 0
        self.address_width = 5

        self.pipe0_reading_address = None

        self.ack_payload_available = False
        # self.ack_payload_length = 5
        self.auto_ack = 0

        self.dynamic_payloads_enabled = False

        self.last_error = None

        if all(param is not None for param in [spi_bus, spi_device, ce_pin, irq_pin]):  # TODO:  is irq_pin necessery?
            self.begin(spi_bus, spi_device, ce_pin, irq_pin)

    def spi_init(self, spi_bus, spi_device):
        # Initialize SPI Connection
        self.spidev = spidev.SpiDev()
        self.spidev.open(spi_bus, spi_device)

        self.spidev.bits_per_word = 8
        self.spidev.max_speed_hz = 500000   # some bugs occur with bigger values (should be 10 MHz as max NRF freq)
        # TODO:  IOError for max_speed?

        self.spidev.cshigh = False      # Is CS active high = No
        self.spidev.mode = 0            # Clock polarity and clock phase = 0
        self.spidev.loop = False        # Echo back = No
        self.spidev.lsbfirst = False    # LSB transferred last
        self.spidev.threewire = False   # common line for MOSI and MISO = No

    def begin(self, spi_bus, spi_device, ce_pin, irq_pin):
        # Setup Configuration
        self.spi_init(spi_bus, spi_device)

        self.ce_pin = ce_pin
        self.irq_pin = irq_pin

        if self.ce_pin is not None:
            GPIO.setup(self.ce_pin, GPIO.OUT)

        if self.irq_pin is not None:
            GPIO.setup(self.irq_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        time.sleep(0.001)
        self.reset()    # Reset Configuration

        # Default Settings
        self.set_retries(5, self.retries)
        self.set_PA_level(NRF24.PA_MAX)
        self.set_data_rate(self.data_rate)
        self.set_CRC_length(self.crc_length)

        self.write_register(NRF24.DYNPD, 0)     # Dynamic payloads disabled

        self.set_channel(self.channel)

        # Clear read, write and interrupt lines
        self.flush_rx()
        self.flush_tx()
        self.clear_irq_flags()

    def end(self):
        if self.spidev:
            self.spidev.close()
            self.spidev = None

    def reset(self):
        reset_values = {
            0x00:   0b00001000,
            0x01:   0b00111111,
            0x02:   0b00000011,
            0x03:   0b00000011,
            0x04:   0b00000011,
            0x05:   0b00000010,
            0x06:   0b00001110,
            0x0A:   [0xE7, 0xE7, 0xE7, 0xE7, 0xE7],
            0x0B:   [0xC2, 0xC2, 0xC2, 0xC2, 0xC2],
            0x0C:   0xC3,
            0x0D:   0xC4,
            0x0E:   0xC5,
            0x0F:   0xC6,
            0x10:   [0xE7, 0xE7, 0xE7, 0xE7, 0xE7],
            0x11:   0,
            0x12:   0,
            0x13:   0,
            0x14:   0,
            0x15:   0,
            0x16:   0,
            0x1C:   0,
            0x1D:   0
        }
        for register, value in reset_values.items():
            self.write_register(register, value)

        self.flush_rx()
        self.flush_tx()
        # clear_irq_flags()?

    def flush_tx(self):
        return self.spidev.xfer2([NRF24.FLUSH_TX])[0]

    def flush_rx(self):
        return self.spidev.xfer2([NRF24.FLUSH_RX])[0]

    def clear_irq_flags(self):
        self.write_register(NRF24.STATUS, NRF24.RX_DR | NRF24.TX_DS | NRF24.MAX_RT)

    def write_register(self, register, value):
        buf = [NRF24.W_REGISTER | (NRF24.REGISTER_MASK & register)]
        buf += self._convert_to_byte_list(value)

        self.spidev.xfer2(buf)

    def read_register(self, register, length=1):
        buf = [NRF24.R_REGISTER | (NRF24.REGISTER_MASK & register)]
        buf += [NRF24.NOP] * max(1, length)

        read = self.spidev.xfer2(buf)
        if length == 1:
            return read[1]

        return read[1:]

    def set_channel(self, channel):
        if channel < 0 or channel > self.max_rf_channel:
            raise ValueError("Channel number out of range. Min:0 ,Max:{}".format(self.max_rf_channel))
        self.write_register(NRF24.RF_CH, channel)
        self.channel = channel

    def get_channel(self):
        channel = self.read_register(NRF24.RF_CH)
        if channel != self.channel:
            warnings.warn("Incorrect channel in settings - overwritten to NRF channel")
            self.channel = channel
        return channel

    def set_PA_level(self, level):
        settings = self.read_register(NRF24.RF_SETUP)
        settings &= ~(NRF24.RF_PWR_LOW | NRF24.RF_PWR_HIGH)

        if level == NRF24.PA_MAX:
            settings |= NRF24.RF_PWR_LOW | NRF24.RF_PWR_HIGH
        elif level == NRF24.PA_HIGH:
            settings |= NRF24.RF_PWR_HIGH
        elif level == NRF24.PA_LOW:
            settings |= NRF24.RF_PWR_LOW
        elif level == NRF24.PA_MIN:
            pass    # already got '00'
        else:
            raise ValueError("Wrong PA level ={}".format(level))

        self.write_register(NRF24.RF_SETUP, settings)

    def get_PA_Level(self):
        power = self.read_register(NRF24.RF_SETUP) & (NRF24.RF_PWR_LOW | NRF24.RF_PWR_HIGH)
        if power == (NRF24.RF_PWR_LOW | NRF24.RF_PWR_HIGH):
            return NRF24.PA_MAX
        elif power == NRF24.RF_PWR_HIGH:
            return NRF24.PA_HIGH
        elif power == NRF24.RF_PWR_LOW:
            return NRF24.PA_LOW
        else:
            return NRF24.PA_MIN

    def set_data_rate(self, rate):
        settings = self.read_register(NRF24.RF_SETUP)
        settings &= ~(NRF24.RF_DR_LOW | NRF24.RF_DR_HIGH)

        if rate == NRF24.BR_250kbps:
            self.data_rate = NRF24.BR_250kbps
            settings |= NRF24.RF_DR_LOW

        elif rate == NRF24.BR_1Mbps:
            self.data_rate = NRF24.BR_1Mbps
            # bits for 1 Mbps are 00 - settings already have it

        elif rate == NRF24.BR_2Mbps:
            self.data_rate = NRF24.BR_2Mbps
            settings |= NRF24.RF_DR_HIGH

        else:
            raise ValueError("Wrong Data Rate:{}".format(bin(rate)))

        self.write_register(NRF24.RF_SETUP, settings)

    def get_data_rate(self):
        settings = self.read_register(NRF24.RF_SETUP) & (NRF24.RF_DR_LOW | NRF24.RF_DR_HIGH)

        if settings == NRF24.RF_DR_LOW:     # TODO : Check for more simple way
            # [RF_DR_LOW,NRF24.RF_DR_HIGH]
            # '10' = 250 kbps
            return NRF24.BR_250kbps

        elif settings == 0x00:
            # '00' = 1 Mbps
            return NRF24.BR_1Mbps

        elif settings == NRF24.RF_DR_HIGH:
            # '01' = 2 Mbps
            return NRF24.BR_2Mbps

        else:
            # f.e '11' = Reserved
            raise RuntimeError("Wrong Data Rate:{}".format(bin(settings)))  # Warning/Handling?

    def set_retries(self, retransmit_delay, retransmit_count):
        # retransmit_delay == ARD
        # retransmit_count == ARC

        if 0 < retransmit_delay or retransmit_delay > 15:
            raise ValueError("Auto Retransmit Delay can't be: {} (Min:0, Max:15)".format(retransmit_delay))
        elif 0 < retransmit_count or retransmit_count > 15:
            raise ValueError("Auto Retransmit Count can't be: {} (Min:0, Max:15)".format(retransmit_count))

        settings = (retransmit_delay << 4) | retransmit_count
        self.write_register(NRF24.SETUP_RETR, settings)

        if self.read_register(NRF24.SETUP_RETR) == settings:
            self.delay = retransmit_delay * 0.00025
            self.retries = retransmit_count
            self.max_timeout = (self.payload_size / float(self._data_rate_bits()) + self.delay) * self.retries
            self.timeout = self.payload_size / float(self._data_rate_bits()) + self.delay
            # TODO : Check correctness of timeout calculation
        else:
            raise RuntimeError("Failed to set Retransmit Settings")

    def get_max_timeout(self):
        return self.max_timeout

    def get_timeout(self):
        return self.timeout

    def set_CRC_length(self, length):
        settings = self.read_register(NRF24.CONFIG)
        settings &= ~(NRF24.CRCO | NRF24.EN_CRC)

        if length == NRF24.CRC_DISABLED:
            pass
        elif length == NRF24.CRC_1:
            settings |= NRF24.CRC_1
        elif length == NRF24.CRC_2:
            settings |= NRF24.CRC_2
        else:
            raise ValueError("Wrong CRC Length:{}".format(length))

        self.write_register(NRF24.CONFIG, settings)
        self.crc_length = length

    def get_CRC_length(self):
        settings = self.read_register(NRF24.CONFIG)
        settings &= (NRF24.EN_CRC | NRF24.CRCO)
        length = 0

        if settings == NRF24.CRC_1:
            length = 1
        elif settings == NRF24.CRC_2:
            length = 2

        if length != self.crc_length:
            warnings.warn("Incorrect CRC length in settings - overwritten to NRF CRC")
            self.crc_length = length

        return length

    def disable_CRC(self):
        # Disable Cyclic Redundancy Check
        settings = self.read_register(NRF24.CONFIG)
        settings &= ~NRF24.EN_CRC

        self.write_register(NRF24.CONFIG, settings)
        # Delay?
        if settings != self.read_register(NRF24.CONFIG):
            raise RuntimeError("Failed to disable CRC")     # warning?
        self.crc_length = 0

    def set_payload_size(self, size):
        if size > NRF24.max_payload_size:
            warnings.warn("Payload size exceed maximum size - Payload will be set to maximum size")
        self.payload_size = min(max(size, 1), NRF24.max_payload_size)

    def get_payload_size(self):
        return self.payload_size

    def power_up(self):
        settings = self.read_register(NRF24.CONFIG)
        settings |= NRF24.PWR_UP
        self.write_register(NRF24.CONFIG, settings)
        time.sleep(self.delay)

    def power_down(self):
        # In power down state device is disabled and using minimal current consumption
        settings = self.read_register(NRF24.CONFIG)
        settings &= ~NRF24.PWR_UP
        self.write_register(NRF24.CONFIG, settings)

    def enable_dynamic_payloads(self):
        settings = self.read_register(NRF24.FEATURE)
        settings |= NRF24.EN_DPL

        self.write_register(NRF24.FEATURE, settings)
        self.write_register(NRF24.DYNPD, 0b00111111)    # enable on all pipes

        self.dynamic_payloads_enabled = True

    def get_dynamic_payload_size(self):
        return self.spidev.xfer2([NRF24.R_RX_PL_WID, NRF24.NOP])[1]

    def set_auto_ACK(self, enabled):
        if enabled:
            self.write_register(NRF24.EN_AA, 0b00111111)
            self.auto_ack = 0b00111111

            if self.crc_length == 0:
                self.set_CRC_length(NRF24.CRC_1)  # Enhanced Shockburst requires at least 1 byte CRC
        else:
            self.write_register(NRF24.EN_AA, 0)
            self.auto_ack = 0

    def set_auto_ACK_pipe(self, pipe, enabled):
        if 0 >= pipe or pipe < 6:
            settings = self.read_register(NRF24.EN_AA)
            if enabled:
                if self.crc_length == 0:
                    self.set_CRC_length(NRF24.CRC_1)
                settings |= 1 << pipe
                self.auto_ack |= 1 << pipe
            else:
                settings &= ~(1 << pipe)
                self.auto_ack &= ~(1 << pipe)

            self.write_register(NRF24.EN_AA, settings)
        else:
            warnings.warn("setAutoAckPipe failed - No such pipe: {}".format(pipe))

    def carrier_detect(self):
        return self.read_register(NRF24.RPD) & 1

    def set_CE_level(self, level, period=0):
        if self.ce_pin is not None:
            GPIO.output(self.ce_pin, level)
            if period > 0:
                time.sleep(period)
                GPIO.output(self.ce_pin, 1 - level)
        else:
            raise ValueError("CE Pin is not set")

    def wait_for_IRQ(self, timeout=30000):
        # Race conditions?
        if self.irq_pin is None:
            raise ValueError("IRQ Pin is not set")

        if GPIO.input(self.irq_pin) == 0:
            return True

        try:
            return GPIO.wait_for_edge(self.irq_pin, GPIO.FALLING, timeout) == 1
        except TypeError:  # If wrong Timeout
            return GPIO.wait_for_edge(self.irq_pin, GPIO.FALLING) == 1

    def read_payload(self, buf, buf_len=-1):
        if buf_len < 0:
            buf_len = self.payload_size

        if not self.dynamic_payloads_enabled:
            data_len = min(self.payload_size, buf_len)
            blank_len = self.payload_size - data_len
        else:
            data_len = self.get_dynamic_payload_size()
            blank_len = 0

        tx_buffer = [NRF24.R_RX_PAYLOAD] + [NRF24.NOP] * (blank_len + data_len + 1)
        payload = self.spidev.xfer2(tx_buffer)
        del buf[:]
        buf += payload[1:data_len + 1]

        self.write_register(NRF24.STATUS, NRF24.RX_DR)
        return data_len

    def read(self, buf, buf_len=-1):
        self.read_payload(buf, buf_len)
        return self.read_register(NRF24.FIFO_STATUS & NRF24.RX_EMPTY)

    def get_status(self):
        return self.spidev.xfer2([NRF24.NOP])[0]

    def write_payload(self, buf):
        buf = self._convert_to_byte_list(buf)
        if self.dynamic_payloads_enabled:
            if len(buf) > NRF24.max_payload_size:
                raise RuntimeError("Dynamic Payload size exceed Maximum size of Payload")
                # RuntimeError?
            blank_len = 0
        else:
            if len(buf) > self.payload_size:
                raise RuntimeError("Payload size exceed Size from settings")
            blank_len = self.payload_size - len(buf)

        tx_buffer = [NRF24.W_TX_PAYLOAD] + buf + ([0x00] * blank_len)
        self.spidev.xfer2(tx_buffer)

        return len(tx_buffer) - 1

    def write(self, buf):
        self.last_error = None
        length = self.write_payload(buf)
        self.set_CE_level(1)

        sent_at = monotonic()
        # Calculate Time on-air
        packet_time = (8 * (1 + self.address_width + length + self.crc_length) + 9) / (self._data_rate_bits() * 1000.)

        if self.auto_ack != 0:
            packet_time *= 2    #Check what can happen if one pipe has no auto ack enabled

        if self.retries != 0 and self.auto_ack != 0:
            timeout = sent_at + (packet_time + self.delay) * self.retries
        else:
            timeout = sent_at + packet_time + self.delay

        while monotonic() < timeout:
            time.sleep(packet_time)
            status = self.get_status()
            if status & NRF24.TX_DS:
                self.set_CE_level(0)
                return True

            if status & NRF24.MAX_RT:
                self.last_error = "Write() - MAX_RT"
                self.set_CE_level(0)
                break
        self.set_CE_level(0)

        if self.last_error is None:
            self.last_error = "Write() - Timeout"

        self.flush_tx()     # Avoid leaving payload in FIFO TX
        return False

    def enable_ACK_payload(self):
        settings = self.read_register(NRF24.FEATURE)
        settings |= NRF24.EN_ACK_PAY | NRF24.EN_DYN_ACK     # | EN_DPL?
        self.write_register(NRF24.FEATURE, settings)
        self.ack_payload_available = True
        #Check if Features can be disabled

    def open_writing_pipe(self, address):
        # Note: LSB is written first

        self.write_register(NRF24.RX_ADDR_P0, address)
        self.write_register(NRF24.TX_ADDR, address)

        if not self.dynamic_payloads_enabled:
            self.write_register(NRF24.RX_PW_P0, self.payload_size)

    def open_reading_pipe(self, pipe, address):
        if pipe < 0 or pipe > 5:
            raise RuntimeError("Invalid pipe number")
        if (pipe >= 2 and len(address) > 1) or len(address) > 5:
            raise RuntimeError("Invalid address length")

        if pipe == 0:
            self.pipe0_reading_address = address

        self.write_register(NRF24.RX_ADDR_P0 + pipe, address)

        if not self.dynamic_payloads_enabled:
            self.write_register(NRF24.RX_PW_P0 + pipe, self.payload_size)

        settings = self.read_register(NRF24.EN_RXADDR) | (1 << pipe)
        self.write_register(NRF24.EN_RXADDR, settings)

    def close_reading_pipe(self, pipe):
        settings = self.read_register(NRF24.EN_RXADDR)
        settings &= ~(1 << pipe)
        self.write_register(NRF24.EN_RXADDR, settings)

    def rx_mode(self):
        config = self.read_register(NRF24.CONFIG)
        status = self.read_register(NRF24.STATUS)

        config |= NRF24.PWR_UP | NRF24.PRIM_RX
        status |= NRF24.RX_DR | NRF24.TX_DS | NRF24.MAX_RT

        self.write_register(NRF24.CONFIG, config)
        self.write_register(NRF24.STATUS, status)

        self.flush_tx()
        self.flush_rx()
        self.clear_irq_flags()

        if self.pipe0_reading_address:
            self.write_register(self.RX_ADDR_P0, self.pipe0_reading_address)

        self.set_CE_level(1)

    def tx_mode(self):
        self.set_CE_level(0)

        self.flush_tx()
        self.flush_rx()
        self.clear_irq_flags()

        config = self.read_register(NRF24.CONFIG)
        rx_addr = self.read_register(NRF24.EN_RXADDR)

        config = (config | NRF24.PWR_UP) & ~NRF24.PRIM_RX   # enable TX
        rx_addr |= 1    # enable pipe 0

        self.write_register(NRF24.CONFIG, config)
        self.write_register(NRF24.EN_RXADDR, rx_addr)

    def set_address_width(self, width):
        possible_width = {
            3:  NRF24.AW_3,
            4:  NRF24.AW_4,
            5:  NRF24.AW_5
        }
        settings = possible_width.get(width)
        if settings is not None:
            self.write_register(NRF24.SETUP_AW, settings)
            self.address_width = width
        else:
            warnings.warn("Address width could not be set - check arguments", RuntimeWarning)

    def get_address_width(self, auto_fix=True):
        settings = self.read_register(NRF24.SETUP_AW)
        possible_width = {
            NRF24.AW_3: 3,
            NRF24.AW_4: 4,
            NRF24.AW_5: 5,
        }
        width = possible_width.get(settings)

        if width is not None:
            if self.address_width != width and auto_fix:
                warnings.warn("Incorrect address width in settings - overwritten to NRF address width", RuntimeWarning)
                self.address_width = width
            elif self.address_width != width:
                warnings.warn("Address width in settings not match width set on NRF" +
                              "\n Use set_address_width()to avoid problems", RuntimeWarning)
            return width

        elif settings == NRF24.AW_ERROR:
            warnings.warn("Illegal value set of address width is set on NRF" +
                          "\n Use set_address_width() to avoid problems", RuntimeWarning)
            return 0

        else:
            raise RuntimeError("Incorrect settings on NRF detected on register: SETUP_AW")


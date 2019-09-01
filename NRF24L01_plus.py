try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
except ImportError:
    raise ImportError("RPi.GPIO module not found")

try:
    import spidev
except ImportError:
    raise ImportError("spidev module not found - Check if installed")

try:
    from time import monotonic
except ImportError:
    from time import time as monotonic

import time
import warnings



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
    BR_250kbps = 0b10

    # Power Amplifier - RF output power , DC current Consumption
    PA_MAX =    0b11   # 0dBm , 11.3 mA
    PA_HIGH =   0b10   # -6dBm , 9 mA
    PA_LOW =    0b01   # -12dBM , 7.5 mA
    PA_MIN =    0b00   # -18dBM , 7 mA

    # -----------------------------------------------------------------------
    # STATUS
    RX_DR =                 0b01000000
    TX_DS =                 0b00100000
    MAX_RT =                0b00010000
    TX_FULL =               0b00000001

    # -----------------------------------------------------------------------
    # FIFO STATUS
    FIFO_TX_FULL =          0b00100000  # TX FIFO full flag
    TX_EMPTY =              0b00010000  # TX FIFO empty flag
    RX_FULL  =              0b00000010  # RX FIFO full flag
    RX_EMPTY =              0b00000001  # RX FIFO empty flag
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
    def _convert_to_byte_list(data):
        # Converts data to list of bytes
        if isinstance(data, int):
            data = [data]
        elif isinstance(data, str):
            data = [ord(var) for var in data]
        elif isinstance(data, float):
            data = [int(x) for x in divmod(data*100, 100)]
            warnings.warn("Float will be converted be converted into 2 int numbers with precision of 0.01")
        else:
            data = [int(var) for var in data]

        return data

    def _data_rate_bits(self):
        # Interpretation of Data Rate bits in kbps
        bits = {
            NRF24.BR_1Mbps:     1000,
            NRF24.BR_2Mbps:     2000,
            NRF24.BR_250kbps:   250,
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
        self.delay = 0.00025    # Change it only via set_retries()
        self.crc_length = 0
        self.address_width = 5

        self.pipe0_reading_address = None

        self.ack_payload_available = False
        # self.ack_payload_length = 5
        self.auto_ack = 0

        self.dynamic_payloads_enabled = False

        self.last_error = None

        if all(param is not None for param in [spi_bus, spi_device, ce_pin, irq_pin]):  # IRQ PIN could be unnecessary
            self.begin(spi_bus, spi_device, ce_pin, irq_pin)

    def __del__(self):
        # At end of life - Reset settings on NRF , end communication and clear GPIO PINS
        if spidev is not None:
            self.reset()
        self.end()
        GPIO.cleanup()

    def spi_init(self, spi_bus, spi_device):
        # Initialize SPI Connection
        self.spidev = spidev.SpiDev()
        self.spidev.open(spi_bus, spi_device)

        self.spidev.bits_per_word = 8
        self.spidev.max_speed_hz = 10000000     # Max supported by NRF24L01+ -> 10 MHz

        self.spidev.cshigh = False      # Is CS active high = No
        self.spidev.mode = 0            # Clock polarity and clock phase = 0
        self.spidev.loop = False        # Echo back = No
        self.spidev.lsbfirst = False    # LSB transferred last
        self.spidev.threewire = False   # common line for MOSI and MISO = No

    def begin(self, spi_bus, spi_device, ce_pin, irq_pin):
        # Setup Configuration
        self.spi_init(spi_bus, spi_device)

        # Initialize GPIO PINOUT
        self.ce_pin = ce_pin
        self.irq_pin = irq_pin

        if self.ce_pin is not None:
            GPIO.setup(self.ce_pin, GPIO.OUT)

        if self.irq_pin is not None:
            GPIO.setup(self.irq_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        time.sleep(0.001)
        self.reset()    # Reset Configuration

        # Default Settings on NRF
        self.set_retries(5, self.retries)   # time_delay = 5*0.000250 , retries = self.retries
        self.set_PA_level(NRF24.PA_MAX)
        self.set_data_rate(self.data_rate)
        self.set_CRC_length(self.crc_length)

        self.write_register(NRF24.DYNPD, 0)     # Dynamic payloads disabled

        self.set_channel(self.channel)

        self.clear_irq_flags()

    def end(self):
        # Ends SPI Communication
        if self.spidev:
            self.spidev.close()
            self.spidev = None

    def reset(self):
        # Reset settings on NRF's registers
        reset_values = {
            NRF24.CONFIG:       0b00001000,
            NRF24.EN_AA:        0b00111111,
            NRF24.EN_RXADDR:    0b00000011,
            NRF24.SETUP_AW:     0b00000011,
            NRF24.SETUP_RETR:   0b00000011,
            NRF24.RF_CH:        0b00000010,
            NRF24.RF_SETUP:     0b00001110,
            NRF24.RX_ADDR_P0:   [0xE7, 0xE7, 0xE7, 0xE7, 0xE7],
            NRF24.RX_ADDR_P1:   [0xC2, 0xC2, 0xC2, 0xC2, 0xC2],
            NRF24.RX_ADDR_P2:   0xC3,
            NRF24.RX_ADDR_P3:   0xC4,
            NRF24.RX_ADDR_P4:   0xC5,
            NRF24.RX_ADDR_P5:   0xC6,
            NRF24.TX_ADDR:      [0xE7, 0xE7, 0xE7, 0xE7, 0xE7],
            NRF24.RX_PW_P0:     0,
            NRF24.RX_PW_P1:     0,
            NRF24.RX_PW_P2:     0,
            NRF24.RX_PW_P3:     0,
            NRF24.RX_PW_P4:     0,
            NRF24.RX_PW_P5:     0,
            NRF24.DYNPD:        0,
            NRF24.FEATURE:      0
        }
        for register, value in reset_values.items():
            self.write_register(register, value)

        self.flush_rx()
        self.flush_tx()

    def flush_tx(self):
        return self.spidev.xfer2([NRF24.FLUSH_TX])[0]

    def flush_rx(self):
        return self.spidev.xfer2([NRF24.FLUSH_RX])[0]

    def clear_irq_flags(self):
        self.write_register(NRF24.STATUS, NRF24.RX_DR | NRF24.TX_DS | NRF24.MAX_RT)

    def clear_tx_flags(self):
        self.write_register(NRF24.STATUS, NRF24.TX_DS | NRF24.MAX_RT)

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
            # bits for 1 Mbps are 00 - settings already have that

        elif rate == NRF24.BR_2Mbps:
            self.data_rate = NRF24.BR_2Mbps
            settings |= NRF24.RF_DR_HIGH

        else:
            raise ValueError("Wrong Data Rate:{}".format(bin(rate)))

        self.write_register(NRF24.RF_SETUP, settings)

    def get_data_rate(self, auto_fix=True):
        settings = self.read_register(NRF24.RF_SETUP) & (NRF24.RF_DR_LOW | NRF24.RF_DR_HIGH)

        if settings == NRF24.RF_DR_LOW:
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
            if auto_fix:
                warnings.warn("Incorrect Data Rate on NRF - Will be set to 1 Mbps to avoid invalid functioning",
                              RuntimeWarning)
                self.set_data_rate(NRF24.BR_1Mbps)
            else:
                raise RuntimeError("Wrong Data Rate:{}".format(bin(settings)))

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
        else:
            raise RuntimeError("Failed to set Retransmit Settings")

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
            raise RuntimeError("Failed to disable CRC")

        self.crc_length = 0

    def set_payload_size(self, size):
        if size > NRF24.max_payload_size:
            warnings.warn("Payload size exceed maximum size - Payload will be set to maximum size")

        self.payload_size = min(max(size, 1), NRF24.max_payload_size)

    def get_payload_size(self):
        return self.payload_size

    def power_up(self):
        # When PWR_UP is set - NRF is able to work in RX and TX Modes
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
            self.write_register(NRF24.EN_AA, 0b00111111)    # Enable Auto Ack on all pipes
            self.auto_ack = 0b00111111

            if self.crc_length == 0:
                warnings.warn("(AutoAcknowledge) Enhanced Shockburst requires at least 1 byte CRC" +
                              "\n CRC Length will be set to 1 byte", RuntimeWarning)
                self.set_CRC_length(NRF24.CRC_1)

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
        return self.read_register(NRF24.RPD) & 1    # RPD is latched when a valid packet is received

    def set_CE_level(self, level, period=0):
        if self.ce_pin is not None:
            GPIO.output(self.ce_pin, level)

            if period > 0:
                time.sleep(period)
                GPIO.output(self.ce_pin, 1 - level)

        else:
            raise ValueError("CE Pin is not set")

    def wait_for_IRQ(self, timeout=30000):
        if self.irq_pin is None:
            raise ValueError("IRQ Pin is not set")

        if GPIO.input(self.irq_pin) == 0:
            return True

        try:
            return GPIO.wait_for_edge(self.irq_pin, GPIO.FALLING, timeout=timeout) == 1

        except TypeError:  # Wrong Timeout
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
        return self.read_register(NRF24.FIFO_STATUS) & NRF24.RX_EMPTY   # Return True if there's nothing left to read

    def get_status(self):
        return self.spidev.xfer2([NRF24.NOP])[0]

    def write_payload(self, buf):
        buf = self._convert_to_byte_list(buf)

        if self.dynamic_payloads_enabled:
            if len(buf) > NRF24.max_payload_size:
                raise RuntimeError("Dynamic Payload size exceed Maximum size of Payload")
            blank_len = 0
        else:
            if len(buf) > self.payload_size:
                raise RuntimeError("Payload size exceed Size from settings")
            blank_len = self.payload_size - len(buf)

        tx_buffer = [NRF24.W_TX_PAYLOAD] + buf + ([0x00] * blank_len)
        self.spidev.xfer2(tx_buffer)

        return len(tx_buffer) - 1

    def write(self, buf, max_rt_handler=True):
        length = self.write_payload(buf)

        time_on_air = (8 * (1 + self.address_width + length + self.crc_length) + 9) / (self._data_rate_bits() * 1000.)
        # ^ Time on-air = (8 bit * [preamble + address + payload + CRC] + 9 bit) / air_data_rate

        time_upload = (8 * length)/self.spidev.max_speed_hz  # Time Upload = payload_length/spi_data_rate

        # time_stdby2a = 130e-6     # Constant

        if self._data_rate_bits() == 1000:
            time_irq = 8.2e-6
        elif self._data_rate_bits() == 2000:
            time_irq = 6.0e-6
        else:
            # 250 kbps - Interpolated
            time_irq = 9.7e-6
        # Or Aproximate all time_irq?

        packet_time = time_on_air

        if self.auto_ack != 0:
            # Auto Acknowledge Enabled
            packet_time *= 2    # += Time on-air ACK - Same Calculation

        if self.retries != 0 and self.auto_ack != 0:    # Auto Acknowledge Enabled
            # timeout = Time Enhanced ShockBurst cycle * retries
            timeout = (time_upload + 2*130e-6 + packet_time + time_irq) * self.retries

            # or timeout = (packet_time + self.delay + 130e-6) * self.retries ?

        else:
            # Auto Acknowledge Disabled
            # timeout = time_upload + time_stdby2a + time_on_air + time_irq
            timeout = time_upload + 130e-6 + time_on_air + time_irq

        self.set_CE_level(1)
        timeout += monotonic()

        while monotonic() < timeout:
            time.sleep(packet_time)
            status = self.get_status()

            if status & NRF24.TX_DS:
                # NRF asserts TX_DS IRQ when ACK packet is received - so it can end transmitting
                self.set_CE_level(0)
                return True

            if status & NRF24.MAX_RT:
                # If MAX_RT is asserted NRF , further communication is disabled
                warnings.warn("Reached MAX_RT - Further Communication is disabled", RuntimeWarning)
                self.last_error = "Write() - MAX_RT"
                break

        self.set_CE_level(0)    # End of transmission

        if self.last_error is None:
            warnings.warn("Reached Timeout in Write", RuntimeWarning)
            self.last_error = "Write() - Timeout"

        elif self.last_error == "Write() - MAX_RT" and max_rt_handler:
            print("Clearing MAX_RT and TX_DS")
            self.clear_tx_flags()

        self.flush_tx()     # Avoid leaving payload in FIFO TX
        return False

    def enable_ACK_payload(self):
        settings = self.read_register(NRF24.FEATURE)
        settings |= NRF24.EN_ACK_PAY | NRF24.EN_DYN_ACK | NRF24.EN_DPL

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
        settings = self.read_register(NRF24.CONFIG)

        settings |= NRF24.PWR_UP | NRF24.PRIM_RX

        self.write_register(NRF24.CONFIG, settings)

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
            if self.address_width != width:
                warnings.warn("Incorrect address width in settings - overwritten to NRF address width", RuntimeWarning)
                self.address_width = width
            return width
        elif settings == NRF24.AW_ERROR:
            if auto_fix:
                warnings.warn("Illegal value set on address width is set on NRF" +
                              "\n Width will be set to default value(5)", RuntimeWarning)
                self.set_address_width(5)
            else:
                raise RuntimeError("Illegal Value of address width detected on NRF")
        else:
            raise RuntimeError("Incorrect settings on NRF detected on register: SETUP_AW")

    def print_register_byte(self, register_address):
        register = {
            NRF24.EN_AA:        "EN_AA",
            NRF24.EN_RXADDR:    "EN_RXADDR",
            NRF24.SETUP_AW:     "SETUP_AW",
            NRF24.SETUP_RETR:   "SETUP_RETR",
            NRF24.RF_CH:        "RF_CH",
            NRF24.RF_SETUP:     "RF_SETUP",
            NRF24.RPD:          "RPD",
            NRF24.DYNPD:        "DYNPD",
            NRF24.FEATURE:      "FEATURE"
        }

        name = register.get(register_address)

        if name is not None:
            print("0x{0:02x} - {1}: 0b{2:08b}".format(
                register_address,
                name,
                self.read_register(register_address)
            ))
        else:
            print("No such register address - 0x{:02x}".format(
                register_address
            ))

    def print_status(self):
        status = self.read_register(NRF24.STATUS)
        print("STATUS: 0b{:08b}".format(
            status
        ))

        rx_p_no_status = (status & 0b00001110) >> 1

        print(" RX_DR: {0:b},\n TX_DS: {1:b},\n MAX_RT: {2:b},\n RX_P_NO: 0b{3:b},\n TX_FULL: {4:b}".format(
            (status & NRF24.RX_DR)  >> 6,
            (status & NRF24.TX_DS)  >> 5,
            (status & NRF24.MAX_RT) >> 4,
            rx_p_no_status,
            status & NRF24.TX_FULL
        ))

        if rx_p_no_status == 0b111:
            print("Data pipe for the payload available for reading from RX_FIFO -> RX FIFO Empty")
        elif rx_p_no_status == 0b110:
            print("Data pipe for the payload available for reading from RX_FIFO -> Not Used")

    def print_observe_TX(self):
        status = self.read_register(NRF24.OBSERVE_TX)
        print("OBSERVE_TX: 0b{:08b}".format(
            status
        ))

        print("PLOS_CNT(Lost Packet Counter): {0} \nARC_CNT(Retransmitted packet counter): {1} ".format(
                status >> 4,
                status & 0b1111
        ))

        if (status >> 4) == 15:
            print("PLOS_CNT reached maximum value and needs reset")

    def print_fifo_status(self):
        status = self.read_register(NRF24.FIFO_STATUS)
        print("FIFO_STATUS: 0b{:08b}".format(
            status
        ))

        print(" TX_FULL: 0b{0:b} \n TX_EMPTY: 0b{1:b} \n RX_FULL: 0b{2:b} \n RX_EMPTY: 0b{3:b} \n".format(
            (status & NRF24.FIFO_TX_FULL) >> 5,
            (status & NRF24.TX_EMPTY)     >> 4,
            (status & NRF24.RX_FULL)      >> 1,
            status & NRF24.RX_EMPTY
        ))

    def print_address_register(self, register_address):
        register = {
            NRF24.RX_ADDR_P0:   "RX_ADDR_P0",
            NRF24.RX_ADDR_P1:   "RX_ADDR_P1",
            NRF24.RX_ADDR_P2:   "RX_ADDR_P2",
            NRF24.RX_ADDR_P3:   "RX_ADDR_P3",
            NRF24.RX_ADDR_P4:   "RX_ADDR_P4",
            NRF24.RX_ADDR_P5:   "RX_ADDR_P5",
            NRF24.TX_ADDR:      "TX_ADDR",
        }

        name = register.get(register_address)

        if name is not None:
            print("0x{0:02x} - {1}: 0b{2:>02x}".format(
                register_address,
                name,
                self.read_register(register_address)
            ))
        else:
            print("No such register address - 0x{:02x}".format(
                register_address
            ))

    def print_details(self):
        print("Current Settings:")

        print(" RF Channel Frequency: {} MHz".format(2400 + self.get_channel()))
        print(" Data Rate: {} kbps".format(self._data_rate_bits()))
        print(" Payload Size: {} bytes".format(self.get_payload_size()))
        print(" CRC Length: {} bytes".format(self.crc_length))

        pa_dbm = ["-18 dBm", "-12 dBm", "-6 dBm", "0 dBm"]
        print(" PA Power: {}".format(
            pa_dbm[self.get_PA_Level()]
        ))

        print("Values on Registers:")
        self.print_register_byte(NRF24.EN_AA)
        self.print_register_byte(NRF24.EN_RXADDR)
        self.print_register_byte(NRF24.SETUP_AW)
        self.print_register_byte(NRF24.SETUP_RETR)
        self.print_register_byte(NRF24.RF_CH)
        self.print_register_byte(NRF24.RF_SETUP)
        self.print_status()
        self.print_observe_TX()
        self.print_register_byte(NRF24.RPD)
        self.print_register_byte(NRF24.DYNPD)
        self.print_register_byte(NRF24.FEATURE)

        print("Address length: {} bytes".format(self.get_address_width()))

        print("\nAddresses:")
        for i in range(6):
            self.print_address_register(NRF24.RX_ADDR_P0+i)

        self.print_address_register(NRF24.TX_ADDR)

        print("\n Last Error: {}\n".format(self.last_error))


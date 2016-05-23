# -*- coding: utf-8 -*-

# Python module: ModbusServer class (ModBus/TCP Server)

import struct
from threading import Lock

# for python2 compatibility
try:
    from socketserver import BaseRequestHandler, TCPServer, ThreadingMixIn
except ImportError:
    from SocketServer import BaseRequestHandler, TCPServer, ThreadingMixIn

# some const
# modbus function code
READ_COILS = 0x01
READ_DISCRETE_INPUTS = 0x02
READ_HOLDING_REGISTERS = 0x03
READ_INPUT_REGISTERS = 0x04
WRITE_SINGLE_COIL = 0x05
WRITE_SINGLE_REGISTER = 0x06
WRITE_MULTIPLE_COILS = 0x0F
WRITE_MULTIPLE_REGISTERS = 0x10
# modbus except code
EXP_NONE = 0x00
EXP_ILLEGAL_FUNCTION = 0x01
EXP_DATA_ADDRESS = 0x02
EXP_DATA_VALUE = 0x03
EXP_SLAVE_DEVICE_FAILURE = 0x04
EXP_ACKNOWLEDGE = 0x05
EXP_SLAVE_DEVICE_BUSY = 0x06
EXP_MEMORY_PARITY_ERROR = 0x08
EXP_GATEWAY_PATH_UNAVAILABLE = 0x0A
EXP_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0B


# data class for modbus bits and words space (thread safe)
class DataBank:
    bits_lock = Lock()
    bits = [False] * 0x10000
    words_lock = Lock()
    words = [0] * 0x10000

    @classmethod
    def get_bits(cls, address, number=1):
        with cls.bits_lock:
            if (address >= 0) and (address + number <= len(cls.bits)):
                return cls.bits[address: number+address]
            else:
                return None

    @classmethod
    def set_bits(cls, address, bit_list):
        with cls.bits_lock:
            if (address >= 0) and (address + len(bit_list) <= len(cls.bits)):
                cls.bits[address: address+len(bit_list)] = bit_list
                return True
            else:
                return None

    @classmethod
    def get_words(cls, address, number=1):
        with cls.words_lock:
            if (address >= 0) and (address + number <= len(cls.words)):
                return cls.words[address: number+address]
            else:
                return None

    @classmethod
    def set_words(cls, address, word_list):
        with cls.words_lock:
            if (address >= 0) and (address + len(word_list) <= len(cls.words)):
                cls.words[address: address+len(word_list)] = word_list
                return True
            else:
                return None


class ModbusService(BaseRequestHandler):
    def handle(self):
        print('Client connected with ', self.client_address)
        while True:
            rx_head = self.request.recv(7)
            # close connection if no standard 7 bytes header
            if not (rx_head and len(rx_head) == 7):
                break
            # decode header
            (rx_hd_tr_id, rx_hd_pr_id,
             rx_hd_length, rx_hd_unit_id) = struct.unpack('>HHHB', rx_head)
            # close connection if frame header content inconsistency
            if not ((rx_hd_pr_id == 0) and (2 < rx_hd_length < 256)):
                break
            # receive body
            rx_body = self.request.recv(rx_hd_length - 1)
            # close connection if lack of bytes in frame body
            if not (rx_body and (len(rx_body) == rx_hd_length - 1)):
                break
            # body decode: function code
            rx_bd_fc = struct.unpack('B', rx_body[0:1])[0]
            # close connection if function code is inconsistency
            if rx_bd_fc > 0x7F:
                break
            # do modbus functions
            exp_status = EXP_NONE
            # functions Read Coils (0x01) or Read Discrete Inputs (0x02)
            if rx_bd_fc in (READ_COILS, READ_DISCRETE_INPUTS):
                (b_address, b_count) = struct.unpack('>HH', rx_body[1:])
                # check quantity of requested bits
                if 0x0001 <= b_count <= 0x07D0:
                    bits_l = DataBank.get_bits(b_address, b_count)
                    if bits_l:
                        # allocate bytes list
                        b_size = int(b_count/8)
                        b_size += 1 if (b_count % 8) else 0
                        bytes_l = [0] * b_size
                        # populate bytes list with data bank bits
                        for i, item in enumerate(bits_l):
                            bytes_l[int(i/8)] += (2**(i % 8)) * int(item)
                        # format body of frame with bits
                        tx_body = struct.pack('BB', rx_bd_fc, len(bytes_l))
                        # add bytes with bits
                        for byte in bytes_l:
                            tx_body += struct.pack('B', byte)
                    else:
                        exp_status = EXP_DATA_ADDRESS
                else:
                    exp_status = EXP_DATA_VALUE
            # functions Read Holding Registers (0x03) or Read Input Registers (0x04)
            elif rx_bd_fc in (READ_HOLDING_REGISTERS, READ_INPUT_REGISTERS):
                (w_address, w_count) = struct.unpack('>HH', rx_body[1:])
                # check quantity of requested words
                if 0x0001 <= w_count <= 0x007D:
                    words_l = DataBank.get_words(w_address, w_count)
                    if words_l:
                        # format body of frame with words
                        tx_body = struct.pack('BB', rx_bd_fc, w_count * 2)
                        for word in words_l:
                            tx_body += struct.pack('>H', word)
                    else:
                        exp_status = EXP_DATA_ADDRESS
                else:
                    exp_status = EXP_DATA_VALUE
            # function Write Single Coil (0x05)
            elif rx_bd_fc is WRITE_SINGLE_COIL:
                (b_address, b_value) = struct.unpack('>HH', rx_body[1:])
                f_b_value = bool(b_value == 0xFF00)
                if DataBank.set_bits(b_address, [f_b_value]):
                    # send write ok frame
                    tx_body = struct.pack('>BHH', rx_bd_fc, b_address, b_value)
                else:
                    exp_status = EXP_DATA_ADDRESS
            # function Write Single Register (0x06)
            elif rx_bd_fc is WRITE_SINGLE_REGISTER:
                (w_address, w_value) = struct.unpack('>HH', rx_body[1:])
                if DataBank.set_words(w_address, [w_value]):
                    # send write ok frame
                    tx_body = struct.pack('>BHH', rx_bd_fc, w_address, w_value)
                else:
                    exp_status = EXP_DATA_ADDRESS
            # function Write Multiple Coils (0x0F)
            elif rx_bd_fc is WRITE_MULTIPLE_COILS:
                (b_address, b_count, byte_count) = struct.unpack('>HHB', rx_body[1:6])
                # check quantity of updated coils
                if 0x0001 <= b_count <= 0x07B0:
                    # allocate bits list
                    bits_l = [False] * b_count
                    # populate bits list with bits from rx frame
                    for i, item in enumerate(bits_l):
                        byte_i = int(i/8)
                        byte_i += 1 if (i % 8) else 0
                        b_offset = byte_i + 6
                        bits_l[i] = bool(rx_body[b_offset] & 2**i % 8)
                    # write words to data bank
                    if DataBank.set_bits(b_address, bits_l):
                        # send write ok frame
                        tx_body = struct.pack('>BHH', rx_bd_fc, b_address, b_count)
                    else:
                        exp_status = EXP_DATA_ADDRESS
                else:
                    exp_status = EXP_DATA_VALUE
            # function Write Multiple Registers (0x10)
            elif rx_bd_fc is WRITE_MULTIPLE_REGISTERS:
                (w_address, w_count, byte_count) = struct.unpack('>HHB', rx_body[1:6])
                # check quantity of updated words
                if 0x0001 <= w_count <= 0x007B:
                    # allocate words list
                    words_l = [0] * w_count
                    # populate words list with words from rx frame
                    for i, item in enumerate(words_l):
                        w_offset = i * 2 + 6
                        words_l[i] = struct.unpack('>H', rx_body[w_offset:w_offset + 2])[0]
                    # write words to data bank
                    if DataBank.set_words(w_address, words_l):
                        # send write ok frame
                        tx_body = struct.pack('>BHH', rx_bd_fc, w_address, w_count)
                    else:
                        exp_status = EXP_DATA_ADDRESS
                else:
                    exp_status = EXP_DATA_VALUE
            else:
                exp_status = EXP_ILLEGAL_FUNCTION
            # check exception
            if exp_status != EXP_NONE:
                # format body of frame with exception status
                tx_body = struct.pack('BB', rx_bd_fc + 0x80, exp_status)
            # build frame header
            tx_head = struct.pack('>HHHB', rx_hd_tr_id, rx_hd_pr_id, len(tx_body) + 1, rx_hd_unit_id)
            # send frame
            self.request.send(tx_head + tx_body)
        print('Client exited')
        self.request.close()


class ThreadedTCPServer(ThreadingMixIn, TCPServer):
    pass


class ModbusServer(object):
    def __init__(self, host='localhost', port=502):
        # args
        self.host = host
        self.port = port
        # set class attribute
        ThreadedTCPServer.allow_reuse_address = True
        ThreadedTCPServer.daemon_threads = True
        # start server
        self.t = ThreadedTCPServer((self.host, self.port), ModbusService)

    def start(self):
        self.t.serve_forever()

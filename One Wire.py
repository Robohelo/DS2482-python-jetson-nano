"""
  From: https://github.com/paeaetech/paeae/blob/master/Libraries/ds2482/DS2482.cpp
  DS2482 library for Arduino
  Copyright (C) 2009-2010 Paeae Technologies

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have readd a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

	crc code is from OneWire library

	-Updates:
		* fixed wireReadByte busyWait (thanks Mike Jackson)
		* Modified search function (thanks Gary Fariss)

  Modified by Robohelo to fit the Jetson nano

"""
__repo__ = "https://github.com/Robohelo/DS2482-python-jetson-nano"

from smbus2 import SMBus
import time
from ctypes import c_uint8


class DS2482():

    def __init__(self, bus, addr):
        self.addr = 0x18 | addr
        self.bus = bus
        self.PTR_STATUS = 0xf0
        self.PTR_READ = 0xe1
        self.PTR_CONFIG = 0xc3
        self.DS2482_STATUS_BUSY = (1 << 0)
        self.DS2482_STATUS_PPD = (1 << 1)
        self.DS2482_STATUS_SD = (1 << 2)
        self.DS2482_STATUS_LL = (1 << 3)
        self.DS2482_STATUS_RST = (1 << 4)
        self.DS2482_STATUS_SBR = (1 << 5)
        self.DS2482_STATUS_TSB = (1 << 6)
        self.DS2482_STATUS_DIR = (1 << 7)
        self.mTimeout = 0
        self.searchAddress = [0, 0, 0, 0, 0, 0, 0, 0]
        self.searchLastDisrepancy = 0
        self.searchExhausted = 0

    def __write(self, message):
        if(isinstance(message, list)):
            self.bus.write_i2c_block_data(self.addr, message[0], message[1:])
        else:
            self.bus.write_byte(self.addr, message)

    def __setReadPtr(self, readPtr):
        self.__write([0xe1, readPtr])

    def __readByte(self):
        return self.bus.read_byte(self.addr)

    def __wireReadStatus(self, setPtr):
        if (setPtr):
            self.__setReadPtr(self.PTR_STATUS)

        return self.__readByte()

    def __busyWait(self, setReadPtr=False):
        status = 0
        loopCount = 1000
        status = self.__wireReadStatus(setReadPtr)
        while((status) & self.DS2482_STATUS_BUSY):
            status = self.__wireReadStatus(setReadPtr)
            loopCount -= 1
            if (loopCount <= 0):

                self.mTimeout = 1
                break

        time.sleep(0.02)

        return status

    # //----------interface
    def __reset(self):

        mTimeout = 0
        self.__write(0xf0)

    def __configure(self, config):

        self.__busyWait(True)
        self.__write([0xd2, config | (~config) << 4])
        return self.__readByte() == config

    def wireReset(self):

        self.__busyWait(True)
        self.__write(0xb4)

        status = self.__busyWait()

        return (status & 0x02) == 1

    def wireWriteByte(self, b):
        self.__busyWait(True)
        self.__write([0xa5, b])

    def wireReadByte(self):
        self.__busyWait(True)
        self.__write(0x96)
        self.__busyWait()
        self.__setReadPtr(self.PTR_READ)
        return self.__readByte()

    def wireWriteBit(self, bit):
        self.__busyWait(True)
        self.__write([0x87, (1 if bit != 0 else 0) << 7])

    def wireReadBit(self):
        self.wireWriteBit(1)
        status = self.__busyWait(True)
        return 1 if status & self.DS2482_STATUS_SBR != 0 else 0

    def wireSkip(self):
        self.wireWriteByte(0xcc)

    def wireSelect(self, rom):
        self.wireWriteByte(0x55)
        for x in range(8):
            rom[x] = self.wireWriteByte(rom)

    # if ONEWIRE_SEARCH
    def wireResetSearch(self):
        self.searchExhausted = 0
        self.searchLastDisrepancy = 0
        for x in range(8):
            self.searchAddress[x] = 0

    def wireSearch(self, newAddr):

        i = 0
        direction = 0
        last_zero = 0

        if (self.searchExhausted):
            return 0, None

        if (not self.wireReset()):
            return 0, None

        self.__busyWait(True)
        self.wireWriteByte(0xf0)

        for i in range(1, 65, 1):

            romByte = (i-1) >> 3
            romBit = 1 << ((i-1) & 7)

            if (i < self.searchLastDisrepancy):
                direction = self.searchAddress[romByte] & romBit
            else:
                direction = i == self.searchLastDisrepancy

            self.__busyWait()
            self.__write([0x78, (1 if direction != 0 else 0) << 7])
            status = self.__busyWait()

            identity = status & self.DS2482_STATUS_SBR
            comp_id = status & self.DS2482_STATUS_TSB
            direction = status & self.DS2482_STATUS_DIR

            if (identity and comp_id):
                return 0, None
            else:
                if (not identity and not comp_id and not direction):
                    last_zero = i

            if (direction):
                self.searchAddress[romByte] |= romBit
            else:
                self.searchAddress[romByte] &= c_uint8(~romBit)

        self.searchLastDisrepancy = last_zero

        if (last_zero == 0):
            self.searchExhausted = 1

        for i in range(8):
            newAddr[i] = self.searchAddress[i]

        return 1, newAddr

#Only if you want to use it in the busio.py
class OneWire(DS2482):
    """
    Stub class for OneWire, which is currently not implemented
    """

    def __init__(self, bus):
        # Initalisieren über Eltern-Klasse
        super().__init__(bus, addr = 0)

    def deinit(self):
        """
        Deinitialize the OneWire bus and release any hardware resources for reuse.
        """
        raise NotImplementedError("OneWire deinit has not been implemented yet")

    def reset(self):
        """
        Reset the OneWire bus and read presence
        """
        return super().wireReset()

    def read_bit(self):
        """
        Read in a bit
        """
        return super().wireReadBit()

    def write_bit(self, value):
        """
        Write out a bit based on value.
        """
        return super().wireWriteBit()

    def read_byte(self):
        """
        Read in a byte
        """
        return super().wireReadByte()

    def write_byte(self, value):
        """
        Write out a byte based on value.
        """
        return super().wireWriteByte(value)

    def search(self, newAddr):
        """
        Search ROM
        """
        return super().wireSearch(newAddr)

    def reset_search(self):
        """
        Resets ROM search
        """
        return super().wireResetSearch()

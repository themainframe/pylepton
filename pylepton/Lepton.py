#!/usr/bin/env python

import numpy as np
import ctypes
import struct
import time

from ioctl_numbers import _IOR, _IOW
from fcntl import ioctl

SPI_IOC_MAGIC = ord("k")

SPI_IOC_RD_MODE = _IOR(SPI_IOC_MAGIC, 1, "=B")
SPI_IOC_WR_MODE = _IOW(SPI_IOC_MAGIC, 1, "=B")

SPI_IOC_RD_LSB_FIRST = _IOR(SPI_IOC_MAGIC, 2, "=B")
SPI_IOC_WR_LSB_FIRST = _IOW(SPI_IOC_MAGIC, 2, "=B")

SPI_IOC_RD_BITS_PER_WORD = _IOR(SPI_IOC_MAGIC, 3, "=B")
SPI_IOC_WR_BITS_PER_WORD = _IOW(SPI_IOC_MAGIC, 3, "=B")

SPI_IOC_RD_MAX_SPEED_HZ = _IOR(SPI_IOC_MAGIC, 4, "=I")
SPI_IOC_WR_MAX_SPEED_HZ = _IOW(SPI_IOC_MAGIC, 4, "=I")

SPI_CPHA = 0x01  # /* clock phase */
SPI_CPOL = 0x02  # /* clock polarity */
SPI_MODE_0 = (0 | 0)  # /* (original MicroWire) */
SPI_MODE_1 = (0 | SPI_CPHA)
SPI_MODE_2 = (SPI_CPOL | 0)
SPI_MODE_3 = (SPI_CPOL | SPI_CPHA)


class Lepton(object):
    """Communication class for FLIR Lepton module on SPI

    Args:
      spi_dev (str): Location of SPI device node. Default '/dev/spidev0.0'.
    """

    # Pixel dimensions of a single frame
    ROWS = 60
    COLS = 80

    # A *packet* contains one word per column, plus two words for the ID and CRC
    VOSPI_FRAME_SIZE = COLS + 2

    # Converting this to a byte count
    VOSPI_FRAME_SIZE_BYTES = VOSPI_FRAME_SIZE * 2

    # Transferring in SPI mode 3
    MODE = SPI_MODE_3
    BITS = 8
    SPEED = 18000000

    # We can fit a maximum of 24 packets in 4096 bytes (the spidev default message limit)
    SPIDEV_MESSAGE_LIMIT = 24

    def __init__(self, spi_dev="/dev/spidev0.0"):
        self._spi_dev = spi_dev

        # Create a buffer to contain a full packet worth of words (uint16)
        self._txbuf = np.zeros(Lepton.VOSPI_FRAME_SIZE, dtype=np.uint16)

        # Build a structure to describe a single SPI transfer
        # struct spi_ioc_transfer {
        #   __u64     tx_buf;
        #   __u64     rx_buf;
        #   __u32     len;
        #   __u32     speed_hz;
        #   __u16     delay_usecs;
        #   __u8      bits_per_word;
        #   __u8      cs_change;
        #   __u32     pad;
        # };
        self._xmit_struct = struct.Struct("=QQIIHBBI")

        # The size of a single SPI transfer
        self._msg_size = self._xmit_struct.size

        # Create a buffer big enough to contain one of these messages for every row
        self._xmit_buf = np.zeros((self._msg_size * Lepton.ROWS), dtype=np.uint8)

        # Create a buffer big enough to contain the captured data (1 uint16 per pixel, including the IDs and CRCs)
        self._capture_buf = np.zeros((Lepton.ROWS, Lepton.VOSPI_FRAME_SIZE, 1), dtype=np.uint16)

        # For each row, pack the SPI transfer into the buffer at the next offset position
        for i in range(Lepton.ROWS):
            self._xmit_struct.pack_into(
                                        # The whole buffer that will contain all the SPI transfer messages when we're done
                                        self._xmit_buf,

                                        # The offset to start writing packed data at
                                        i * self._msg_size,

                                        # __u64     tx_buf;
                                        self._txbuf.ctypes.data,

                                        # The target memory for the transfer (rx buf) is the relevant offset in capture_buf
                                        # __u64     rx_buf;
                                        self._capture_buf.ctypes.data + Lepton.VOSPI_FRAME_SIZE_BYTES * i,

                                        # __u32     len;
                                        Lepton.VOSPI_FRAME_SIZE_BYTES,

                                        # __u32     speed_hz;
                                        Lepton.SPEED,

                                        # __u16     delay_usecs;
                                        0,

                                        # __u8      bits_per_word;
                                        Lepton.BITS,

                                        # __u8      cs_change;
                                        1,

                                        # __u32     pad;
                                        0
            )

    def __enter__(self):
        # Open the file handle
        self._handle = open(self._spi_dev, "w+")

        # Set up the SPI bus, not sure there's a need for the RD type calls here as they're reading not writing modes
        ioctl(self._handle, SPI_IOC_RD_MODE, struct.pack("=B", Lepton.MODE))
        ioctl(self._handle, SPI_IOC_WR_MODE, struct.pack("=B", Lepton.MODE))
        ioctl(self._handle, SPI_IOC_RD_BITS_PER_WORD, struct.pack("=B", Lepton.BITS))
        ioctl(self._handle, SPI_IOC_WR_BITS_PER_WORD, struct.pack("=B", Lepton.BITS))
        ioctl(self._handle, SPI_IOC_RD_MAX_SPEED_HZ, struct.pack("=I", Lepton.SPEED))
        ioctl(self._handle, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack("=I", Lepton.SPEED))
        return self

    def __exit__(self, type, value, tb):
        # Close the file handle
        self._handle.close()

    @staticmethod
    def capture_segment(handle, xs_buf, xs_size, capture_buf):

        # How many packets we've got left to capture (60)
        messages = Lepton.ROWS

        # Create a SPI transfer descriptor
        iow = _IOW(SPI_IOC_MAGIC, 0, xs_size)

        # Perform the SPI transfer
        ioctl(handle, iow, xs_buf, True)

        # While we're reading discard packets (I.e. begin 0xff)
        while (capture_buf[0] & 0xf) == 0xf:
            # Keep reading
            ioctl(handle, iow, xs_buf, True)

        # We did get a valid one to end the above loop
        messages -= 1

        # While there are messages left to read
        while messages > 0:
            # If there are more than we can read in one go
            if messages > Lepton.SPIDEV_MESSAGE_LIMIT:
                # Read the maximum number possible at once
                count = Lepton.SPIDEV_MESSAGE_LIMIT
            else:
                # Otherwise read them all
                count = messages

            # Create a SPI transfer descriptor message speccing the size to fit all the received bytes
            iow = _IOW(SPI_IOC_MAGIC, 0, xs_size * count)

            # Perform the action
            ret = ioctl(handle, iow, xs_buf[xs_size * (60 - messages):], True)

            # Handle when the ioctl call failed
            if ret < 1:
                raise IOError("can't send {0} spi messages ({1})".format(60, ret))

            # Knock off the messages we just completed
            messages -= count

    def capture(self, data_buffer=None, log_time=False, debug_print=False, retry_reset=True):
        """Capture a frame of data.

        Captures 80x60 uint16 array of non-normalized (raw 12-bit) data. Returns that frame and a frame_id (which
        is currently just the sum of all pixels). The Lepton will return multiple, identical frames at a rate of up
        to ~27 Hz, with unique frames at only ~9 Hz, so the frame_id can help you from doing additional work
        processing duplicate frames.

        Args:
          data_buffer (numpy.ndarray): Optional. If specified, should be ``(60,80,1)`` with `dtype`=``numpy.uint16``.

        Returns:
          tuple consisting of (data_buffer, frame_id)
        """

        # Start timer
        start = time.time()

        # Prepare a suitable data buffer to receive the pixel words (uint16s) into
        if data_buffer is None:
            data_buffer = np.ndarray((Lepton.ROWS, Lepton.COLS, 1), dtype=np.uint16)
        elif data_buffer.ndim < 2 or data_buffer.shape[0] < Lepton.ROWS or data_buffer.shape[1] < Lepton.COLS or data_buffer.itemsize < 2:
            raise Exception("Provided input array not large enough")

        # While we've not captured a good segment
        while True:
            Lepton.capture_segment(self._handle, self._xmit_buf, self._msg_size, self._capture_buf[0])

            # Verify that packet 20 within this segment does indeed contain packet ID 20
            if retry_reset and (self._capture_buf[20, 0] & 0xFF0F) != 0x1400:
                # Wait 0.185ms (according to spec) before trying to read another segment
                # during this time the CS will be deasserted
                if debug_print:
                    print "Garbage frame number reset waiting..."
                time.sleep(0.185)
            else:
                break

        # Byteswap the array
        self._capture_buf.byteswap(True)

        # Cut off the first two words from each packet (the Packet ID and CRC)
        data_buffer[:, :] = self._capture_buf[:, 2:]

        # Stop the clock
        end = time.time()

        # Perform debug printouts
        if debug_print:
            print "---"
            for i in range(Lepton.ROWS):
                fid = self._capture_buf[i, 0, 0]
                crc = self._capture_buf[i, 1, 0]
                fnum = fid & 0xFFF
                print "0x{0:04x} 0x{1:04x} : Row {2:2} : crc={1}".format(fid, crc, fnum)
            print "---"

        if log_time:
            print "frame processed int {0}s, {1}hz".format(end - start, 1.0 / (end - start))

        # TODO: turn on telemetry to get real frame id, sum on this array is fast enough though (< 500us)
        return data_buffer, data_buffer.sum()

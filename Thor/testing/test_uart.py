import unittest
from pystlink import PyStlink
# import some definitions that explain what different kind of commands are available for the user


# Note: If PyStlink can't find the usb it's probably due to a driver issue. See: https://goo.gl/tDXv54

class TestUART(unittest.TestCase):
    @classmethod
    def setupClass(cls):
        """
        Check for device existence, can connect, gives ack, etc.
        :return:
        """
        cls.stlink = PyStlink()
        cls.stlink.start()


    @classmethod
    def tearDownClass(cls):
        """
        Clean up operations
        :return:
        """
        pass

    def setUpModule(self):
        """
        Put the MCU into a known state before running next test
        :return:
        """
        pass

    def tearDownModule(self):
        """
        Put the MCU into a known state before running next test
        :return:
        """
        pass

    def test_BlockingTX(self):
        pass

    def test_BlockingRX(self):
        pass

    def test_InterruptTX(self):
        pass

    def test_InterruptRX(self):
        pass

    def test_DmaTX(self):
        pass

    def test_DmaRX(self):
        pass

    def set_mode(self, peripheral, mode):
        pass

    def set_baudrate(self, baud):
        pass

    def detect_usb_to_serial_converter(self):
        pass

    def detect_stlink(self):
        """
        Could check for some stupid fancy stuff here.
        :return:
        """
        pass


if __name__ == '__main__':
    unittest.main()

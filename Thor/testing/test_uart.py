import unittest

# import some definitions that explain what different kind of commands are available for the user


class TestUART(unittest.TestCase):
    @classmethod
    def setupClass(cls):
        """
        Check for device existence, can connect, gives ack, etc.
        :return:
        """
        pass

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
        raise NotImplementedError

    def test_BlockingRX(self):
        raise NotImplementedError

    def test_InterruptTX(self):
        raise NotImplementedError

    def test_InterruptRX(self):
        raise NotImplementedError

    def test_DmaTX(self):
        raise NotImplementedError

    def test_DmaRX(self):
        raise NotImplementedError

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

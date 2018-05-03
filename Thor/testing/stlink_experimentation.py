from pystlink import PyStlink
from lib.stlinkusb import StlinkUsbConnector



if __name__ == '__main__':
    #stlink = StlinkUsbConnector()

    test = PyStlink()
    test.start()

    test.cmd(['flash', 'erase', 'verify', 'ChimeraDevelopment.bin'])
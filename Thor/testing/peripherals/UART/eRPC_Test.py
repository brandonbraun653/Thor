import time
import erpc

from peripherals.UART.erpc.py.uart_rpc import client

CRC_VALUE = 33796


def test():
    transport = erpc.transport.SerialTransport('COM5', 115200)
    transport.crc_16 = CRC_VALUE

    clientManager = erpc.client.ClientManager(transport, erpc.basic_codec.BasicCodec)

    print("Trying to open client\n")
    myClient = client.IOClient(clientManager)

    print("Turning Green LED on?")
    myClient.turnGreenLEDON()

    time.sleep(3)

    print("Turning Green LED off?")
    myClient.turnGreenLEDOFF()


if __name__ == "__main__":
    test()

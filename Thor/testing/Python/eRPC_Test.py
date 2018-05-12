import time
import erpc
import Python.led as erpc_test

CRC_VALUE = 49684


def client():
    transport = erpc.transport.SerialTransport('COM5', 115200)
    transport.crc_16 = CRC_VALUE

    clientManager = erpc.client.ClientManager(transport, erpc.basic_codec.BasicCodec)

    print("Trying to open client\n")
    myClient = erpc_test.client.IOClient(clientManager)

    print("Turning Green LED on?")
    myClient.turnGreenLEDON()

    time.sleep(1)

    print("Turning Green LED off?")
    myClient.turnGreenLEDOFF()


if __name__ == "__main__":
    client()

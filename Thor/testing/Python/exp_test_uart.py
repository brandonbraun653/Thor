import time
import serial

ser = serial.Serial(
    port='COM5',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=2.0
)


if __name__ == "__main__":
    while 1:
        data = ser.readline()
        print(data)

        if data == b'Waiting for 5 characters...\r\n':
            print("\tSending hello")
            ser.write(b'hello')
            print(ser.readline().decode('utf-8'))

        time.sleep(0.25)

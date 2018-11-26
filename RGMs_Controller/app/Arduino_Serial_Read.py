import serial
import time


def main():
    S = serial.Serial()
    S.port = "COM7"
    S.baudrate = 9600
    S.bytesize = 8
    S.stopbits = 1
    S.timeout = 0.1  # timeout

    try:
        S.open()
        time.sleep(1)
    except:
        S.close()
        S.open()
        time.sleep(1)
    data = []
    index = 0
    while True:
        S.write(bytes(2))
        data_origin = S.read(2)
        # print(int.from_bytes(data_origin, byteorder='big'))
        temp = int.from_bytes(data_origin, byteorder='big')
        # print(temp)
        data.append(temp)
        if index > 50000:
            break
        index += 1
        if index % 100 == 0:
            print(index)
    filename = time.strftime('Current_Result/' + '%Y-%m-%d %H-%M-%S.txt', time.localtime(time.time()))
    fileObject = open(filename, 'w')
    for ip in data:
        fileObject.write(str(ip))
        fileObject.write('\n')
    fileObject.close()


if __name__ == '__main__':
    main()

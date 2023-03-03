import serial
import time

DELAY_TIME = 0.1                #Время задержки при оправке и чтении
CONNECTION_DELAY_TIME = 2       #Время задержки при подключении (должно быть больше тк это дело долгое)

"""
Класс управление ардуино
При создании задать Порт, Скорость, timeout
Перед использованием активировать методо run!!!!!!!!!!
Поддерживает отправку и чтение строк а также сообщение об ошибках
"""
class ArduinoControl:
    def __init__(self, port, speed, timeout):
        self.port = port
        self.baudrate = speed
        self.timeout = timeout
        self.arduino = None
    #Инициализация платы
    def run(self):
        try:
            self.arduino = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            print(f'Arduino plate on port {self.port} was connected successfully')
        except:
            print(f'CONNECTION ERROR!!!')
        time.sleep(CONNECTION_DELAY_TIME)
    def stop(self):
        try:
            self.arduino.close()
            print(f'Arduino plate on port {self.port} was disconnected successfully')
        except:
            print(f'DISCONNECTION ERROR!!!')
        time.sleep(CONNECTION_DELAY_TIME)
    #Чтение строки из порта
    def readLine(self):
        data = None
        try:
            data = self.arduino.readline().decode().strip()
        except:
            print("FATAL ERROR!!! Could not read data from device")
        time.sleep(DELAY_TIME)
        print(data)
        return data
    #Отправка строки
    def write(self, data):
        try:
            self.arduino.write(data.encode())
        except:
            print("FATAL ERROR!!! Could not send data to device")
        time.sleep(DELAY_TIME)
from Hard import motorSimulator
"""
Класс двигателя
При инициализакии передать тип двигателя => L или R для правильного опрелелния при обновлении
"""
maxVoltage = 12     #Максимальное возможное напряжения на двигателе
class Motor:
    # Инициализация
    def __init__(self, type, simulation_dt):
        self.motorType = type
        self.theta = 0  # угол поворота двигателя
        self.speed = 0  # скорость двигателя
        self.Simulation = motorSimulator(speed=0, angle=0, dt = simulation_dt)

    # Подача напряжения на двигатель
    def runMotor(self, voltage):
        if voltage > maxVoltage: voltage = maxVoltage
        if voltage < -maxVoltage: voltage = -maxVoltage
        self.Simulation.setMotorPower(voltage)
        #print(f'Motor {self.motorType} gets power {voltage} volts')

    # Обновление информации
    def update(self):
        self.theta, self.speed = self.Simulation.getMotorState()
        #print(f'Information about motor {self.motorType} was updated!')

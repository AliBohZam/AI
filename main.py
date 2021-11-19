from machine import I2C, Pin, PWM, Timer
import utime

class LCD:
    def __init__(self, scl, sda):
        self.i2c = I2C(0, scl=Pin(scl), sda=Pin(sda))

    class Color():
        Red = 0
        Green = 1
        Blue = 2
        Yellow = 3
        Cyan = 4
        Magenta = 5

    def setBackLight(self, color):
        color = color % 6
        self.i2c.writeto_mem(0x62, 0, bytes('\x00', 'utf8'))
        self.i2c.writeto_mem(0x62, 1, bytes('\x00', 'utf8'))
        self.i2c.writeto_mem(0x62, 8, bytes('\xaa', 'utf8'))
        if color == self.Color.Red:
            self.i2c.writeto_mem(0x62, 4, bytes('\xff', 'utf8'))
            self.i2c.writeto_mem(0x62, 3, bytes('\x00', 'utf8'))
            self.i2c.writeto_mem(0x62, 2, bytes('\x00', 'utf8'))
        elif color == self.Color.Green:
            self.i2c.writeto_mem(0x62, 4, bytes('\x00', 'utf8'))
            self.i2c.writeto_mem(0x62, 3, bytes('\xff', 'utf8'))
            self.i2c.writeto_mem(0x62, 2, bytes('\x00', 'utf8'))
        elif color == self.Color.Blue:
            self.i2c.writeto_mem(0x62, 4, bytes('\x00', 'utf8'))
            self.i2c.writeto_mem(0x62, 3, bytes('\x00', 'utf8'))
            self.i2c.writeto_mem(0x62, 2, bytes('\xff', 'utf8'))
        elif color == self.Color.Yellow:
            self.i2c.writeto_mem(0x62, 4, bytes('\xff', 'utf8'))
            self.i2c.writeto_mem(0x62, 3, bytes('\xff', 'utf8'))
            self.i2c.writeto_mem(0x62, 2, bytes('\x00', 'utf8'))
        elif color == self.Color.Cyan :
            self.i2c.writeto_mem(0x62, 4, bytes('\x00', 'utf8'))
            self.i2c.writeto_mem(0x62, 3, bytes('\xff', 'utf8'))
            self.i2c.writeto_mem(0x62, 2, bytes('\xff', 'utf8'))
        elif color == self.Color.Magenta:
            self.i2c.writeto_mem(0x62, 4, bytes('\xff', 'utf8'))
            self.i2c.writeto_mem(0x62, 3, bytes('\x00', 'utf8'))
            self.i2c.writeto_mem(0x62, 2, bytes('\xff', 'utf8'))

    def textCommand(self, cmd):
        self.i2c.writeto_mem(0x3e, 0x80, cmd)

    def setText(self, text):
        self.textCommand(bytes('\x01', 'utf8')) # clear display
        utime.sleep_ms(10)
        self.textCommand(bytes('\x0c', 'utf8')) # display on, no cursor
        self.textCommand(bytes('\x28', 'utf8')) # 2 lines
        utime.sleep_ms(10)
        count = 0
        row = 0
        for char in text:
            if char == '\n' or count == 16:
                count = 0
                row += 1
                if row == 2:
                    break
                self.textCommand(bytes('\xc0', 'utf8'))
                if char == '\n':
                    continue
            count += 1
            self.i2c.writeto_mem(0x3e, 0x40, char)

    def setTextNoRefresh(self, text):
        self.textCommand(bytes('\x02', 'utf8')) # return home
        utime.sleep_ms(10)
        self.textCommand(bytes('\x0c', 'utf8')) # display on, no cursor
        self.textCommand(bytes('\x28', 'utf8')) # 2 lines
        utime.sleep_ms(10)
        count = 0
        row = 0
        for char in text:
            if char == '\n' or count == 16:
                count = 0
                row += 1
                if row == 2:
                    break
                self.textCommand(bytes('\xc0', 'utf8'))
                if char == '\n':
                    continue
            count += 1
            self.i2c.writeto_mem(0x3e, 0x40, char)

class MPU():
    def __init__(self, scl, sda):
        self.i2c = I2C(1, scl=Pin(scl), sda=Pin(sda))
        if 0x71 != self.whoami:
            raise RuntimeError("MPU6500 not found in I2C bus.")
        self._accel_so = 16384 # 1 / 16384 ie. 0.061 mg / digit
        self._gyro_so = 131 # 250 DPS
        self._accel_sf = 9.80665 # 1 g = 9.80665 m/s2 ie. standard gravity
        self._gyro_sf = 0.017453292519943 # 1 deg/s is 0.017453292519943 rad/s

    @property
    def acceleration(self):
        """
        X, Y, Z values in g
        """
        so = self._accel_so
        sf = self._accel_sf

        xyz = self._register_three_shorts(0x3b)
        return tuple([value / so * sf for value in xyz])

    @property
    def gyro(self):
        """
        X, Y, Z values in radians per second
        """
        so = self._gyro_so
        sf = self._gyro_sf

        xyz = self._register_three_shorts(0x43)
        xyz = [value / so * sf for value in xyz]

        return tuple(xyz)

    @property
    def temperature(self):
        """
        Value in celcius
        """
        temp = self._register_short(0x41)
        return ((temp - 21) / 333.87) + 21

    @property
    def whoami(self):
        return self._register_char(0x75)

    def _register_short(self, register, value=None, buf=bytearray(2)):
        if value is None:
            self.i2c.readfrom_mem_into(0x68, register, buf)
            return ((buf[0] << 8) | buf[1])

        return self.i2c.writeto_mem(0x68, register, bytes(value))

    def _register_three_shorts(self, register, buf=bytearray(6)):
        self.i2c.readfrom_mem_into(0x68, register, buf)
        return ((buf[4] << 8) | buf[5], (buf[2] << 8) | buf[3], (buf[0] << 8) | buf[1])

    def _register_char(self, register, value=None, buf=bytearray(1)):
        if value is None:
            self.i2c.readfrom_mem_into(0x68, register, buf)
            return buf[0]

        return self.i2c.writeto_mem(0x68, register, bytes(value))

class Servo:
    def __init__(self, pin, minDuty, maxDuty):
        self.pin = pin
        self.min = minDuty
        self.max = maxDuty
        self.servo = PWM(Pin(pin))
        self.servo.freq(50)
        self.servo.duty_u16((minDuty + maxDuty) >> 1)

    def setServo(self, level):
        level = level % 9
        self.servo.duty_u16(((self.max - self.min) >> 3) * level + self.min)

class Sonar:
    def __init__(self, pin):
        self.pin = pin

    def getDistance(self):
        pin = Pin(self.pin, Pin.OUT)
        pin.value(0)
        utime.sleep_us(2)
        pin.value(1)
        utime.sleep_us(5)
        pin.value(0)
        pin = Pin(self.pin, Pin.IN)
        try:
            timeout = 0
            threshold = 10000
            while timeout < threshold and pin.value() == 0:
                timeout = timeout + 1
                signalOff = utime.ticks_us()
            while timeout < threshold and pin.value() == 1:
                timeout = timeout + 1
                signalOn = utime.ticks_us()
            if timeout < threshold:
                timePassed = signalOn - signalOff
                distance = (timePassed * 0.0343) / 2
                return distance
            return -1
        except:
            return -1

def tick(timer):
    global counter

    lcd.setBackLight(counter >> 5)
    led.toggle()

    # TODO use AI to drive servos
    salt = 0xbeefbeef
    sugar = 0xdeaddead
    servo1.setServo(counter ^ salt)
    servo2.setServo(counter ^ sugar)

    counter = counter + 1

def loop():
    lastUpdate = utime.ticks_us()
    last_acc_x = 4.3
    last_acc_y = 34.8
    last_acc_z = 19.6
    last_gyr_x = 0.0
    last_gyr_y = 8.7
    last_gyr_z = 0.0
    while True:
        curTime = utime.ticks_us()
        acceleration = imu.acceleration
        gyro = imu.gyro
        diffTime = curTime - lastUpdate;
        lastUpdate = curTime;
        x = acceleration[0];
        y = acceleration[1];
        z = acceleration[2];
        acc_value = abs(x + y + z - last_acc_x - last_acc_y - last_acc_z) / diffTime * 100000;
        last_acc_x = x;
        last_acc_y = y;
        last_acc_z = z;
        x = gyro[0];
        y = gyro[1];
        z = gyro[2];
        gyr_value = abs(x + y + z - last_gyr_x - last_gyr_y - last_gyr_z) / diffTime * 100000;
        last_gyr_x = x;
        last_gyr_y = y;
        last_gyr_z = z;

        dist = sonar.getDistance()

        if (dist > 0):
            reward = acc_value * 10 + gyr_value
            lcd.setTextNoRefresh("{:5.1f}cm  {:5.2f}'C{:f}".format(dist, imu.temperature, reward))
            utime.sleep_ms(5)

counter = 0
lcd = LCD(1, 0)
led = Pin(25, Pin.OUT)
imu = MPU(27, 26)
servo1 = Servo(16, 3276, 4641)
servo2 = Servo(17, 5733, 8464)
sonar = Sonar(15)
timer = Timer()
timerFrequency = 10

if __name__ == '__main__':
    timer.init(freq=timerFrequency, mode=Timer.PERIODIC, callback=tick)
    lcd.setText("Pi Pico!")
    utime.sleep(3)
    lcd.setText("")
    loop()

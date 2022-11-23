from gpiozero import Robot, DigitalInputDevice
from time import sleep

SAMPLETIME = 1

class QuadratureEncoder(object):
    """
    A simple quadrature encoder class
    Note - this class does not determine direction
    """
    def __init__(self, pin_a, pin_b):
        self._value = 0

        encoder_a = DigitalInputDevice(pin_a)
        encoder_a.when_activated = self._increment
        encoder_a.when_deactivated = self._increment

        encoder_b = DigitalInputDevice(pin_b)
        encoder_b.when_activated = self._increment
        encoder_b.when_deactivated = self._increment
        
    def reset(self):
        self._value = 0

    def _increment(self):
        self._value += 1

    @property
    def value(self):
        return self._value

e1 = QuadratureEncoder(16,17)
e2 = QuadratureEncoder(18, 19)
r = Robot((10,9), (8,7))

m1_speed = 1.0
m2_speed = 1.0
r.value = (m1_speed, m2_speed)

while True:
    print("e1 {} e2 {}".format(e1.value, e2.value)
    sleep(SAMPLETIME)
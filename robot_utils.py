import threading
import time

from gpiozero import DigitalOutputDevice, PWMOutputDevice, DistanceSensor
from gpiozero.pins.lgpio import LGPIOFactory


PIN_FACTORY = LGPIOFactory()


class Robot:
    Sensor_ECHO_Pin = 8
    Sensor_TRIGGER_Pin = 11

    Motor_A_EN = 4
    Motor_B_EN = 17
    Motor_A_Pin1 = 26
    Motor_A_Pin2 = 21
    Motor_B_Pin1 = 27
    Motor_B_Pin2 = 18

    def __init__(self, speed=70):
        self.speed = max(0, min(100, speed)) / 100.0

        pf = PIN_FACTORY

        self.pwm_A = PWMOutputDevice(self.Motor_A_EN, frequency=1000, pin_factory=pf)
        self.pwm_B = PWMOutputDevice(self.Motor_B_EN, frequency=1000, pin_factory=pf)

        self.a_in1 = DigitalOutputDevice(self.Motor_A_Pin1, pin_factory=pf)
        self.a_in2 = DigitalOutputDevice(self.Motor_A_Pin2, pin_factory=pf)
        self.b_in1 = DigitalOutputDevice(self.Motor_B_Pin1, pin_factory=pf)
        self.b_in2 = DigitalOutputDevice(self.Motor_B_Pin2, pin_factory=pf)

        self.pwm_A.value = 0.0
        self.pwm_B.value = 0.0

        self._move_thread = None
        self._stop_flag = threading.Event()
        self._lock = threading.Lock()

        self.sensor = DistanceSensor(
            echo=self.Sensor_ECHO_Pin,
            trigger=self.Sensor_TRIGGER_Pin,
            max_distance=1.0,
            pin_factory=pf,
        )

    def get_sensor_distance(self):
        return float(self.sensor.distance)

    def _run_motor(self, motor_func):
        self._stop_flag.clear()
        motor_func()
        while not self._stop_flag.is_set():
            time.sleep(0.01)
        self.stop()

    def _start(self, motor_func):
        with self._lock:
            if self._move_thread and self._move_thread.is_alive():
                self._stop_flag.set()
                self._move_thread.join(timeout=1.0)

            self._stop_flag.clear()
            self._move_thread = threading.Thread(
                target=self._run_motor,
                args=(motor_func,),
                daemon=True,
            )
            self._move_thread.start()

    def _set_motor_outputs(self, in1, in2, status):
        if status == 1:
            in1.on()
            in2.off()
        elif status == -1:
            in1.off()
            in2.on()
        else:
            in1.off()
            in2.off()

    # Garde ton mapping de câblage actuel
    def motor_A(self, status):
        self._set_motor_outputs(self.b_in1, self.b_in2, status)
        self.pwm_B.value = self.speed if status != 0 else 0.0

    def motor_B(self, status):
        self._set_motor_outputs(self.a_in1, self.a_in2, status)
        self.pwm_A.value = self.speed if status != 0 else 0.0

    def recule(self):
        self._start(lambda: (self.motor_A(1), self.motor_B(1)))

    def avance(self):
        self._start(lambda: (self.motor_A(-1), self.motor_B(-1)))

    def gauche(self):
        self._start(lambda: (self.motor_A(-1), self.motor_B(1)))

    def droite(self):
        self._start(lambda: (self.motor_A(1), self.motor_B(-1)))

    def stop(self):
        self._stop_flag.set()
        self.motor_A(0)
        self.motor_B(0)

    def cleanup(self):
        self.stop()

        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=1.0)

        try:
            self.sensor.close()
        except Exception:
            pass

        for dev in [
            self.a_in1, self.a_in2,
            self.b_in1, self.b_in2,
            self.pwm_A, self.pwm_B
        ]:
            try:
                dev.close()
            except Exception:
                pass
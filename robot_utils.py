import RPi.GPIO as GPIO
from gpiozero import DistanceSensor
import threading
import time


class Robot:
    Sensor_ECHO_Pin = 8
    Sensor_TRIGGER_Pin = 11

    Motor_A_EN = 4
    Motor_B_EN = 17
    Motor_A_Pin1 = 26
    Motor_A_Pin2 = 21
    Motor_B_Pin1 = 27
    Motor_B_Pin2 = 18

    def __init__(self, speed=100):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.Motor_A_EN, GPIO.OUT)
        GPIO.setup(self.Motor_B_EN, GPIO.OUT)
        GPIO.setup(self.Motor_A_Pin1, GPIO.OUT)
        GPIO.setup(self.Motor_A_Pin2, GPIO.OUT)
        GPIO.setup(self.Motor_B_Pin1, GPIO.OUT)
        GPIO.setup(self.Motor_B_Pin2, GPIO.OUT)

        self.pwm_A = GPIO.PWM(self.Motor_A_EN, 1000)
        self.pwm_B = GPIO.PWM(self.Motor_B_EN, 1000)
        self.pwm_A.start(0)
        self.pwm_B.start(0)

        self.speed = max(0, min(100, int(speed)))
        self._move_thread = None
        self._stop_flag = threading.Event()

        self.sensor = DistanceSensor(
            echo=self.Sensor_ECHO_Pin,
            trigger=self.Sensor_TRIGGER_Pin,
            max_distance=1
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
        if self._move_thread and self._move_thread.is_alive():
            self._stop_flag.set()
            self._move_thread.join(timeout=1.0)

        self._stop_flag.clear()
        self._move_thread = threading.Thread(
            target=self._run_motor,
            args=(motor_func,),
            daemon=True
        )
        self._move_thread.start()

    def motor_A(self, status):
        if status == 1:
            GPIO.output(self.Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
        elif status == -1:
            GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_B_Pin2, GPIO.HIGH)
        else:
            GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_B_Pin2, GPIO.LOW)

        self.pwm_B.ChangeDutyCycle(self.speed if status != 0 else 0)

    def motor_B(self, status):
        if status == 1:
            GPIO.output(self.Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
        elif status == -1:
            GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_A_Pin2, GPIO.HIGH)
        else:
            GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_A_Pin2, GPIO.LOW)

        self.pwm_A.ChangeDutyCycle(self.speed if status != 0 else 0)

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

        GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin2, GPIO.LOW)

        self.pwm_A.ChangeDutyCycle(0)
        self.pwm_B.ChangeDutyCycle(0)

    def cleanup(self):
        self.stop()

        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=1.0)

        try:
            self.sensor.close()
        except Exception:
            pass

        try:
            self.pwm_A.stop()
        except Exception:
            pass

        try:
            self.pwm_B.stop()
        except Exception:
            pass

        GPIO.cleanup()
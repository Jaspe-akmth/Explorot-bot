import time
from statistics import median

from gpiozero import DigitalOutputDevice, PWMOutputDevice, DistanceSensor
from gpiozero.pins.lgpio import LGPIOFactory


PIN_FACTORY = LGPIOFactory()


def clamp_pct(v):
    return max(0, min(100, int(v)))


class Robot:
    Sensor_ECHO_Pin = 8
    Sensor_TRIGGER_Pin = 11

    Motor_A_EN = 4
    Motor_B_EN = 17
    Motor_A_Pin1 = 26
    Motor_A_Pin2 = 21
    Motor_B_Pin1 = 27
    Motor_B_Pin2 = 18

    def __init__(self, drive_speed=78, turn_speed=78, pwm_freq_hz=1000):
        pf = PIN_FACTORY

        self.drive_speed = clamp_pct(drive_speed)
        self.turn_speed = clamp_pct(turn_speed)
        self.pwm_freq_hz = int(pwm_freq_hz)

        self.pwm_A = PWMOutputDevice(
            self.Motor_A_EN,
            frequency=self.pwm_freq_hz,
            pin_factory=pf,
        )
        self.pwm_B = PWMOutputDevice(
            self.Motor_B_EN,
            frequency=self.pwm_freq_hz,
            pin_factory=pf,
        )

        self.a_in1 = DigitalOutputDevice(self.Motor_A_Pin1, pin_factory=pf)
        self.a_in2 = DigitalOutputDevice(self.Motor_A_Pin2, pin_factory=pf)
        self.b_in1 = DigitalOutputDevice(self.Motor_B_Pin1, pin_factory=pf)
        self.b_in2 = DigitalOutputDevice(self.Motor_B_Pin2, pin_factory=pf)

        self.sensor = DistanceSensor(
            echo=self.Sensor_ECHO_Pin,
            trigger=self.Sensor_TRIGGER_Pin,
            max_distance=1.0,
            queue_len=5,
            pin_factory=pf,
        )

        self.stop()

    def set_drive_speed(self, pct):
        self.drive_speed = clamp_pct(pct)

    def set_turn_speed(self, pct):
        self.turn_speed = clamp_pct(pct)

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

    def _set_left_motor(self, status, speed_pct):
        # garde ton câblage d'origine
        self._set_motor_outputs(self.b_in1, self.b_in2, status)
        self.pwm_B.value = (speed_pct / 100.0) if status != 0 else 0.0

    def _set_right_motor(self, status, speed_pct):
        # garde ton câblage d'origine
        self._set_motor_outputs(self.a_in1, self.a_in2, status)
        self.pwm_A.value = (speed_pct / 100.0) if status != 0 else 0.0

    def _apply(self, left_status, right_status, left_speed_pct, right_speed_pct):
        self._set_left_motor(left_status, left_speed_pct)
        self._set_right_motor(right_status, right_speed_pct)

    def avance(self):
        self._apply(-1, -1, self.drive_speed, self.drive_speed)

    def recule(self):
        self._apply(1, 1, self.drive_speed, self.drive_speed)

    def gauche(self):
        self._apply(-1, 1, self.turn_speed, self.turn_speed)

    def droite(self):
        self._apply(1, -1, self.turn_speed, self.turn_speed)

    def stop(self):
        self._apply(0, 0, 0, 0)

    def get_sensor_distance(self):
        vals = []
        for _ in range(5):
            try:
                d = float(self.sensor.distance)
                if 0.02 <= d <= 1.0:
                    vals.append(d)
            except Exception:
                pass
            time.sleep(0.008)

        if not vals:
            return 1.0

        d = median(vals)

        if d < 0.02:
            d = 0.02
        if d > 1.0:
            d = 1.0
        return d

    def cleanup(self):
        self.stop()

        try:
            self.sensor.close()
        except Exception:
            pass

        for dev in [
            self.a_in1, self.a_in2,
            self.b_in1, self.b_in2,
            self.pwm_A, self.pwm_B,
        ]:
            try:
                dev.close()
            except Exception:
                pass
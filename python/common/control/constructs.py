import attr


@attr.frozen
class PIDGains:
    kp: float
    ki: float
    kd: float

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        
        self._integral = 0.0
        self._last_error = 0.0
        
    def update(self, current_value: float, dt: float) -> float:
        """
        Calculate the PID output.
        """
        error = self.setpoint - current_value
        
        self._integral += error * dt
        derivative = (error - self._last_error) / dt if dt > 0 else 0.0
        
        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)
        
        self._last_error = error
        return output

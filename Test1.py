import time

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.clear()

    def clear(self):
        self.setpoint = 0.0
        self.last_time = time.time()
        self.last_error = 0.0
        self.integral = 0.0

    def update(self, feedback_value):
        error = self.setpoint - feedback_value
        current_time = time.time()
        delta_time = current_time - self.last_time
        delta_error = error - self.last_error

        self.integral += error * delta_time
        derivative = delta_error / delta_time

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        self.last_time = current_time
        self.last_error = error

        return output

# Example usage
if __name__ == "__main__":
    # Initialize PID controller
    pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.5)

    # Set the desired setpoint
    pid.setpoint = 10.0

    # Simulate feedback loop
    for _ in range(10):
        # Get feedback value from sensor or system
        feedback = 8.0

        # Compute control output
        control_output = pid.update(feedback)

        # Apply control output to system or actuator
        # In this example, we simply print the control output
        print("Control output:", control_output)

        # Wait for a small time interval
        time.sleep(1.0)

import time
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.clear()

    def clear(self):
        self.setpoint = 0.0
        self.last_time = time.time()
        print(self.last_time)
        self.last_error = 0.0
        self.integral = 0.0

    def update(self, feedback_value):
        error = self.setpoint - feedback_value
        current_time = time.time()
        time.sleep(1)
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
    pid.setpoint = 10


    # Lists to store time and control output values for plotting
    time_values = []
    control_output_values = []
    a = 0
    # Simulate feedback loop
    for _ in range(100):
        # Get feedback value from sensor or system
        feedback = 8.0

        # Compute control output
        control_output = pid.update(feedback)

        # Append time and control output values
        if a > 0:
            time_values.append(time.time())
            control_output_values.append(control_output)
        
        a = a + 1

        # Apply control output to system or actuator
        # In this example, we simply print the control output
        print("Control output:", control_output)

        # Wait for a small time interval
        time.sleep(0.01)

    # Plot the control output over time
    plt.plot(time_values, control_output_values)
    plt.xlabel('Time')
    plt.ylabel('Control Output')
    plt.title('PID Control Output')
    plt.grid(True)
    plt.show()

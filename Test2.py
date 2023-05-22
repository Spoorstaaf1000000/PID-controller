import time
import random
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
        self.last_error = 0.0
        self.integral = 0.0

    def update(self, feedback_value):
        error = self.setpoint - feedback_value
        current_time = time.time()
        delta_time = current_time - self.last_time
        delta_error = error - self.last_error

        self.integral += error * delta_time
        derivative = delta_error / delta_time if delta_time > 0 else 0

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        self.last_time = current_time
        self.last_error = error

        return output

# Example usage
if __name__ == "__main__":
    # Initialize PID controller
    pid = PIDController(Kp=10.0, Ki=0, Kd=0)
    # Set the desired setpoint
    pid.setpoint = 10.0
    # Lists to store time, setpoint, feedback, and control output values for plotting
    time_values = []
    setpoint_values = []
    feedback_values = []
    control_output_values = []

    # Start time for elapsed time calculation
    start_time = time.time()
    feedback = 10

    # Simulate feedback loop
    for _ in range(100):
        # Get feedback value from sensor or system
        # In this example, generate random noise around the setpoint
        feedback = feedback + random.uniform(-0.5, 0.5)

        # Compute control output
        control_output = pid.update(feedback)
        # Append elapsed time, setpoint, feedback, and control output values
        elapsed_time = time.time() - start_time
        time_values.append(elapsed_time)
        setpoint_values.append(pid.setpoint)
        feedback_values.append(feedback)
        control_output_values.append(control_output)

        # Apply control output to system or actuator
        # In this example, we do not apply it to the system but print the control output
        print("Feedback:", feedback)

        # Wait for a small time interval
        time.sleep(0.1)

    # Plot the setpoint, feedback, and control output over time
    plt.plot(time_values, setpoint_values, label='Setpoint')
    plt.plot(time_values, feedback_values, label='Feedback')
    # plt.plot(time_values, control_output_values, label='Control Output')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('PID Control')
    plt.legend()
    plt.grid(True)
    plt.show()

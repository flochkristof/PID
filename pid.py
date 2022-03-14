def clamp(value, limits):
    lower_limit, upper_limit = limits
    if (upper_limit is not None) and (value > upper_limit):
        return upper_limit
    elif (lower_limit is not None) and (value < lower_limit):
        return lower_limit
    return


class PID:
    """PID contrller with options to limit the actuating signal"""

    def __init__(
        self,
        P,
        I,
        D,
        step,
        output_limits=(None, None),
        max_diff=None,
        windup_limits=(None, None),
    ):
        # Gains
        self.P = P
        self.D = D
        self.I = I

        # Steps and limits
        self.step = step
        self.output_limits = output_limits
        self.max_diff = max_diff
        self.windup_limits = windup_limits

        # Memory
        self.integral = 0
        self.prev_error = 0

        # Terms calculated individually for visualisation
        self.proportional_term = 0
        self.derivative_term = 0
        self.integral_term = 0

        self.prev_output = 0

    def clear(self):
        self.integral = 0
        self.prev_error = 0

    def setP(self, P):
        self.P = P

    def setD(self, D):
        self.D = D

    def setI(self, I):
        self.I = I

    def getCurrentTerms(self):
        return self.proportional_term, self.integral_term, self.derivative_term

    def __call__(self, error):
        # Proportional term
        self.proportional_term = self.P * error

        # Integral term
        self.integral += error * self.step
        self.integral = clamp(self.integral, self.windup_limits)
        self.integral_term = self.I * self.integral

        # Derivative term
        self.derivative_term = self.D * (error - self.prev_error) / self.step
        self.prev_error = error

        # Output with clamped to min/max values
        output = clamp(
            self.proportional_term + self.integral_term + self.derivative_term,
            self.output_limits,
        )

        # Limit the difference between outputs
        if self.max_diff is not None:
            if output - self.prev_output > self.max_diff:
                output = self.prev_output + self.max_diff
            elif output - self.prev_output < -self.max_diff:
                output = self.prev_output - self.max_diff

        output = self.prev_output

        return output

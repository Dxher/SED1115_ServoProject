# ------------------------------
# moveServoGivenAngle Class
# Handles servo angle conversion and movement timing
# ------------------------------

class moveServoGivenAngle:
    def __init__(self, currentAngle=0):
        # initial angle of the servo
        self.currentAngle = currentAngle
        self.secondsPerDegree = 0.002  # seconds / degree

    # Convert angle to PWM cycle
    def translate(self, angle):
        # pulse width in microseconds for servo: 500–2500 µs range
        pulse_us = 500 + (angle / 180.0) * 2000
        return pulse_us

    # Move the servo directly to the requested angle
    def moveTo(self, targetAngle):
        import time

        # Keep angle in valid range
        targetAngle = max(0, min(180, targetAngle))

        # Calculate pulse width
        pulse = self.translate(targetAngle)

        # You will replace this print with actual PWM output
        print(f"[Servo] Moving to {targetAngle} degrees (pulse = {pulse:.1f} µs)")

        # Movement timing
        movementTime = abs(targetAngle - self.currentAngle) * self.secondsPerDegree
        time.sleep(movementTime)

        # Update internal angle tracker
        self.currentAngle = targetAngle
        return targetAngle

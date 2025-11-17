from machine import Pin, PWM
import time

class ServoIO:
    def __init__(self, currentAngle=0):
        # constructor
        self.currentAngle = currentAngle
        self.secondsPerDegree = 0.002  # seconds / degree

        # initialize servo hardware on GPIO 0 pin, sets up frequency
        self.servo = PWM(Pin(0))
        self.servo.freq(50)

    # Convert angle to PWM cycle
    def translate(self, angle): 
        pulse_us = 500 + (2000 * angle / 180)  # math to get width of the pulse in the PWM signal, the "high" width btw
        duty = int(pulse_us / 20000 * 65535) #this is the conversion to duty cycle from pulse width which itself is from the angle
        return duty

    # Calculates the time and direction needed to move from current to target 
    def timeForAngle(self, targetAngle):
        # Angle difference, basicaly how many degrees do i need to move from here to get there
        delta = targetAngle - self.currentAngle

        # Directionality determination
        if delta > 0:
            direction = "positive"
            step = 1
        elif delta < 0:
            direction = "negative"
            step = -1
        else:
            direction = "none"
            step = 0

        # time required for the servo to be running based on the degrees needed to move * time to move / degree.
        secondsNeeded = abs(delta) * self.secondsPerDegree

        # moves servo with magic (uses the python commands to control the servo via pico. in case you didn't know, self 
        #is the object that repersents the thing your doing, in our case, a servo). Read up on objects, its important for this project.
        if step != 0:
            for angle in range(self.currentAngle, targetAngle + step, step):
                duty = self.translate(angle)
                self.servo.duty_u16(duty)
                time.sleep(self.secondsPerDegree)   # timing based on servo speed

        # Updates angle, may want to remove this if it doesnt work, idk its just easier to do here i guess but it may fck w calibrartion
        self.currentAngle = targetAngle

        # Returns usefull data, if we care for testing.
        return {
            "direction": direction,
            "delta_degrees": abs(delta),
        }


from machine import ADC
import time

def main():
    # create servo objects
    shoulder = moveServoGivenAngle(currentAngle=90)
    elbow = moveServoGivenAngle(currentAngle=90)
    pen = moveServoGivenAngle(currentAngle=0)

    # potentiometers
    potX = ADC(26)  # pin 26 → shoulder
    potY = ADC(27)  # pin 27 → elbow

    print("Running main…")

    while True:
        # read raw ADC
        x_val = potX.read_u16()
        y_val = potY.read_u16()

        # map to 0–180 degrees
        shoulderAngle = int((x_val / 65535) * 180)
        elbowAngle = int((y_val / 65535) * 180)

        # move servos
        shoulder.timeForAngle(shoulderAngle)
        elbow.timeForAngle(elbowAngle)

        # pen servo unchanged until you add controls

        time.sleep(0.01)


main()

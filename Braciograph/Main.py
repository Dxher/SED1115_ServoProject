from machine import ADC
import time

def main():
    # create servo objects
    shoulder = ServoIO(currentAngle=90)   # uses GPIO 0 automatically
    elbow = ServoIO(currentAngle=90)      # ALSO uses GPIO 0 unless you change class
    pen = ServoIO(currentAngle=0)         # same issue—class always uses pin 0

    # create potentiometer ADC objects
    potX = ADC(26)   # pin 26 → shoulder
    potY = ADC(27)   # pin 27 → elbow

    print("Running main…")

    while True:
        # read pots (0–65535)
        x_val = potX.read_u16()
        y_val = potY.read_u16()

        # map to 0–180 degrees
        shoulderAngle = int((x_val / 65535) * 180)
        elbowAngle = int((y_val / 65535) * 180)

        # move servos using your class
        shoulder.timeForAngle(shoulderAngle)
        elbow.timeForAngle(elbowAngle)

        # pen remains unchanged until you add input for it

        time.sleep(0.01)


# run main on boot
main()

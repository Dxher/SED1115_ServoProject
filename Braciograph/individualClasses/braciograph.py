from machine import Pin, PWM, ADC
import math

# Constants
ArmSE = 155 # Length of shoulder-to-elbow link (mm)
ArmEW = 155 # Length of elbow-to-wrist link (mm)
ShoulderX, ShoulderY = -50, 139.5  # shoulder offset from paper origin

ServoFreq = 50 # servo PWM frequency (Hz)

paper_x_max = 215 #(mm)
paper_y_max = 279 #(mm)
y_divider = 234 # Variable to convert duty to y in mm
x_divider = 304 # Variable to convert duty to x in mm

# Servo setup
pwm_shoulder = PWM(Pin(0))
pwm_shoulder.freq(ServoFreq)

pwm_elbow = PWM(Pin(1))
pwm_elbow.freq(ServoFreq)

# Potentiometers
pot_x = ADC(27)
pot_y = ADC(26)

def read_potentiometer(pot):
     return pot.read_u16() # return potentionmeter value (0–65535)

def translate(angle):
	"""
	Converts an angle in degrees to the corresponding input
	for the duty_u16 method of the servo class
	This function was provided from Lab7
	"""
	MIN = 1638 # 0 degrees
	MAX = 8192 # 180 degrees
	DEG = (MAX - MIN) / 180 # value per degree

	# clamp angle to be between 0 and 180
	angle = max(0, min(180, angle))

	return int(angle * DEG + MIN)

def inverse_kinematics(Cx, Cy):
    """
    Given the desired coordinates, this function allows us to do the math to retrieve
    the shoulder and elbows angles. All the equations were retrieved from SED1115 Lab8
    """
    AC = math.sqrt((ShoulderX - Cx)**2 + (ShoulderY - Cy)**2)
    AbaseC = math.sqrt((ShoulderX - Cx)**2 + Cy**2)
    Angle_BAC = math.acos((ArmSE**2 + AC**2 - ArmEW**2) / (2 * ArmSE * AC))
    Angle_ACB = math.asin((ArmSE * math.sin(Angle_BAC)) / ArmEW)
    Angle_YAC = math.acos((ShoulderY**2 + AC**2 - AbaseC**2) / (2 * ShoulderY * AC))
    alpha = math.degrees(Angle_YAC + Angle_BAC)
    beta = math.degrees(Angle_ACB + Angle_BAC)
    return alpha, beta

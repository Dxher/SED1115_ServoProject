from machine import Pin, PWM, ADC

# Constants
ArmSE = 155 # Length of shoulder-to-elbow link (mm)
ArmEW = 155 # Length of elbow-to-wrist link (mm)
ShoulderX, ShoulderY = -50, 139.5  # shoulder offset from paper origin

ServoFreq = 50 # servo PWM frequency (Hz)

paper_x_max = 215
paper_y_max = 279
y_divider = 234
x_divider = 304

# Servo setup
pwm_shoulder = PWM(Pin(0))
pwm_shoulder.freq(ServoFreq)

pwm_elbow = PWM(Pin(1))
pwm_elbow.freq(ServoFreq)

# Potentiometers
pot_x = ADC(27)
pot_y = ADC(26)

def read_potentiometer(pot):
     duty = pot.read_u16()    # read potentionmeter value (0–65535)
     return duty

def translate(angle):
	"""
	Converts an angle in degrees to the corresponding input
	for the duty_u16 method of the servo class
	"""
	MIN = 1638 # 0 degrees
	MAX = 8192 # 180 degrees
	DEG = (MAX - MIN) / 180 # value per degree

	# clamp angle to be between 0 and 180
	angle = max(0, min(180, angle))

	return int(angle * DEG + MIN)
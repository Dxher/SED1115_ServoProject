from machine import Pin, PWM, ADC
import math, time

# Constants
ServoFreq = 50  # Hz

# Robot arm dimensions (in mm)
Lab = 155  # shoulder to elbow
Lbc = 155  # elbow to pen

# Shoulder base position (in mm)
ShoulderX = -50
ShoulderY = 139.5

# Default calibration (will be overwritten by file)
SHOULDER_A = 1.0
SHOULDER_B = 0.0
ELBOW_A = 1.0
ELBOW_B = 0.0

# Drawing area (paper size in mm)
PAPER_WIDTH = 215
PAPER_HEIGHT = 279

# Servos
pwm_shoulder = PWM(Pin(0))
pwm_shoulder.freq(ServoFreq)
pwm_elbow = PWM(Pin(1))
pwm_elbow.freq(ServoFreq)
pwm_pen = PWM(Pin(2))
pwm_pen.freq(ServoFreq)

# Potentiometers
pot_x = ADC(Pin(26)) #X-axis potentiometer
pot_y = ADC(Pin(27)) #Y-axis potentiometer

# Button for pen toggle
pen_button = Pin(22, Pin.IN, Pin.PULL_DOWN)

# Pen state
pen_is_down = False
pen_moving = False
pen_move_start_time = 0
pen_up_angle = 0    # Angle when pen is lifted
pen_down_angle = 30 # Angle when pen is on paper



# SERVO HELPERS
def translate(angle):
    MIN = 1638.     #min angle 0 degrees
    MAX = 8192      #max angle 180 degrees
    DEG = (MAX - MIN) / 180 # PWM units per degree of servo movement
    angle = max(0, min(180, angle)) #clamp angle to be between 0 and 180
    return int(angle * DEG + MIN)   #convert degrees to PWM units


def set_servo_deg(pwm, angle_deg): #Give the servo (pwm) an angle (angle_deg) to go to
    pwm.duty_u16(translate(angle_deg)) # send to servo


# CALIBRATION READER + FIT FUNCTIONS
def read_calibrated_angles(filename="calibrated_angles.txt"):
    """
    Reads calibration data from file structured as:

    [SHOULDER]
    desired,actual,error
    ...
    [ELBOW]
    desired,actual,error
    ...
    """

    data = {"SHOULDER": [], "ELBOW": []} #
    current_section = None  #tracks which servo we are reading data for

    try:
        with open(filename, "r") as file: #open file 
            for line in file:   #read line by line
                line = line.strip() #remove whitespace
                if not line:    #skip empty lines
                    continue
                if line.startswith("#"): #skip comments
                    continue

                # Section header
                if line.startswith("[") and line.endswith("]"): #detect section
                    name = line[1:-1].strip().upper() #get section name
                    current_section = name if name in data else None #validate section
                    continue

                if current_section is None: # Ignore data outside known sections

                    continue

                parts = line.split(",") #split line into parts
                if len(parts) < 2:
                    continue

                try:
                    desired = float(parts[0]) #convert to desired angle
                    actual = float(parts[1]) #convert to actual angle
                except ValueError:
                    continue. #skip invalid lines

                data[current_section].append((desired, actual)) #store pair

        return data
#error handling
    except OSError as e:
        print("Error opening file:", filename, e)
        return {"SHOULDER": [], "ELBOW": []}
    except Exception as e:
        print("Error parsing calibration file:", e)
        return {"SHOULDER": [], "ELBOW": []}


def compute_linear_fit(pairs):
    """
    Compute A and B for mapping: actual ≈ A * desired + B
    """
    n = len(pairs)
    if n < 2:
        raise ValueError("Not enough calibration points") #need at least 2 points

    sum_x = sum_y = sum_xx = sum_xy = 0.0 #initialize formulas

    for x, y in pairs:
        sum_x  += x #sum of desired angles
        sum_y  += y #sum of actual angles
        sum_xx += x * x #sum of desired^2
        sum_xy += x * y #sum of desired * actual

    denom = n * sum_xx - sum_x * sum_x
    if denom == 0:
        raise ValueError("Degenerate calibration data")
#solve the intersection between A and B
    A = (n * sum_xy - sum_x * sum_y) / denom
    B = (sum_y * sum_xx - sum_x * sum_xy) / denom

    return A, B


def load_calibration(filename="calibrated_angles.txt"):
    """
    Load calibration, compute A/B, update globals.
    """
    global SHOULDER_A, SHOULDER_B, ELBOW_A, ELBOW_B

    data = read_calibrated_angles(filename)

    if data["SHOULDER"]:
        try:
            SHOULDER_A, SHOULDER_B = compute_linear_fit(data["SHOULDER"])
            print("Loaded SHOULDER calibration:", SHOULDER_A, SHOULDER_B)
        except Exception as e:
            print("Shoulder calibration failed:", e)

    if data["ELBOW"]:
        try:
            ELBOW_A, ELBOW_B = compute_linear_fit(data["ELBOW"])
            print("Loaded ELBOW calibration:", ELBOW_A, ELBOW_B)
        except Exception as e:
            print("Elbow calibration failed:", e)



# INVERSE KINEMATICS

def inverse_kinematics(Cx, Cy):
    """
    Given a target point C (Cx, Cy) in mm, compute 
    shoulder_servo_deg and elbow_servo_deg
    """

    # 1) AC length and reachability check
    dx = Cx - ShoulderX
    dy = Cy - ShoulderY
    Lac = math.sqrt(dx*dx + dy*dy)

    #Two arms lengths added
    max_reach = Lab + Lbc
    if Lac == 0 or Lac > max_reach:
        raise ValueError("Point C is unreachable for this arm")

    # 2) Geometric elbow angle (theta2), elbow-up solution ---
    # Law of cosines for elbow angle
    cos_t2 = (dx*dx + dy*dy - Lab*Lab - Lbc*Lbc) / (2 * Lab * Lbc)
    # Clamp for numerical safety
    cos_t2 = min(1, max(-1,cos_t2))

    # Elbow-up → positive sin
    sin_t2 = math.sqrt(max(0.0, 1.0 - cos_t2*cos_t2))
    theta2 = math.atan2(sin_t2, cos_t2)  # radians

    # --- 3) Geometric shoulder angle (theta1) ---
    k1 = Lab + Lbc * cos_t2
    k2 = Lbc * sin_t2
    theta1 = math.atan2(dy, dx) - math.atan2(k2, k1)  # radians

    # Convert to degrees
    shoulder_geom_deg = math.degrees(theta1)
    elbow_geom_deg    = math.degrees(theta2)

    # --- 4) Map geometric angles → servo angles using calibration ---
    shoulder_servo = SHOULDER_A * shoulder_geom_deg + SHOULDER_B
    elbow_servo    = ELBOW_A    * elbow_geom_deg    + ELBOW_B

    # Clamp to paper max degrees
    servoA = max(13, min(162, shoulder_servo))
    servoB = max(1, min(140, elbow_servo))

    return servoA, servoB

# POSITION + MOVEMENT LOGIC
def read_potentiometers():
    """
    Read potentiometer values and map to X, Y coordinates
    
    Returns:
        tuple: (x, y) coordinates in mm
    """
    # Read raw ADC values (0-65535 for 16-bit ADC)
    raw_x = pot_x.read_u16()
    raw_y = pot_y.read_u16()
    
    # Map to paper coordinates
    # Apply some smoothing by averaging with current position
    x = (raw_x / 65535) * 215
    y = (raw_y / 65535) * 279
    
    return x, y

def send_angle(shoulder_angle, elbow_angle):    
    # Send to servos
    set_servo_deg(pwm_shoulder, shoulder_angle)
    set_servo_deg(pwm_elbow, elbow_angle)

def move_to(x, y):
    """
    Move the pen to position (x, y) in mm
    """
        #logic that prevents it from gooing out of bound shere

    # I was just using this to see x, y values during testing - Ev
    #print(x,y)    

    servoA, servoB = inverse_kinematics(x, y) # Calculate servo angles

    send_angle(servoA, servoB) # Send angles to servos
    
    time.sleep_ms(50)

def toggle_pen():
    global pen_is_down, pen_moving, pen_move_start_time
    """Toggle pen between up and down states"""
    if pen_is_down:
        set_servo_deg(pwm_pen, pen_up_angle)
        pen_is_down = False
        pen_moving = True
        pen_move_start_time = time.ticks_ms()
        print("Pen UP")
    else:
        if not pen_is_down:
            set_servo_deg(pwm_pen, pen_down_angle)
            pen_is_down = True
            pen_moving = True
            pen_move_start_time = time.ticks_ms()
            print("Pen DOWN")
# MAIN LOOP
def main():    # Read the initial state of the pen button
    prev = pen_button.value() 

    while True: #starting a loop
        now = pen_button.value() #read the state of the button
 # Detect rising edge: button was NOT pressed before, but IS pressed now
        if prev == 0 and now == 1: #
            toggle_pen()
            time.sleep_ms(200) #delay for debouncing to avoid double trigering

        prev = now

        x, y = read_potentiometers() # Read X and Y analog values from potentiometers
        move_to(x, y) # Move the pen to (x, y)

        time.sleep_ms(10) #to reduce cpu load
#program 

if __name__ == "__main__": 
    try:
        load_calibration("calibrated_angles.txt") #Load calibration offsets for arm accuracy
        main()
    except Exception as e:        # Start the main control loop
        print("\nProgram stopped:", e)  #error handling

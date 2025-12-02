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

# Constants that calbrate geometric angles to servo angles
SHOULDER_A = 1
SHOULDER_B = 146
ELBOW_A = -1.1
ELBOW_B = 171.11

# Servos
pwm_shoulder = PWM(Pin(0))
pwm_shoulder.freq(ServoFreq)
pwm_elbow = PWM(Pin(1))
pwm_elbow.freq(ServoFreq)
pwm_pen = PWM(Pin(2))
pwm_pen.freq(ServoFreq)

# Potentiometers for X and Y control
pot_x = ADC(Pin(26))  # X-axis potentiometer
pot_y = ADC(Pin(27))  # Y-axis potentiometer

# Button for pen toggle
pen_button = Pin(22, Pin.IN, Pin.PULL_DOWN)

# Pen state
pen_is_down = False
pen_moving = False
pen_move_start_time = 0
pen_up_angle = 0      # Angle when pen is lifted
pen_down_angle = 30    # Angle when pen is on paper

def translate(angle):
    """
    Converts an angle in degrees to the corresponding input
    for the duty_u16 method of the servo class
    """
    MIN = 1638  # 0 degrees
    MAX = 8192  # 180 degrees
    DEG = (MAX - MIN) / 180  # value per degree

    # clamp angle to be between 0 and 180
    angle = max(0, min(180, angle))

    return int(angle * DEG + MIN)

def set_servo_deg(pwm, angle_deg):
    """Give the servo (pwm) an angle (angle_deg) to go to"""
    pwm.duty_u16(translate(angle_deg))

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

def read_gcode(filename):
    """
    Reads a G-code file and returns a list of commands
    """
    commands = []
    try:
        with open(filename, "r") as file:
            for line in file:
                line = line.strip() # Remove leading/trailing whitespace
                if not line: # Skip empty lines
                    continue

                parts = line.split() # Split line into parts
                cmd = parts[0] # First part is the command
                params = {} 

                for token in parts[1:]:# Process parameters
                    key = token[0] # Parameter key (first character)
                    value = float(token[1:]) # Make value a float
                    params[key] = value # Store parameter

                commands.append({"cmd": cmd, "params": params}) # Add command to list

        return commands
    except OSError:
        print("Error: Could not open file", filename)
        return []

def main():
    """
    Compare drawing results with and without calibration
    """
    prev_state = 1
    while True:

        current_state = pen_button.value()
        if prev_state == 0 and current_state == 1:
            toggle_pen()
            
            time.sleep_ms(200)
        
        prev_state = current_state
        
        x,y = read_potentiometers()

        """
        This code corrects the position of the pen by setting desired
        coordinates within bounds, the issue is that when the
        pen is moving, my overwrite is overwritten. It does, however
        fix the position when the pen stops. - Ev
        """
        if (x>0 or x<215) and (y>0 or y<279):
            move_to(x, y)
        if (x>215):
            x = 215
        else:
            x = 0
        if (y<0):
            y= 0
        else:
            y=279
            
        time.sleep_us(50)

if __name__ == "__main__":  
    try:  
        main()
    except:
        print("\nThe porgram has been forcefully stopped")

"""
Reword the read_gcode function to make it reaed our calibrated file.
Keep in mind error handling.

Using the variables, create a plot that would provide us with a new 
SHOULDER_A = 1
SHOULDER_B = 146
ELBOW_A = -1.1
ELBOW_B = 171.11

think of it as a slope: y=mx + b

Where the SHOULDER_A is m and SHOULDER_B is b

You'll need two slopes: 1 for Shoulder, 1 for Elbow

This is all conceptual, I don't know if this would actually work ;)


"""
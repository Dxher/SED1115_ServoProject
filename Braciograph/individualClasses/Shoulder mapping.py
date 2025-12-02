def compute_servo_angles(alpha_degrees, beta_degrees):
    """
    Convert the math angles alpha and beta into the actual servo angles
    for the shoulder and elbow servos.

    You MUST tune the offsets and directions for your setup.
    These example formulas are a starting point based on typical brachiograph layouts.
    """

    # --- SHOULDER SERVO MAPPING ---
    # Example idea:
    #   alpha = 0   -> servo around 90 degrees (arm straight "up")
    #   alpha increases -> servo rotates in the opposite direction
    #
    # So we map servo_shoulder_angle = SHOULDER_CENTER - alpha
    SHOULDER_CENTER_ANGLE = 90   # adjust if your "center" is different
    shoulder_servo_angle = SHOULDER_CENTER_ANGLE - alpha_degrees

    # --- ELBOW SERVO MAPPING ---
    # Example idea:
    #   beta = 0   -> arm fully straight, elbow servo maybe at 90 degrees
    #   beta increases -> elbow bends, servo rotates
    #
    # So we map servo_elbow_angle = ELBOW_CENTER + (beta_degrees - 90) or similar.
    ELBOW_CENTER_ANGLE = 90      # adjust as needed
    elbow_servo_angle = ELBOW_CENTER_ANGLE + (beta_degrees - 90)

    # CLAMP BOTH TO [0, 180] SO WE NEVER ASK THE SERVO FOR INVALID ANGLES
    if shoulder_servo_angle < 0:
        shoulder_servo_angle = 0
    if shoulder_servo_angle > 180:
        shoulder_servo_angle = 180

    if elbow_servo_angle < 0:
        elbow_servo_angle = 0
    if elbow_servo_angle > 180:
        elbow_servo_angle = 180

    return shoulder_servo_angle, elbow_servo_angle

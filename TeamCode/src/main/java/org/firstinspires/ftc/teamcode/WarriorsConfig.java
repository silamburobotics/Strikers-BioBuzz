package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * Configuration Parameters & Tunables for Team WARRIORS.
 * Editable live via FTC Dashboard (http://192.168.43.1:8080/dash).
 */
@Config
public class WarriorsConfig {

    // --- Mecanum Drive Configuration ---
    public static String LEFT_FRONT_NAME = "leftFront";
    public static String LEFT_BACK_NAME = "leftBack";
    public static String RIGHT_FRONT_NAME = "rightFront";
    public static String RIGHT_BACK_NAME = "rightBack";
    public static double DRIVE_POWER_SCALE = 1.0;
    public static boolean INVERT_DRIVE_Y = true;        // Standard drive direction (Push Up = Forward)
    public static boolean INVERT_STRAFE_X = false;       // Standard strafe direction (Push Right = Strafe Right)
    public static boolean INVERT_TURN_JOYSTICK = true;  // Standard inverted turning (rx = -right_stick_x)

    // --- Intake Motor & Dual Synchronized Lift Servos ---
    public static String INTAKE_NAME = "intake";
    public static double INTAKE_POWER = 1.0;
    public static String INTAKE_LIFT_LEFT_NAME = "intakeLiftLeft";
    public static String INTAKE_LIFT_RIGHT_NAME = "intakeLiftRight";
    public static double INTAKE_LIFT_UP_POS = 0.40;       // UP height position
    public static double INTAKE_LIFT_DOWN_POS = 0.80;     // DOWN height position
    public static double RIGHT_SERVO_TRIM_OFFSET = 0.0;   // Trim offset for right servo

    // --- Axon Gate & Shifter Servos ---
    public static String GATE_SERVO = "gateServo";
    public static double OPEN_POSITION = 0.4;             // Gate Open position
    public static double CLOSED_POSITION = 0.7;           // Gate Closed position

    public static String SHIFTER_SERVO = "shifterServo";
    public static double SHIFTER_OPEN_POSITION = 0.25;    // Shifter Open position (Nectar)
    public static double SHIFTER_CLOSED_POSITION = 0.95;  // Shifter Closed position (Pollen)

    // --- Turret Motor Configuration ---
    public static String TURRET_NAME = "turret";
    public static double TURRET_TICKS_PER_REV = 537.7;    // GoBILDA 312 RPM output encoder resolution
    public static double TURRET_GEAR_RATIO = 2.2727;      // GoBILDA 175t:77t turret gear ratio (175.0 / 77.0)
    public static double TURRET_MAX_POWER = 0.35;         // Capped power limit (35%)
    public static double TURRET_MAX_VELOCITY_DEG_PER_SEC = 90.0; // Max speed cap in degrees/sec (90°/sec)
    public static double TURRET_MAX_ANGLE_DEG = 90.0;     // +/- 90 degrees soft limit from center home
    public static boolean TURRET_REVERSED = false;        // Set true if turret rotates opposite

    // --- Flywheel Motors Configuration ---
    public static String MOTOR_1_NAME = "flyWheel1";
    public static String MOTOR_2_NAME = "flyWheel2";
    public static double FLYWHEEL_OPEN_RPM = 2320.0;     // Manual RPM when shifter is OPEN / Nectar mode
    public static double FLYWHEEL_CLOSED_RPM = 2150.0;   // Manual RPM when shifter is CLOSED / Pollen mode
    public static double TICKS_PER_REV = 28.0;         // GoBILDA 6000 RPM motor encoder resolution
    public static boolean MOTOR_2_REVERSED = true;     // Set true if motor 2 spins opposite to motor 1
    public static boolean FLYWHEEL_ENABLED = false;    // Toggle flywheel motors ON/OFF
}

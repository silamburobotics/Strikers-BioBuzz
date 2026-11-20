package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Configuration Parameters & Tunables for Team STRIKERS.
 * Editable live via FTC Dashboard (http://192.168.43.1:8080/dash).
 */
@Config
public class StrikersConfig {

    // --- Mecanum Drive Configuration ---
    public static String LEFT_FRONT_NAME = "leftFront";
    public static String LEFT_BACK_NAME = "leftBack";
    public static String RIGHT_FRONT_NAME = "rightFront";
    public static String RIGHT_BACK_NAME = "rightBack";
    public static double DRIVE_POWER_SCALE = 1.0;
    public static boolean INVERT_DRIVE_Y = false;        // Standard drive direction (Push Up = Forward)
    public static boolean INVERT_STRAFE_X = false;       // Standard strafe direction (Push Right = Strafe Right)
    public static boolean INVERT_TURN_JOYSTICK = false; // Reversed turning (rx = right_stick_x)

    // --- Intake Motor & Dual Synchronized Lift Servos ---
    public static String INTAKE_NAME = "intake";
    public static double INTAKE_POWER = 1.0;
    public static String INTAKE_LIFT_LEFT_NAME = "intakeLiftLeft";
    public static String INTAKE_LIFT_RIGHT_NAME = "intakeLiftRight";
    public static double INTAKE_LIFT_UP_POS = 0.78;       // Shared UP height position (Right servo automatically mirrored)
    public static double INTAKE_LIFT_DOWN_POS = 0.375;     // Shared DOWN height position (Right servo automatically mirrored)

    // --- Axon Gate & Shifter Servos ---
    public static String GATE_SERVO = "gateServo";
    public static double OPEN_POSITION = 0.4;             // Gate Open position
    public static double CLOSED_POSITION = 0.7;           // Gate Closed position
    public static double GATE_KICK_POSITION = 0.4;        // When the gate opens it first goes here...
    public static double GATE_KICK_MS = 200.0;            // ...for this many ms, then comes back to OPEN_POSITION

    public static String SHIFTER_SERVO = "shifterServo";
    public static double SHIFTER_OPEN_POSITION = 0.25;    // Shifter Open position (Nectar)
    public static double SHIFTER_CLOSED_POSITION = 0.95;  // Shifter Closed position (Pollen)

    // --- Turret Motor Configuration ---
    public static String TURRET_NAME = "turret";
    public static double TURRET_TICKS_PER_REV = 537.7;    // GoBILDA 312 RPM output encoder resolution
    public static double TURRET_GEAR_RATIO = 2.2727;      // GoBILDA 175t:77t turret gear ratio (175.0 / 77.0)
    public static double TURRET_MAX_POWER = 0.25;         // Smooth power limit (25%) to prevent overshooting
    public static double TURRET_MAX_VELOCITY_DEG_PER_SEC = 90.0; // Max speed cap in degrees/sec (90°/sec)
    public static double TURRET_MAX_ANGLE_DEG = 90.0;     // +/- 90 degrees soft limit from center home
    public static boolean TURRET_REVERSED = false;        // Set true if turret rotates opposite

    // --- Flywheel Motors Configuration ---
    public static String MOTOR_1_NAME = "flyWheel1";
    public static String MOTOR_2_NAME = "flyWheel2";
    public static double FLYWHEEL_OPEN_RPM = 2575.0;     // Manual RPM when shifter is OPEN / Nectar mode
    public static double FLYWHEEL_CLOSED_RPM = 2400.0;   // Manual RPM when shifter is CLOSED / Pollen mode
    public static double TICKS_PER_REV = 28.0;         // GoBILDA 6000 RPM motor encoder resolution
    public static boolean MOTOR_1_REVERSED = false;    // Set true if motor 1 spins opposite
    public static boolean MOTOR_2_REVERSED = true;     // Set true if motor 2 spins opposite to motor 1
    public static boolean FLYWHEEL_ENABLED = false;    // Toggle flywheel motors ON/OFF

    // --- Battery-based flywheel speed (read once at init, used by teleop and all autos) ---
    public static double BATTERY_HIGH_VOLTS = 13.0;      // above this -> HIGH_BATTERY_RPM
    public static double BATTERY_LOW_VOLTS = 12.5;       // below this -> LOW_BATTERY_RPM
    public static double HIGH_BATTERY_RPM = 2800.0;      // battery > 13.0 V
    public static double MID_BATTERY_RPM = 2860.0;       // 12.5 V to 13.0 V, and the default if the battery cannot be read
    public static double LOW_BATTERY_RPM = 2900.0;       // battery < 12.5 V

    /** Lowest valid hub battery voltage, or 0 if it cannot be read. */
    public static double readBatteryVolts(HardwareMap hardwareMap) {
        double lowest = Double.POSITIVE_INFINITY;
        try {
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double v = sensor.getVoltage();
                if (!Double.isNaN(v) && v > 1.0 && v < lowest) lowest = v;
            }
        } catch (Exception e) {
            return 0.0;
        }
        return lowest == Double.POSITIVE_INFINITY ? 0.0 : lowest;
    }

    /** Flywheel RPM for the current battery. Falls back to MID_BATTERY_RPM if the voltage cannot be read. */
    public static double getBatteryAdjustedRPM(HardwareMap hardwareMap) {
        double volts = readBatteryVolts(hardwareMap);
        if (volts <= 0.0) return MID_BATTERY_RPM;
        if (volts > BATTERY_HIGH_VOLTS) return HIGH_BATTERY_RPM;
        if (volts < BATTERY_LOW_VOLTS) return LOW_BATTERY_RPM;
        return MID_BATTERY_RPM;
    }
}

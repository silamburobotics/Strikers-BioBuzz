package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * OpMode to control a GoBILDA 312 RPM Yellow Jacket motor driving a gear turret.
 * <p>
 * Gear Ratio Specifications:
 * - Motor Pinion Teeth: 77
 * - Turret Main Gear Teeth: 175
 * - Gear Ratio: 175 / 77 = 25 / 11 ~ 2.2727:1
 * - Encoder Ticks per Motor Rev: 537.7 (GoBILDA 312 RPM 5202/5203 series)
 * - Ticks per Turret Rev: 537.7 * (175 / 77) ~ 1222.045 ticks/rev
 * - Ticks per Turret Degree: ~ 3.39457 ticks/degree
 * <p>
 * Controls:
 * - Gamepad 1 Right Stick X: Manual analog turret rotation (constrained to -90° .. +90°)
 * - D-Pad Up / Down: 0° (Center / Initializing position)
 * - D-Pad Right: +90° (Maximum Right)
 * - D-Pad Left: -90° (Maximum Left)
 * - Button Y: +45°
 * - Button X: -45°
 * - Button B: +90°
 * - Button A: -90°
 * - Options / Back: Reset Zero Position Calibration to Current Physical Orientation
 */
@Config
@TeleOp(name = "Turret 312RPM Control", group = "TeleOp")
public class TurretControlOpMode extends LinearOpMode {

    // --- HARDWARE CONFIGURATION ---
    public static String MOTOR_NAME = "turret";
    public static boolean REVERSE_MOTOR = false;

    // --- GEAR RATIO & MOTOR CONSTANTS ---
    public static double MOTOR_TEETH = 77.0;
    public static double TURRET_TEETH = 175.0;
    public static double GOBILDA_312_RPM_TICKS_PER_REV = 537.7; // GoBILDA 312 RPM Output Shaft Encoder PPR

    // --- SAFETY & SPEED CONFIGURATION ---
    public static double MAX_TURRET_POWER = 0.30; // Capped max power (0.0 to 1.0) for slow & smooth motion
    public static double MAX_TURRET_VELOCITY_DEG_PER_SEC = 60.0; // Velocity cap in degrees/sec (e.g. 60°/sec = 1.5s for 90°)
    public static double TURRET_MIN_ANGLE_DEG = -90.0; // Minimum allowed turret rotation angle relative to init (-90°)
    public static double TURRET_MAX_ANGLE_DEG = 90.0;  // Maximum allowed turret rotation angle relative to init (+90°)
    public static double MANUAL_ROTATION_SPEED_DEG_PER_SEC = 45.0; // Degrees per second in manual stick mode

    // Hardware device
    private DcMotorEx turretMotor;

    // State variables
    private double targetAngleDegrees = 0.0;
    private boolean isManualMode = false;

    // Gamepad button edge detection
    private boolean lastOptions = false;

    @Override
    public void runOpMode() {
        // Broadcast telemetry to both Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Turret Motor with fallback lookup
        try {
            turretMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        } catch (Exception e1) {
            try {
                turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
            } catch (Exception e2) {
                turretMotor = null;
            }
        }

        if (turretMotor == null) {
            telemetry.addData("ERROR", "Turret Motor not found in Robot Configuration! (Name it 'turret' or 'turretMotor')");
            telemetry.update();
            sleep(3000);
            return;
        }

        turretMotor.setDirection(REVERSE_MOTOR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // Reset and prepare encoder
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial target position to zero
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("=== Turret 312 RPM System Initialized ===");
        telemetry.addData("Motor Pinion Teeth", MOTOR_TEETH);
        telemetry.addData("Turret Main Gear Teeth", TURRET_TEETH);
        telemetry.addData("Gear Ratio", "%.4f (%.2f:1)", getGearRatio(), getGearRatio());
        telemetry.addData("Ticks per Turret Degree", "%.5f", getTicksPerDegree());
        telemetry.addLine("\nControls:");
        telemetry.addLine("  Right Stick X: Manual Jogging [-90°, +90°]");
        telemetry.addLine("  D-Pad: Presets (Up: 0°, Right: +90°, Left: -90°, Down: 0°)");
        telemetry.addLine("  Y/X/B/A: Presets (+45°, -45°, +90°, -90°)");
        telemetry.addLine("  Options/Back: Reset Zero Calibration to Current Position");
        telemetry.update();

        waitForStart();

        long lastLoopTimeNanos = System.nanoTime();

        while (opModeIsActive()) {
            long currentLoopTimeNanos = System.nanoTime();
            double deltaTimeSec = (currentLoopTimeNanos - lastLoopTimeNanos) / 1e9;
            lastLoopTimeNanos = currentLoopTimeNanos;

            // Calculate conversion parameters dynamically (in case tuned via Dashboard)
            double ticksPerDegree = getTicksPerDegree();

            // --- 1. RESET ZERO CALIBRATION ---
            boolean currentOptions = gamepad1.options || gamepad1.back || gamepad2.options || gamepad2.back;
            if (currentOptions && !lastOptions) {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                targetAngleDegrees = 0.0;
                turretMotor.setTargetPosition(0);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            lastOptions = currentOptions;

            // --- 2. GAMEPAD PRESETS (Target Angles [-90° .. +90°]) ---
            if (gamepad1.dpad_up || gamepad2.dpad_up || gamepad1.dpad_down || gamepad2.dpad_down) {
                targetAngleDegrees = 0.0; // Initializing position / Center
                isManualMode = false;
            } else if (gamepad1.dpad_right || gamepad2.dpad_right || gamepad1.b || gamepad2.b) {
                targetAngleDegrees = 90.0; // Max Right (+90°)
                isManualMode = false;
            } else if (gamepad1.dpad_left || gamepad2.dpad_left || gamepad1.a || gamepad2.a) {
                targetAngleDegrees = -90.0; // Max Left (-90°)
                isManualMode = false;
            } else if (gamepad1.y || gamepad2.y) {
                targetAngleDegrees = 45.0; // Halfway Right (+45°)
                isManualMode = false;
            } else if (gamepad1.x || gamepad2.x) {
                targetAngleDegrees = -45.0; // Halfway Left (-45°)
                isManualMode = false;
            }

            // --- 3. MANUAL JOYSTICK CONTROL ---
            double stickInput = Math.abs(gamepad1.right_stick_x) > 0.05 ? gamepad1.right_stick_x : gamepad2.right_stick_x;
            if (Math.abs(stickInput) > 0.05) {
                isManualMode = true;
                // Move target position based on joystick deflection & loop time delta
                targetAngleDegrees += stickInput * MANUAL_ROTATION_SPEED_DEG_PER_SEC * deltaTimeSec;
            }

            // --- 4. APPLY SOFT LIMITS ---
            targetAngleDegrees = Math.max(TURRET_MIN_ANGLE_DEG, Math.min(TURRET_MAX_ANGLE_DEG, targetAngleDegrees));

            // --- 5. UPDATE MOTOR TARGET POSITION & VELOCITY LIMITS ---
            int targetTicks = (int) Math.round(targetAngleDegrees * ticksPerDegree);
            turretMotor.setTargetPosition(targetTicks);

            if (turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // Limit both power and velocity for slow, controlled movement
            double maxVelocityTicksPerSec = MAX_TURRET_VELOCITY_DEG_PER_SEC * ticksPerDegree;
            turretMotor.setVelocity(maxVelocityTicksPerSec);
            turretMotor.setPower(MAX_TURRET_POWER);

            // --- 6. TELEMETRY & CALCULATIONS ---
            int currentTicks = turretMotor.getCurrentPosition();
            double currentAngleDegrees = currentTicks / ticksPerDegree;
            double errorDegrees = targetAngleDegrees - currentAngleDegrees;
            double currentVelocityTicksPerSec = turretMotor.getVelocity();
            double currentMotorRPM = (currentVelocityTicksPerSec / GOBILDA_312_RPM_TICKS_PER_REV) * 60.0;
            double currentTurretRPM = currentMotorRPM / getGearRatio();

            telemetry.addData("Mode", isManualMode ? "MANUAL (Stick)" : "TARGET (Preset)");
            telemetry.addData("Target Angle (Deg)", "%.2f°", targetAngleDegrees);
            telemetry.addData("Current Angle (Deg)", "%.2f°", currentAngleDegrees);
            telemetry.addData("Angle Error (Deg)", "%.2f°", errorDegrees);
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Current Ticks", currentTicks);
            telemetry.addData("Motor Speed (RPM)", "%.1f RPM", currentMotorRPM);
            telemetry.addData("Turret Speed (RPM)", "%.1f RPM", currentTurretRPM);
            telemetry.addData("Motor Current (Amps)", "%.2f A", turretMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Soft Limits Range", "[%.0f°, %.0f°]", TURRET_MIN_ANGLE_DEG, TURRET_MAX_ANGLE_DEG);

            telemetry.update();
        }

        // Safety shutdown
        turretMotor.setPower(0);
    }

    /**
     * Calculates the overall gear ratio between turret main gear and motor pinion gear.
     * Ratio = Turret Gear Teeth / Motor Pinion Teeth
     */
    public static double getGearRatio() {
        return TURRET_TEETH / MOTOR_TEETH;
    }

    /**
     * Calculates total encoder ticks per revolution of the turret.
     */
    public static double getTicksPerTurretRev() {
        return GOBILDA_312_RPM_TICKS_PER_REV * getGearRatio();
    }

    /**
     * Calculates encoder ticks per degree of turret rotation.
     */
    public static double getTicksPerDegree() {
        return getTicksPerTurretRev() / 360.0;
    }
}

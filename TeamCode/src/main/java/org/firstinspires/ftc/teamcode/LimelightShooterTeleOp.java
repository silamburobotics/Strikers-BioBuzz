package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * TeleOp OpMode combining Limelight 3A AprilTag vision tracking with
 * a 312 RPM GoBILDA gear turret (-90° to +90° limits), flywheel velocity control,
 * and adjustable hood servo trajectory angle.
 */
@TeleOp(name = "Limelight 3A AprilTag Shooter Control", group = "TeleOp")
public class LimelightShooterTeleOp extends LinearOpMode {

    // =========================================================================
    // LIMELIGHT & TARGET CONSTANTS
    // =========================================================================
    private static final int APRILTAG_PIPELINE_INDEX = 0; // Pipeline configured for AprilTags in Limelight web UI
    private static final double LIMELIGHT_LENS_HEIGHT_INCHES = 12.0; // Height of Limelight camera lens off floor
    private static final double TARGET_CENTER_HEIGHT_INCHES = 36.0; // Height of AprilTag center off floor
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 20.0;     // Upward mount angle of Limelight relative to horizontal

    private static final int DESIRED_TAG_ID = -1; // -1 targets any detected AprilTag
    private static final double AIM_TOLERANCE_DEG = 1.0; // Degrees of error considered "aimed"

    // =========================================================================
    // TURRET GEAR & MOTOR CONSTANTS (GoBILDA 312 RPM 5202/5203 Series)
    // =========================================================================
    public static double MOTOR_TEETH = 77.0;
    public static double TURRET_TEETH = 175.0;
    public static double GOBILDA_312_RPM_TICKS_PER_REV = 537.7;

    public static double TURRET_MIN_ANGLE_DEG = -90.0; // Soft limit minimum (-90°)
    public static double TURRET_MAX_ANGLE_DEG = 90.0;  // Soft limit maximum (+90°)
    public static double MAX_TURRET_POWER = 0.50;      // Turret motor power cap (50%)
    public static double MAX_TURRET_VELOCITY_DEG_PER_SEC = 90.0; // Speed cap (90°/sec = 1.0s for full 90° turn)
    public static double TURRET_TRACKING_GAIN = 0.85;   // Proportional gain for Limelight tracking
    public static boolean REVERSE_TURRET_MOTOR = false; // Set to true if motor direction needs inversion

    // =========================================================================
    // FLYWHEEL & HOOD CONSTANTS
    // =========================================================================
    private static final double FLYWHEEL_TICKS_PER_REV = 28.0; // Bare motor / 1:1 Yellowjacket encoder ticks
    private static final double MIN_FLYWHEEL_RPM = 2000.0;
    private static final double MAX_FLYWHEEL_RPM = 4500.0;

    private static final double HOOD_MIN_POS = 0.2; // Position for close range
    private static final double HOOD_MAX_POS = 0.8; // Position for far range

    // Hardware Devices
    private Limelight3A limelight;
    private DcMotorEx turretMotor;
    private DcMotorEx flywheelMotor1;
    private DcMotorEx flywheelMotor2;
    private Servo hoodServo;
    private DcMotorEx intakeMotor;

    // State Variables
    private double targetTurretAngleDeg = 0.0;
    private boolean autoAimEnabled = true;
    private boolean flywheelEnabled = false;

    private boolean lastGamepad1A = false;
    private boolean lastGamepad1Y = false;
    private boolean lastOptions = false;

    @Override
    public void runOpMode() {
        // Stream telemetry to both Driver Station and FTC Dashboard (http://192.168.43.1:8080/dash)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---------------------------------------------------------------------
        // 1. HARDWARE MAPPING
        // ---------------------------------------------------------------------
        // Limelight 3A Mapping
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception ignored) {
            limelight = null;
        }

        if (limelight == null) {
            List<Limelight3A> limelights = hardwareMap.getAll(Limelight3A.class);
            if (limelights != null && !limelights.isEmpty()) {
                limelight = limelights.get(0);
            }
        }

        // Turret Motor Mapping
        try {
            turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
            turretMotor.setDirection(REVERSE_TURRET_MOTOR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setTargetPosition(0);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } catch (Exception e) {
            turretMotor = null;
        }

        // Flywheel Motors Mapping
        try {
            flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
            flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");

            flywheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            flywheelMotor1 = null;
            flywheelMotor2 = null;
        }

        // Hood Servo Mapping
        try {
            hoodServo = hardwareMap.get(Servo.class, "axonServo2");
        } catch (Exception e) {
            hoodServo = null;
        }

        // Intake Motor Mapping
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            intakeMotor = null;
        }

        // Configure Limelight
        if (limelight != null) {
            limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
            limelight.start();
        }

        telemetry.addLine("=== Limelight 3A & Turret System Initialized ===");
        telemetry.addData("Turret Gear Ratio", "%.4f (175:77)", getGearRatio());
        telemetry.addData("Ticks/Degree", "%.4f", getTicksPerDegree());
        telemetry.addData("Turret Limits", "[%.0f°, %.0f°]", TURRET_MIN_ANGLE_DEG, TURRET_MAX_ANGLE_DEG);
        telemetry.addData("Controls", "Y: Flywheels | A: Auto-Aim | Options: Reset Turret 0°");
        telemetry.update();

        waitForStart();

        long lastLoopTimeNanos = System.nanoTime();

        while (opModeIsActive()) {
            long currentLoopTimeNanos = System.nanoTime();
            double deltaTimeSec = (currentLoopTimeNanos - lastLoopTimeNanos) / 1e9;
            lastLoopTimeNanos = currentLoopTimeNanos;

            double ticksPerDegree = getTicksPerDegree();

            // -----------------------------------------------------------------
            // 2. GAMEPAD INPUTS & ZERO RESET
            // -----------------------------------------------------------------
            // Zero Calibration Reset (Options / Back Button)
            boolean currentOptions = gamepad1.options || gamepad1.back || gamepad2.options || gamepad2.back;
            if (currentOptions && !lastOptions) {
                if (turretMotor != null) {
                    turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    targetTurretAngleDeg = 0.0;
                    turretMotor.setTargetPosition(0);
                    turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            lastOptions = currentOptions;

            // Toggle Flywheel state with 'Y' button
            if (gamepad1.y && !lastGamepad1Y) flywheelEnabled = !flywheelEnabled;
            lastGamepad1Y = gamepad1.y;

            // Toggle Auto-Aim with 'A' button
            if (gamepad1.a && !lastGamepad1A) autoAimEnabled = !autoAimEnabled;
            lastGamepad1A = gamepad1.a;

            // -----------------------------------------------------------------
            // 3. LIMELIGHT 3A APRILTAG DETECTION
            // -----------------------------------------------------------------
            LLResult result = (limelight != null) ? limelight.getLatestResult() : null;

            boolean targetVisible = false;
            double tx = 0.0;
            double ty = 0.0;
            double distanceInches = 0.0;
            int detectedTagId = -1;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        int fid = fiducial.getFiducialId();
                        if (DESIRED_TAG_ID < 0 || fid == DESIRED_TAG_ID) {
                            targetVisible = true;
                            detectedTagId = fid;
                            tx = fiducial.getTargetXDegrees();
                            ty = fiducial.getTargetYDegrees();
                            break;
                        }
                    }
                }

                if (!targetVisible && result.getTa() > 0) {
                    targetVisible = true;
                    tx = result.getTx();
                    ty = result.getTy();
                }

                if (targetVisible) {
                    double angleToTargetRad = Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEG + ty);
                    double heightDifference = TARGET_CENTER_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES;

                    if (Math.tan(angleToTargetRad) > 0) {
                        distanceInches = heightDifference / Math.tan(angleToTargetRad);
                    }
                }
            }

            // -----------------------------------------------------------------
            // 4. AUTOMATIC TURRET TRACKING & SOFT LIMITS [-90°, +90°]
            // -----------------------------------------------------------------
            int currentTurretTicks = (turretMotor != null) ? turretMotor.getCurrentPosition() : 0;
            double currentTurretAngleDeg = currentTurretTicks / ticksPerDegree;
            boolean isAimed = false;

            if (targetVisible && autoAimEnabled) {
                // Adjust turret angle based on Limelight horizontal offset (tx)
                // Inverting sign (-tx) so turret rotates TOWARDS the AprilTag center
                double desiredTurretAngle = currentTurretAngleDeg - (tx * TURRET_TRACKING_GAIN);
                targetTurretAngleDeg = desiredTurretAngle;
                isAimed = Math.abs(tx) <= AIM_TOLERANCE_DEG;
            } else {
                // Manual Jogging with Right Stick X if Auto-Aim is disabled or no target visible
                double stickInput = Math.abs(gamepad1.right_stick_x) > 0.05 ? gamepad1.right_stick_x : gamepad2.right_stick_x;
                if (Math.abs(stickInput) > 0.05) {
                    targetTurretAngleDeg += stickInput * 45.0 * deltaTimeSec;
                } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                    targetTurretAngleDeg = 0.0; // Center preset
                }
            }

            // Strictly enforce -90° to +90° soft limits
            targetTurretAngleDeg = Range.clip(targetTurretAngleDeg, TURRET_MIN_ANGLE_DEG, TURRET_MAX_ANGLE_DEG);

            // Update Turret Motor Position & Limits
            if (turretMotor != null) {
                int targetTicks = (int) Math.round(targetTurretAngleDeg * ticksPerDegree);
                turretMotor.setTargetPosition(targetTicks);

                if (turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                double maxVelocityTicksPerSec = MAX_TURRET_VELOCITY_DEG_PER_SEC * ticksPerDegree;
                turretMotor.setVelocity(maxVelocityTicksPerSec);
                turretMotor.setPower(MAX_TURRET_POWER);
            }

            // -----------------------------------------------------------------
            // 5. SHOOTER FLYWHEEL & HOOD ADJUSTMENT
            // -----------------------------------------------------------------
            double targetFlywheelRPM = 3000.0;
            double targetHoodPosition = 0.5;

            if (targetVisible && autoAimEnabled) {
                targetFlywheelRPM = calculateRPMForDistance(distanceInches);
                targetHoodPosition = calculateHoodForDistance(distanceInches);
            }

            if (hoodServo != null) {
                hoodServo.setPosition(targetHoodPosition);
            }

            if (flywheelMotor1 != null && flywheelMotor2 != null) {
                if (flywheelEnabled) {
                    double targetTicksPerSec = (targetFlywheelRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;
                    flywheelMotor1.setVelocity(targetTicksPerSec);
                    flywheelMotor2.setVelocity(targetTicksPerSec);
                } else {
                    flywheelMotor1.setVelocity(0);
                    flywheelMotor2.setVelocity(0);
                }
            }

            if (intakeMotor != null) {
                intakeMotor.setPower(gamepad1.right_trigger);
            }

            // -----------------------------------------------------------------
            // 6. TELEMETRY
            // -----------------------------------------------------------------
            double turretErrorDeg = targetTurretAngleDeg - currentTurretAngleDeg;
            double currentRPM1 = (flywheelMotor1 != null) ? (flywheelMotor1.getVelocity() / FLYWHEEL_TICKS_PER_REV) * 60.0 : 0.0;
            double currentRPM2 = (flywheelMotor2 != null) ? (flywheelMotor2.getVelocity() / FLYWHEEL_TICKS_PER_REV) * 60.0 : 0.0;

            telemetry.addData("=== Vision Target ===", targetVisible ? "LOCKED (Tag " + detectedTagId + ")" : "SEARCHING");
            if (targetVisible) {
                telemetry.addData("Limelight tx", "%.2f°", tx);
                telemetry.addData("Limelight ty", "%.2f°", ty);
                telemetry.addData("Calculated Distance", "%.1f in", distanceInches);
                telemetry.addData("Aim Locked", isAimed ? "YES" : "AIMING...");
            }

            telemetry.addData("=== Turret Status ===", autoAimEnabled ? "AUTO-TRACKING" : "MANUAL");
            telemetry.addData("Target Angle", "%.2f°", targetTurretAngleDeg);
            telemetry.addData("Current Angle", "%.2f°", currentTurretAngleDeg);
            telemetry.addData("Angle Error", "%.2f°", turretErrorDeg);
            telemetry.addData("Turret Range", "[%.0f°, %.0f°]", TURRET_MIN_ANGLE_DEG, TURRET_MAX_ANGLE_DEG);

            telemetry.addData("=== Shooter Status ===", flywheelEnabled ? "ACTIVE" : "DISABLED");
            telemetry.addData("Target RPM", "%.0f", targetFlywheelRPM);
            telemetry.addData("Flywheel 1 RPM", "%.0f", currentRPM1);
            telemetry.addData("Flywheel 2 RPM", "%.0f", currentRPM2);
            telemetry.addData("Target Hood Pos", "%.2f", targetHoodPosition);
            telemetry.update();
        }

        // Shutdown safely
        if (limelight != null) limelight.stop();
        if (turretMotor != null) turretMotor.setPower(0);
        if (flywheelMotor1 != null) flywheelMotor1.setVelocity(0);
        if (flywheelMotor2 != null) flywheelMotor2.setVelocity(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
    }

    private double getGearRatio() {
        return TURRET_TEETH / MOTOR_TEETH;
    }

    private double getTicksPerDegree() {
        return (GOBILDA_312_RPM_TICKS_PER_REV * getGearRatio()) / 360.0;
    }

    private double calculateRPMForDistance(double distanceInches) {
        double calculatedRPM = 2500.0 + (15.0 * distanceInches);
        return Range.clip(calculatedRPM, MIN_FLYWHEEL_RPM, MAX_FLYWHEEL_RPM);
    }

    private double calculateHoodForDistance(double distanceInches) {
        double minDistance = 24.0;
        double maxDistance = 120.0;
        double normDistance = Range.clip((distanceInches - minDistance) / (maxDistance - minDistance), 0.0, 1.0);
        return HOOD_MIN_POS + normDistance * (HOOD_MAX_POS - HOOD_MIN_POS);
    }
}

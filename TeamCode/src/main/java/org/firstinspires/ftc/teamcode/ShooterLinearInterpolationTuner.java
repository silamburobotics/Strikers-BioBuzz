package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

/**
 * Interactive Tuning OpMode for calibrating Linear Interpolation parameters
 * (Distance vs Flywheel RPM and Hood/Shifter Position) for Pollen and Nectar ball types.
 * <p>
 * Controls:
 * - Gamepad 1 Button A: Intake FORWARD | Gamepad 1 Button B: Intake REVERSE
 * - Gamepad 2 D-Pad Up / Down: Adjust Test RPM (+/- 50 RPM; hold RT for +/- 10 RPM)
 * - Gamepad 2 D-Pad Left / Right: Adjust Shifter/Hood Position (+/- 0.02)
 * - Gamepad 2 Button A: Toggle Ball Type (Pollen vs Nectar)
 * - Gamepad 2 Button Y: Toggle Flywheel ON / OFF
 * - Gamepad 2 Button B: Open Axon Gate Servo (Test Fire!)
 * - Gamepad 2 Button X: Close Axon Gate Servo
 * - Gamepad 1 Button X: Save Current Calibration Data Point (Distance, RPM, Hood)
 */
@Config
@TeleOp(name = "Shooter Linear Interpolation Tuner", group = "Tuning")
public class ShooterLinearInterpolationTuner extends LinearOpMode {

    // Calibration Point Data Structure
    public static class CalibrationPoint {
        public double distanceInches;
        public double flywheelRPM;
        public double shifterPosition;
        public boolean isNectar;

        public CalibrationPoint(double distanceInches, double flywheelRPM, double shifterPosition, boolean isNectar) {
            this.distanceInches = distanceInches;
            this.flywheelRPM = flywheelRPM;
            this.shifterPosition = shifterPosition;
            this.isNectar = isNectar;
        }
    }

    // Configurable Tuning Parameters via Dashboard
    public static double TEST_RPM = 2500.0;
    public static double TEST_SHIFTER_POS = 0.95;
    public static boolean IS_NECTAR_MODE = false; // false = Pollen, true = Nectar
    public static boolean MOTOR_2_REVERSED = true; // Set true if motor 2 spins opposite to motor 1

    // Hardware Names
    public static String MOTOR_1_NAME = "flyWheel1";
    public static String MOTOR_2_NAME = "flyWheel2";
    public static String INTAKE_NAME = "intake";
    public static String GATE_SERVO_NAME = "gateServo";
    public static String SHIFTER_SERVO_NAME = "shifterServo";
    public static String TURRET_NAME = "turret";

    // Servo Gate Positions
    public static double GATE_OPEN_POS = 0.50;
    public static double GATE_CLOSED_POS = 0.70;

    // Hardware Devices
    private LimelightShooterSubsystem visionSubsystem;
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx intake;
    private DcMotorEx turret;
    private Servo gateServo;
    private Servo shifterServo;

    // State Variables
    private boolean flywheelEnabled = false;
    private boolean isGateOpen = false;
    private double intakePower = 0.0;

    // Edge Detection
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastGamepad2Y = false;
    private boolean lastGamepad2A = false;
    private boolean lastGamepad1A = false;
    private boolean lastGamepad1B = false;
    private boolean lastGamepad1X = false;

    // Captured Calibration Points
    private final List<CalibrationPoint> pollenPoints = new ArrayList<>();
    private final List<CalibrationPoint> nectarPoints = new ArrayList<>();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. Initialize Limelight Vision Subsystem
        visionSubsystem = new LimelightShooterSubsystem(hardwareMap);

        // 2. Initialize Shooter Motors & Intake Motor
        try {
            shooter1 = hardwareMap.get(DcMotorEx.class, MOTOR_1_NAME);
            shooter2 = hardwareMap.get(DcMotorEx.class, MOTOR_2_NAME);
            shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            shooter1 = null;
            shooter2 = null;
        }

        try {
            intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            intake = null;
        }

        // 3. Initialize Servos
        try {
            gateServo = hardwareMap.get(Servo.class, GATE_SERVO_NAME);
            gateServo.setPosition(GATE_CLOSED_POS);
        } catch (Exception e) {
            gateServo = null;
        }

        try {
            shifterServo = hardwareMap.get(Servo.class, SHIFTER_SERVO_NAME);
            shifterServo.setPosition(TEST_SHIFTER_POS);
        } catch (Exception e) {
            shifterServo = null;
        }

        // 4. Initialize Turret Motor
        try {
            turret = hardwareMap.get(DcMotorEx.class, TURRET_NAME);
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setTargetPosition(0);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(0.50);
        } catch (Exception e) {
            turret = null;
        }

        telemetry.addLine("=== Shooter Linear Interpolation Tuner Initialized ===");
        telemetry.addLine("G1.A: Intake Forward | G1.B: Intake Reverse");
        telemetry.addLine("G2.D-Pad Up/Down: Tune RPM | G2.D-Pad Left/Right: Tune Hood/Shifter");
        telemetry.addLine("G2.A: Toggle Pollen/Nectar Mode | G2.Y: Toggle Flywheel");
        telemetry.addLine("G2.B: Fire Ball (Open Gate) | G1.X: Save Calibration Point");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            visionSubsystem.update();

            // -----------------------------------------------------------------
            // 1. GAMEPAD 1 INTAKE MOTOR CONTROL
            // -----------------------------------------------------------------
            if (gamepad1.a && !lastGamepad1A) {
                intakePower = (intakePower == 1.0) ? 0.0 : 1.0;
            }
            lastGamepad1A = gamepad1.a;

            if (gamepad1.b && !lastGamepad1B) {
                intakePower = (intakePower == -1.0) ? 0.0 : -1.0;
            }
            lastGamepad1B = gamepad1.b;

            if (intake != null) {
                intake.setPower(intakePower);
            }

            // -----------------------------------------------------------------
            // 2. GAMEPAD 2 TUNING CONTROLS
            // -----------------------------------------------------------------
            // Toggle Flywheels with Gamepad 2 Y
            if (gamepad2.y && !lastGamepad2Y) {
                flywheelEnabled = !flywheelEnabled;
            }
            lastGamepad2Y = gamepad2.y;

            // Toggle Ball Type (Pollen vs Nectar) with Gamepad 2 A
            if (gamepad2.a && !lastGamepad2A) {
                IS_NECTAR_MODE = !IS_NECTAR_MODE;
                TEST_SHIFTER_POS = IS_NECTAR_MODE ? 0.25 : 0.95;
            }
            lastGamepad2A = gamepad2.a;

            // Tune Test RPM with D-Pad Up / Down (Fine adjustment with Right Trigger)
            double rpmStep = (gamepad2.right_trigger > 0.3) ? 10.0 : 50.0;
            if (gamepad2.dpad_up && !lastDpadUp) {
                TEST_RPM += rpmStep;
            }
            lastDpadUp = gamepad2.dpad_up;

            if (gamepad2.dpad_down && !lastDpadDown) {
                TEST_RPM -= rpmStep;
            }
            lastDpadDown = gamepad2.dpad_down;

            TEST_RPM = Range.clip(TEST_RPM, 1500.0, 5500.0);

            // Tune Shifter Position with D-Pad Left / Right
            if (gamepad2.dpad_right && !lastDpadRight) {
                TEST_SHIFTER_POS += 0.02;
            }
            lastDpadRight = gamepad2.dpad_right;

            if (gamepad2.dpad_left && !lastDpadLeft) {
                TEST_SHIFTER_POS -= 0.02;
            }
            lastDpadLeft = gamepad2.dpad_left;

            TEST_SHIFTER_POS = Range.clip(TEST_SHIFTER_POS, 0.0, 1.0);

            // Test Fire Gate Servo Control (Gamepad 2 B = Fire, X = Close)
            if (gamepad2.b) {
                isGateOpen = true;
            } else if (gamepad2.x) {
                isGateOpen = false;
            }

            if (gateServo != null) {
                gateServo.setPosition(isGateOpen ? GATE_OPEN_POS : GATE_CLOSED_POS);
            }

            if (shifterServo != null) {
                shifterServo.setPosition(TEST_SHIFTER_POS);
            }

            // -----------------------------------------------------------------
            // 3. TURRET APRILTAG AUTO-ALIGNMENT
            // -----------------------------------------------------------------
            double ticksPerDegree = (537.7 * 8.0) / 360.0;
            if (turret != null) {
                double currentTurretAngle = turret.getCurrentPosition() / ticksPerDegree;
                double targetAngle = visionSubsystem.getTurretTargetAngle(currentTurretAngle);
                int targetTicks = (int) Math.round(targetAngle * ticksPerDegree);
                turret.setTargetPosition(targetTicks);
                turret.setPower(0.50);
            }

            // -----------------------------------------------------------------
            // 4. MASTER ENCODER FLYWHEEL CONTROL (FORWARD DIRECTION)
            // -----------------------------------------------------------------
            if (shooter1 != null && shooter2 != null) {
                shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
                shooter2.setDirection(MOTOR_2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

                double actualTPS1 = shooter1.getVelocity();
                double actualRPM1 = (actualTPS1 * 60.0) / 28.0;

                double shooterPower = 0.0;
                if (flywheelEnabled && TEST_RPM > 0) {
                    double basePower = TEST_RPM / 6000.0;
                    double errorRPM = TEST_RPM - actualRPM1;
                    shooterPower = Range.clip(basePower + (errorRPM * 0.00035), 0.0, 1.0);
                }

                shooter1.setPower(shooterPower);
                shooter2.setPower(shooterPower);
            }

            // -----------------------------------------------------------------
            // 5. SAVE CALIBRATION POINT (Gamepad 1 Button X)
            // -----------------------------------------------------------------
            double currentDistance = visionSubsystem.getCalculatedDistanceInches();

            if (gamepad1.x && !lastGamepad1X) {
                if (visionSubsystem.isTargetVisible() && currentDistance > 0) {
                    CalibrationPoint point = new CalibrationPoint(currentDistance, TEST_RPM, TEST_SHIFTER_POS, IS_NECTAR_MODE);
                    if (IS_NECTAR_MODE) {
                        nectarPoints.add(point);
                    } else {
                        pollenPoints.add(point);
                    }
                }
            }
            lastGamepad1X = gamepad1.x;

            // -----------------------------------------------------------------
            // 6. TELEMETRY & LINEAR REGRESSION STATS
            // -----------------------------------------------------------------
            telemetry.addData("=== Mode ===", IS_NECTAR_MODE ? "NECTAR (Bigger Ball)" : "POLLEN (Smaller Ball)");
            telemetry.addData("Intake Motor (G1.A/B)", "Power: %.2f", intakePower);
            telemetry.addData("Limelight Target", visionSubsystem.isTargetVisible() ?
                    String.format("LOCKED (Tag #%d | Dist: %.1f in)", visionSubsystem.getDetectedTagId(), currentDistance) : "SEARCHING...");
            telemetry.addData("Target Lock", visionSubsystem.isAimed() ? "READY TO SHOOT!" : "ALIGNING...");

            telemetry.addData("=== Current Tuning Setup ===", "");
            telemetry.addData("Test RPM (D-Pad Up/Down)", "%.0f RPM (Motor ON: %s)", TEST_RPM, flywheelEnabled ? "YES" : "NO");
            telemetry.addData("Test Shifter Pos (D-Pad L/R)", "%.2f", TEST_SHIFTER_POS);

            if (shooter1 != null) {
                double actualRPM = (shooter1.getVelocity() * 60.0) / 28.0;
                telemetry.addData("Actual Flywheel RPM", "%.0f RPM", actualRPM);
            }

            telemetry.addData("=== Saved Data Points ===", "Pollen: %d | Nectar: %d", pollenPoints.size(), nectarPoints.size());

            // Compute Linear Fit Equation if 2+ points captured
            if (pollenPoints.size() >= 2) {
                LinearFit fit = calculateLinearFit(pollenPoints);
                telemetry.addData("Pollen Curve", "RPM = %.1f + (%.2f * distance)", fit.intercept, fit.slope);
            }
            if (nectarPoints.size() >= 2) {
                LinearFit fit = calculateLinearFit(nectarPoints);
                telemetry.addData("Nectar Curve", "RPM = %.1f + (%.2f * distance)", fit.intercept, fit.slope);
            }

            telemetry.update();
        }

        visionSubsystem.stop();
        if (shooter1 != null) shooter1.setPower(0);
        if (shooter2 != null) shooter2.setPower(0);
        if (intake != null) intake.setPower(0);
    }

    // Helper Linear Regression Struct
    public static class LinearFit {
        public double slope;
        public double intercept;

        public LinearFit(double slope, double intercept) {
            this.slope = slope;
            this.intercept = intercept;
        }
    }

    /**
     * Calculates Linear Regression Slope and Intercept from captured calibration points.
     */
    private LinearFit calculateLinearFit(List<CalibrationPoint> points) {
        int n = points.size();
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

        for (CalibrationPoint p : points) {
            sumX += p.distanceInches;
            sumY += p.flywheelRPM;
            sumXY += p.distanceInches * p.flywheelRPM;
            sumX2 += p.distanceInches * p.distanceInches;
        }

        double denominator = (n * sumX2 - sumX * sumX);
        if (Math.abs(denominator) < 1e-6) return new LinearFit(0, 0);

        double slope = (n * sumXY - sumX * sumY) / denominator;
        double intercept = (sumY - slope * sumX) / n;

        return new LinearFit(slope, intercept);
    }
}

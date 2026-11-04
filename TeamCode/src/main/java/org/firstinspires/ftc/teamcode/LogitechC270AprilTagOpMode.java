package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * TeleOp OpMode for Logitech C270 USB Webcam using FTC VisionPortal & AprilTagProcessor.
 * Detects AprilTags, calculates target distance/bearing, adjusts flywheel RPM and hood angle,
 * and streams live telemetry to both the Driver Station and FTC Dashboard (http://192.168.43.1:8080/dash).
 */
@TeleOp(name = "Logitech C270 AprilTag Shooter", group = "Vision")
public class LogitechC270AprilTagOpMode extends LinearOpMode {

    // =========================================================================
    // CONSTANTS & SETTINGS
    // =========================================================================
    private static final int DESIRED_TAG_ID = -1; // -1 to target any tag, or specific ID like 1, 2, 3...
    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    private static final double MIN_FLYWHEEL_RPM = 2000.0;
    private static final double MAX_FLYWHEEL_RPM = 4500.0;

    private static final double HOOD_MIN_POS = 0.2; // Close range
    private static final double HOOD_MAX_POS = 0.8; // Far range

    // Hardware
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private DcMotorEx flywheelMotor1;
    private DcMotorEx flywheelMotor2;
    private Servo hoodServo;
    private DcMotorEx intakeMotor;

    // Controls
    private boolean flywheelEnabled = false;
    private boolean autoAimEnabled = true;
    private boolean lastGamepad1Y = false;
    private boolean lastGamepad1A = false;

    @Override
    public void runOpMode() {
        // Send telemetry to Driver Station AND FTC Dashboard (http://192.168.43.1:8080/dash)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. Initialize Logitech C270 Vision Portal
        initLogitechC270();

        // 2. Initialize Motors & Servos (Safely handled if not present in config)
        initHardware();

        telemetry.addData("Status", "Logitech C270 & Hardware Initialized.");
        telemetry.addData("FTC Dashboard URL", "http://192.168.43.1:8080/dash");
        telemetry.addData("Controls", "Y: Toggle Flywheels | A: Toggle Auto-Aim | Trigger: Intake");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Gamepad toggles
            if (gamepad1.y && !lastGamepad1Y) flywheelEnabled = !flywheelEnabled;
            lastGamepad1Y = gamepad1.y;

            if (gamepad1.a && !lastGamepad1A) autoAimEnabled = !autoAimEnabled;
            lastGamepad1A = gamepad1.a;

            // Stream controls
            if (gamepad1.dpad_down && visionPortal.getProcessorEnabled(aprilTagProcessor)) {
                visionPortal.setProcessorEnabled(aprilTagProcessor, false);
            } else if (gamepad1.dpad_up && !visionPortal.getProcessorEnabled(aprilTagProcessor)) {
                visionPortal.setProcessorEnabled(aprilTagProcessor, true);
            }

            // -----------------------------------------------------------------
            // APRILTAG DETECTION & POSE ESTIMATION
            // -----------------------------------------------------------------
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            AprilTagSingleDetection targetDetection = null;

            if (currentDetections != null) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection instanceof AprilTagSingleDetection) {
                        AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                        if (singleDet.metadata != null) {
                            if (DESIRED_TAG_ID < 0 || singleDet.id == DESIRED_TAG_ID) {
                                targetDetection = singleDet;
                                break;
                            }
                        }
                    }
                }
            }

            // -----------------------------------------------------------------
            // SHOOTER CONTROL & AUTOMATION
            // -----------------------------------------------------------------
            double targetFlywheelRPM = 3000.0;
            double targetHoodPosition = 0.5;

            if (targetDetection != null && targetDetection.ftcPose != null) {
                double rangeInches = targetDetection.ftcPose.range;     // Distance to target in inches
                double bearingDeg = targetDetection.ftcPose.bearing;    // Horizontal angle in degrees
                double elevationDeg = targetDetection.ftcPose.elevation;// Elevation angle in degrees

                telemetry.addData("=== Target Locked ===", "AprilTag ID %d (%s)", targetDetection.id, targetDetection.metadata.name);
                telemetry.addData("Range (Distance)", "%.1f inches", rangeInches);
                telemetry.addData("Bearing (Angle)", "%.1f deg", bearingDeg);
                telemetry.addData("Elevation", "%.1f deg", elevationDeg);
                telemetry.addData("Pose (XYZ)", "X: %.1f, Y: %.1f, Z: %.1f in",
                        targetDetection.ftcPose.x, targetDetection.ftcPose.y, targetDetection.ftcPose.z);

                if (autoAimEnabled) {
                    targetFlywheelRPM = calculateRPMForDistance(rangeInches);
                    targetHoodPosition = calculateHoodForDistance(rangeInches);
                }
            } else {
                telemetry.addData("=== Target Status ===", "SEARCHING FOR APRILTAG...");
            }

            // Apply Servo & Motor Outputs
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

            // Telemetry Output
            double currentRPM1 = (flywheelMotor1 != null) ? (flywheelMotor1.getVelocity() / FLYWHEEL_TICKS_PER_REV) * 60.0 : 0.0;
            double currentRPM2 = (flywheelMotor2 != null) ? (flywheelMotor2.getVelocity() / FLYWHEEL_TICKS_PER_REV) * 60.0 : 0.0;

            telemetry.addData("=== Shooter Status ===", flywheelEnabled ? "ACTIVE" : "DISABLED");
            telemetry.addData("Target RPM", "%.0f", targetFlywheelRPM);
            telemetry.addData("Flywheel 1 RPM", "%.0f", currentRPM1);
            telemetry.addData("Flywheel 2 RPM", "%.0f", currentRPM2);
            telemetry.addData("Hood Servo Pos", "%.2f", targetHoodPosition);
            telemetry.addData("Auto-Aim Mode (A)", autoAimEnabled ? "ON" : "OFF");
            telemetry.update();
        }

        // Close VisionPortal & Stop Motors on exit
        if (visionPortal != null) {
            visionPortal.close();
        }
        if (flywheelMotor1 != null) flywheelMotor1.setVelocity(0);
        if (flywheelMotor2 != null) flywheelMotor2.setVelocity(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
    }

    /**
     * Initializes the AprilTag Processor and Logitech C270 Webcam VisionPortal.
     */
    private void initLogitechC270() {
        // Build AprilTag Processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Get Webcam Name from Hardware Map (default "Webcam 1" or first available webcam)
        WebcamName webcamName;
        try {
            webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        } catch (Exception e) {
            List<WebcamName> webcams = hardwareMap.getAll(WebcamName.class);
            if (webcams != null && !webcams.isEmpty()) {
                webcamName = webcams.get(0);
            } else {
                throw new RuntimeException("Logitech C270 Webcam not found in Robot Configuration! Name it 'Webcam 1'.");
            }
        }

        // Build VisionPortal for Logitech C270 (640x480 resolution)
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .setCameraResolution(new Size(640, 480)) // Native 720p/480p resolution for Logitech C270
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }

    /**
     * Safe initialization for shooter motors and servos.
     */
    private void initHardware() {
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

        try {
            hoodServo = hardwareMap.get(Servo.class, "axonServo2");
        } catch (Exception e) {
            hoodServo = null;
        }

        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            intakeMotor = null;
        }
    }

    private double calculateRPMForDistance(double distanceInches) {
        double calculatedRPM = 2500.0 + (15.0 * distanceInches);
        return Range.clip(calculatedRPM, MIN_FLYWHEEL_RPM, MAX_FLYWHEEL_RPM);
    }

    private double calculateHoodForDistance(double distanceInches) {
        double normDistance = Range.clip((distanceInches - 24.0) / (120.0 - 24.0), 0.0, 1.0);
        return HOOD_MIN_POS + normDistance * (HOOD_MAX_POS - HOOD_MIN_POS);
    }
}

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * TeleOp OpMode for 3D AprilTag Field Localization and Position Tracking.
 * Based on ConceptAprilTagLocalization sample for TeamCode.
 * Calculates 3D Robot Field Position (XYZ) and Orientation (Yaw/Pitch/Roll)
 * using C270/USB Webcam and streams live data to Driver Station and FTC Dashboard (http://192.168.43.1:8080/dash).
 */
@Config
@TeleOp(name = "AprilTag Field Localization", group = "Vision")
public class AprilTagLocalizationOpMode extends LinearOpMode {

    // --- CAMERA MOUNTING OFFSETS RELATIVE TO ROBOT CENTER ---
    // Position: X (right/left inches), Y (forward/backward inches), Z (height above ground inches)
    public static double CAMERA_X_OFFSET_INCHES = 0.0;    // 0 = centered horizontally
    public static double CAMERA_Y_OFFSET_INCHES = 6.0;    // 6.0 = mounted 6 inches forward from center
    public static double CAMERA_Z_OFFSET_INCHES = 12.0;   // 12.0 = mounted 12 inches above ground

    // Orientation Angles (Degrees): Yaw (horizontal angle), Pitch (tilt angle), Roll
    public static double CAMERA_YAW_DEG = 0.0;            // 0 = pointing straight forward
    public static double CAMERA_PITCH_DEG = -90.0;        // -90 = camera mounted horizontally
    public static double CAMERA_ROLL_DEG = 0.0;           // 0 = upright camera mounting

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Send telemetry to Driver Station AND FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize AprilTag Processor & VisionPortal
        initAprilTag();

        telemetry.addLine("=== AprilTag 3D Field Localization System ===");
        telemetry.addData("FTC Dashboard URL", "http://192.168.43.1:8080/dash");
        telemetry.addLine("Touch START to begin position tracking...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Process AprilTag detections and display 3D localization telemetry
            telemetryAprilTagLocalization();

            telemetry.update();

            // Toggle camera stream on D-Pad
            if (gamepad1.dpad_down && visionPortal.getProcessorEnabled(aprilTagProcessor)) {
                visionPortal.setProcessorEnabled(aprilTagProcessor, false);
            } else if (gamepad1.dpad_up && !visionPortal.getProcessorEnabled(aprilTagProcessor)) {
                visionPortal.setProcessorEnabled(aprilTagProcessor, true);
            }

            sleep(20);
        }

        // Close VisionPortal on exit
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Initializes AprilTagProcessor with camera mounting pose and VisionPortal.
     */
    private void initAprilTag() {
        // Define camera position and orientation relative to robot origin
        Position cameraPosition = new Position(DistanceUnit.INCH,
                CAMERA_X_OFFSET_INCHES, CAMERA_Y_OFFSET_INCHES, CAMERA_Z_OFFSET_INCHES, 0);

        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                CAMERA_YAW_DEG, CAMERA_PITCH_DEG, CAMERA_ROLL_DEG, 0);

        // Build AprilTag Processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        // Get Webcam Name from Hardware Map
        WebcamName webcamName;
        try {
            webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        } catch (Exception e) {
            List<WebcamName> webcams = hardwareMap.getAll(WebcamName.class);
            if (webcams != null && !webcams.isEmpty()) {
                webcamName = webcams.get(0);
            } else {
                throw new RuntimeException("Webcam not found in Robot Configuration! Name it 'Webcam 1'.");
            }
        }

        // Build VisionPortal for 640x480 resolution
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        try {
            FtcDashboard.getInstance().startCameraStream(visionPortal, 15);
        } catch (Exception ignored) {}
    }

    /**
     * Extracts and displays 3D field location and relative target pose.
     */
    private void telemetryAprilTagLocalization() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("AprilTags in View", currentDetections.size());

        if (currentDetections == null || currentDetections.isEmpty()) {
            telemetry.addLine("[SEARCHING] No AprilTags in camera view.");
            return;
        }

        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                String tagName = (singleDet.metadata != null && singleDet.metadata.name != null)
                        ? singleDet.metadata.name : "AprilTag";

                telemetry.addLine(String.format("\n==== Tag #%d (%s) ====", singleDet.id, tagName));

                if (detection.ftcPose != null) {
                    telemetry.addLine(String.format("Target Relative: Range %.1f in | Bearing %.1f° | Elev %.1f°",
                            detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }

                if (detection.robotPose != null && detection.robotPose.getPosition() != null) {
                    Position pos = detection.robotPose.getPosition();
                    YawPitchRollAngles orient = detection.robotPose.getOrientation();

                    telemetry.addLine(String.format("Robot Field Position: X: %.1f | Y: %.1f | Z: %.1f in",
                            pos.x, pos.y, pos.z));
                    if (orient != null) {
                        telemetry.addLine(String.format("Robot Orientation: Pitch: %.1f° | Roll: %.1f° | Yaw: %.1f°",
                                orient.getPitch(AngleUnit.DEGREES),
                                orient.getRoll(AngleUnit.DEGREES),
                                orient.getYaw(AngleUnit.DEGREES)));
                    }
                } else {
                    if (singleDet.center != null) {
                        telemetry.addLine(String.format("Pixel Center: (%.0f, %.0f) px", singleDet.center.x, singleDet.center.y));
                    }
                }
            }
        }
    }
}

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
 * Unified Vision Subsystem supporting both Limelight 3A and Logitech C270 Webcams.
 * Automatically detects whichever camera is connected to the robot.
 * Provides AprilTag tracking, alliance filtering, distance calculation,
 * turret auto-aim angles, dynamic flywheel RPMs, and hood/shifter servo positioning.
 */
@Config
public class LimelightShooterSubsystem {

    public enum CameraType {
        NONE,
        LIMELIGHT_3A,
        LOGITECH_C270
    }

    public static String CAMERA_SELECTION = "AUTO"; // "AUTO" = Auto-detect based on connected hardware, or force "LIMELIGHT" / "C270"

    // Configuration Constants
    public static int APRILTAG_PIPELINE_INDEX = 0;
    public static double LIMELIGHT_LENS_HEIGHT_INCHES = 12.0;
    public static double TARGET_CENTER_HEIGHT_INCHES = 36.0;
    public static double LIMELIGHT_MOUNT_ANGLE_DEG = 45.0;     // 45 degree upward camera mounting pitch angle
    public static int DESIRED_TAG_ID = -1;                 // -1 targets any AprilTag

    public static double TURRET_AUTO_AIM_GAIN = 1.0;       // 1.0 = direct 1:1 target angle calculation
    public static double TURRET_AIM_OFFSET_DEG = 0.0;      // Horizontal offset angle (in degrees) from AprilTag center to target basket
    public static double AIM_DEADBAND_DEG = 0.75;          // Deadband tolerance to prevent hunting/oscillations
    public static double TURRET_MIN_ANGLE_DEG = -90.0;
    public static double TURRET_MAX_ANGLE_DEG = 90.0;

    // Ball Type Flywheel RPM Curves (Pollen = Smaller Ball, Nectar = Bigger Ball)
    public static double POLLEN_BASE_RPM = 2150.0;         // Base RPM for Pollen (Smaller ball)
    public static double POLLEN_RPM_SLOPE = 6.0;           // RPM increase per inch of distance (6 RPM/inch)
    public static double NECTAR_BASE_RPM = 2320.0;         // Base RPM for Nectar (Bigger ball)
    public static double NECTAR_RPM_SLOPE = 6.0;           // RPM increase per inch of distance (6 RPM/inch)

    public static double FLYWHEEL_MIN_RPM = 2000.0;
    public static double FLYWHEEL_MAX_RPM = 4500.0;

    public static double SHIFTER_OPEN_POSITION = 0.25;
    public static double SHIFTER_CLOSED_POSITION = 0.95;

    // Alliance AprilTag Filtering Constants
    // Blue Alliance Tags: Left (39, 40), Right (43, 44)
    // Red Alliance Tags: Left (31, 32), Right (35, 36)
    public static final int[] BLUE_TAG_IDS = {39, 40, 43, 44};
    public static final int[] RED_TAG_IDS = {31, 32, 35, 36};

    // Camera Hardware & Types
    private CameraType activeCameraType = CameraType.NONE;
    private Limelight3A limelight;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // State Variables
    private boolean targetVisible = false;
    private double tx = 0.0;
    private double ty = 0.0;
    private double calculatedDistanceInches = 0.0;
    private int detectedTagId = -1;

    // Target Lock-On State
    private int lockedTagId = -1;
    private boolean targetLocked = false;
    private int framesSinceTagSeen = 0;
    private static final int MAX_PERSISTENCE_FRAMES = 10; // Persistence during brief vision occlusion (~300ms)

    public LimelightShooterSubsystem(HardwareMap hardwareMap) {
        activeCameraType = CameraType.NONE;

        boolean allowLimelight = !CAMERA_SELECTION.equalsIgnoreCase("C270");
        boolean allowC270 = !CAMERA_SELECTION.equalsIgnoreCase("LIMELIGHT");

        // 1. Initialize Limelight 3A (for Robot A)
        if (allowLimelight) {
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

            if (limelight != null) {
                try {
                    limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
                    limelight.start();
                    activeCameraType = CameraType.LIMELIGHT_3A;
                    FtcDashboard.getInstance().startCameraStream(limelight, 15);
                } catch (Exception ignored) {
                    limelight = null;
                }
            }
        }

        // 2. Initialize Logitech C270 Webcam (for Robot B)
        if (activeCameraType == CameraType.NONE && allowC270) {
            WebcamName webcamName = null;
            try {
                webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            } catch (Exception ignored) {
                List<WebcamName> webcams = hardwareMap.getAll(WebcamName.class);
                if (webcams != null && !webcams.isEmpty()) {
                    webcamName = webcams.get(0);
                }
            }

            if (webcamName != null) {
                try {
                    aprilTagProcessor = new AprilTagProcessor.Builder()
                            .setDrawTagID(true)
                            .setDrawTagOutline(true)
                            .setDrawAxes(true)
                            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                            .build();

                    visionPortal = new VisionPortal.Builder()
                            .setCamera(webcamName)
                            .setCameraResolution(new Size(640, 480))
                            .addProcessor(aprilTagProcessor)
                            .enableLiveView(true)
                            .setAutoStopLiveView(true)
                            .build();

                    activeCameraType = CameraType.LOGITECH_C270;
                    FtcDashboard.getInstance().startCameraStream(visionPortal, 15);
                } catch (Exception ignored) {
                    visionPortal = null;
                    aprilTagProcessor = null;
                    activeCameraType = CameraType.NONE;
                }
            }
        }
    }

    /**
     * Checks if a given AprilTag ID or metadata belongs to the specified alliance.
     * Blue Alliance Tags: 39, 40, 43, 44 (or Audience/Backdrop Tags 1, 2, 3, 11, 12, 13)
     * Red Alliance Tags: 31, 32, 35, 36 (or Audience/Backdrop Tags 4, 5, 6, 14, 15, 16)
     */
    public static boolean isAllianceTag(int tagId, boolean isRedAlliance) {
        if (isRedAlliance) {
            return tagId == 31 || tagId == 32 || tagId == 35 || tagId == 36 || tagId == 4 || tagId == 5 || tagId == 6 || tagId == 14 || tagId == 15 || tagId == 16;
        } else {
            return tagId == 39 || tagId == 40 || tagId == 43 || tagId == 44 || tagId == 1 || tagId == 2 || tagId == 3 || tagId == 11 || tagId == 12 || tagId == 13;
        }
    }

    public static boolean isAllianceTag(AprilTagSingleDetection singleDet, boolean isRedAlliance) {
        if (singleDet == null) return false;
        if (singleDet.metadata != null && singleDet.metadata.name != null) {
            String name = singleDet.metadata.name.toUpperCase();
            if (isRedAlliance && name.contains("RED")) return true;
            if (!isRedAlliance && name.contains("BLUE")) return true;
        }
        return isAllianceTag(singleDet.id, isRedAlliance);
    }

    /**
     * Resets the target lock-on state to allow acquiring a new AprilTag.
     */
    public void resetLock() {
        lockedTagId = -1;
        targetLocked = false;
        targetVisible = false;
        framesSinceTagSeen = 0;
    }

    /**
     * Polls the active camera (Limelight 3A or Logitech C270) for the latest frame results,
     * filtered strictly by the active alliance tag set.
     * Automatically locks onto detected target tags and maintains lock stability.
     */
    public void update(boolean isRedAlliance) {
        targetVisible = false;

        if (activeCameraType == CameraType.LIMELIGHT_3A) {
            updateLimelight(isRedAlliance);
        } else if (activeCameraType == CameraType.LOGITECH_C270) {
            updateLogitechC270(isRedAlliance);
        }

        // Persistence handling if tag is briefly lost in current frame
        if (!targetVisible) {
            if (targetLocked && framesSinceTagSeen < MAX_PERSISTENCE_FRAMES) {
                framesSinceTagSeen++;
                targetVisible = true; // Retain target lock and distance during brief occlusion
            } else {
                targetLocked = false;
                lockedTagId = -1;
                detectedTagId = -1;
                tx = 0.0;
                ty = 0.0;
                framesSinceTagSeen = 0;
            }
        }
    }

    /**
     * Limelight 3A polling implementation.
     */
    private void updateLimelight(boolean isRedAlliance) {
        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                LLResultTypes.FiducialResult matchedFiducial = null;

                // 1. Lock persistence
                if (lockedTagId != -1 && isAllianceTag(lockedTagId, isRedAlliance)) {
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == lockedTagId) {
                            matchedFiducial = fiducial;
                            break;
                        }
                    }
                }

                // 2. Lock acquisition
                if (matchedFiducial == null) {
                    double bestTx = Double.MAX_VALUE;
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        int fid = fiducial.getFiducialId();
                        if (isAllianceTag(fid, isRedAlliance)) {
                            double absTx = Math.abs(fiducial.getTargetXDegrees());
                            if (absTx < bestTx) {
                                bestTx = absTx;
                                matchedFiducial = fiducial;
                            }
                        }
                    }
                }

                // 3. Process matched tag
                if (matchedFiducial != null) {
                    targetVisible = true;
                    targetLocked = true;
                    lockedTagId = matchedFiducial.getFiducialId();
                    detectedTagId = lockedTagId;
                    tx = matchedFiducial.getTargetXDegrees();
                    ty = matchedFiducial.getTargetYDegrees();
                    framesSinceTagSeen = 0;

                    double angleToTargetRad = Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEG + ty);
                    double heightDifference = TARGET_CENTER_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES;

                    if (Math.tan(angleToTargetRad) > 0) {
                        double rawDistance = heightDifference / Math.tan(angleToTargetRad);
                        calculatedDistanceInches = Range.clip(rawDistance, 12.0, 144.0);
                    }
                }
            }
        }
    }

    /**
     * Logitech C270 Webcam (FTC VisionPortal & AprilTagProcessor) polling implementation.
     */
    private void updateLogitechC270(boolean isRedAlliance) {
        if (aprilTagProcessor == null) return;

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        if (currentDetections != null && !currentDetections.isEmpty()) {
            AprilTagSingleDetection matchedDetection = null;

            // 1. Lock persistence
            if (lockedTagId != -1) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection instanceof AprilTagSingleDetection) {
                        AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                        if (singleDet.id == lockedTagId && singleDet.ftcPose != null && isAllianceTag(singleDet, isRedAlliance)) {
                            matchedDetection = singleDet;
                            break;
                        }
                    }
                }
            }

            // 2. Lock acquisition
            if (matchedDetection == null) {
                double bestBearing = Double.MAX_VALUE;
                for (AprilTagDetection detection : currentDetections) {
                    if (detection instanceof AprilTagSingleDetection) {
                        AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                        if (singleDet.ftcPose != null && isAllianceTag(singleDet, isRedAlliance)) {
                            double absBearing = Math.abs(singleDet.ftcPose.bearing);
                            if (absBearing < bestBearing) {
                                bestBearing = absBearing;
                                matchedDetection = singleDet;
                            }
                        }
                    }
                }
            }

            // 3. Process matched tag
            if (matchedDetection != null && matchedDetection.ftcPose != null) {
                targetVisible = true;
                targetLocked = true;
                lockedTagId = matchedDetection.id;
                detectedTagId = lockedTagId;
                tx = matchedDetection.ftcPose.bearing;     // Horizontal angle offset in degrees
                ty = matchedDetection.ftcPose.elevation;   // Elevation angle in degrees
                calculatedDistanceInches = Range.clip(matchedDetection.ftcPose.range, 12.0, 144.0); // 3D Distance in inches
                framesSinceTagSeen = 0;
            }
        }
    }

    /**
     * Default update method (defaults to Blue Alliance).
     */
    public void update() {
        update(false);
    }

    public CameraType getCameraType() {
        return activeCameraType;
    }

    public String getCameraName() {
        switch (activeCameraType) {
            case LIMELIGHT_3A:
                return "Limelight 3A";
            case LOGITECH_C270:
                return "Logitech C270 Webcam";
            default:
                return "NONE (No Camera Detected)";
        }
    }

    public boolean isLimelightConnected() {
        return activeCameraType == CameraType.LIMELIGHT_3A;
    }

    public boolean isCameraConnected() {
        return activeCameraType != CameraType.NONE;
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }

    public boolean isTargetLocked() {
        return targetLocked && targetVisible;
    }

    public int getLockedTagId() {
        return lockedTagId;
    }

    public boolean isAimed() {
        return targetVisible && Math.abs(getTxError()) <= AIM_DEADBAND_DEG;
    }

    public double getTx() {
        return tx;
    }

    public double getTxError() {
        return tx - TURRET_AIM_OFFSET_DEG;
    }

    public double getTy() {
        return ty;
    }

    public double getCalculatedDistanceInches() {
        return calculatedDistanceInches;
    }

    public int getDetectedTagId() {
        return detectedTagId;
    }

    /**
     * Calculates the new target angle for the turret to align with the target shooting area,
     * offset relative to the AprilTag center, constrained to soft limits [-90°, +90°].
     */
    public double getTurretTargetAngle(double currentTurretAngleDeg) {
        double errorTx = getTxError();
        if (!targetVisible || Math.abs(errorTx) <= AIM_DEADBAND_DEG) {
            return currentTurretAngleDeg;
        }
        double desiredTurretAngle = currentTurretAngleDeg - (errorTx * TURRET_AUTO_AIM_GAIN);
        return Range.clip(desiredTurretAngle, TURRET_MIN_ANGLE_DEG, TURRET_MAX_ANGLE_DEG);
    }

    /**
     * Calculates required flywheel RPM dynamically based on AprilTag distance, ball type, and custom base RPM:
     * - isNectarMode = false (Pollen / Smaller Ball)
     * - isNectarMode = true (Nectar / Bigger Ball)
     */
    public double getFlywheelTargetRPM(boolean isNectarMode, double customBaseRPM) {
        if (!targetVisible || calculatedDistanceInches <= 0) {
            return customBaseRPM;
        }
        double slope = isNectarMode ? NECTAR_RPM_SLOPE : POLLEN_RPM_SLOPE;
        double calculatedRPM = customBaseRPM + (slope * calculatedDistanceInches);
        return Range.clip(calculatedRPM, FLYWHEEL_MIN_RPM, FLYWHEEL_MAX_RPM);
    }

    public double getFlywheelTargetRPM(boolean isNectarMode) {
        double baseRPM = isNectarMode ? NECTAR_BASE_RPM : POLLEN_BASE_RPM;
        return getFlywheelTargetRPM(isNectarMode, baseRPM);
    }

    /**
     * Calculates required shifter/hood servo position dynamically based on distance to the AprilTag.
     */
    public double getShifterTargetPosition() {
        if (!targetVisible || calculatedDistanceInches <= 0) return SHIFTER_CLOSED_POSITION;
        double minDistance = 24.0;
        double maxDistance = 120.0;
        double normDistance = Range.clip((calculatedDistanceInches - minDistance) / (maxDistance - minDistance), 0.0, 1.0);
        return SHIFTER_CLOSED_POSITION - normDistance * (SHIFTER_CLOSED_POSITION - SHIFTER_OPEN_POSITION);
    }

    /**
     * Stops Limelight 3A / VisionPortal on OpMode shutdown.
     */
    public void stop() {
        try {
            FtcDashboard.getInstance().stopCameraStream();
        } catch (Exception ignored) {}

        if (limelight != null) {
            try {
                limelight.stop();
            } catch (Exception ignored) {}
        }
        if (visionPortal != null) {
            try {
                visionPortal.close();
            } catch (Exception ignored) {}
        }
    }
}

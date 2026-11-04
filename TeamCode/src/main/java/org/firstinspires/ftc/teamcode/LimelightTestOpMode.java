package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/**
 * A lightweight, dedicated OpMode to test the Limelight 3A vision sensor.
 * Streams real-time Limelight status, FPS, tx/ty offsets, and AprilTag detections
 * directly to both the Driver Station and FTC Dashboard (http://192.168.43.1:8080/dash).
 */
@TeleOp(name = "Limelight Simple Test", group = "Vision")
public class LimelightTestOpMode extends LinearOpMode {

    private Limelight3A limelight;
    private int currentPipeline = 0;

    @Override
    public void runOpMode() {
        // Send telemetry simultaneously to Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Locate Limelight 3A device
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            List<Limelight3A> allLimelights = hardwareMap.getAll(Limelight3A.class);
            if (allLimelights != null && !allLimelights.isEmpty()) {
                limelight = allLimelights.get(0);
            }
        }

        if (limelight == null) {
            telemetry.addData("ERROR", "Limelight 3A device not found in Robot Configuration!");
            telemetry.update();
            return;
        }

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();

        telemetry.addData("Status", "Limelight 3A Initialized!");
        telemetry.addData("FTC Dashboard URL", "http://192.168.43.1:8080/dash");
        telemetry.addData("Controls", "D-Pad Up/Down: Change Pipeline Index");
        telemetry.update();

        waitForStart();

        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        while (opModeIsActive()) {
            // Gamepad controls to cycle Limelight pipelines (0 through 9)
            if (gamepad1.dpad_up && !lastDpadUp) {
                currentPipeline = (currentPipeline + 1) % 10;
                limelight.pipelineSwitch(currentPipeline);
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                currentPipeline = (currentPipeline - 1 + 10) % 10;
                limelight.pipelineSwitch(currentPipeline);
            }
            lastDpadDown = gamepad1.dpad_down;

            // 1. Limelight Hardware Status
            LLStatus status = limelight.getStatus();
            if (status != null) {
                telemetry.addData("Limelight Name", status.getName());
                telemetry.addData("Temp / CPU / FPS", "%.1f°C | %.1f%% CPU | %d FPS",
                        status.getTemp(), status.getCpu(), (int) status.getFps());
                telemetry.addData("Active Pipeline", "Index %d (%s)",
                        status.getPipelineIndex(), status.getPipelineType());
            }

            // 2. Limelight Vision Targeting Results
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("Target Status", "LOCKED");
                telemetry.addData("tx (Horizontal Offset)", "%.2f deg", result.getTx());
                telemetry.addData("ty (Vertical Offset)", "%.2f deg", result.getTy());
                telemetry.addData("ta (Target Area)", "%.2f%%", result.getTa());

                // AprilTag / Fiducial Detections
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                if (fiducialResults != null && !fiducialResults.isEmpty()) {
                    telemetry.addData("# AprilTags Detected", fiducialResults.size());
                    for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
                        telemetry.addData("AprilTag ID " + fiducial.getFiducialId(),
                                "X: %.1f°, Y: %.1f° (Family: %s)",
                                fiducial.getTargetXDegrees(), fiducial.getTargetYDegrees(), fiducial.getFamily());
                    }
                } else {
                    telemetry.addData("AprilTags", "None detected in current pipeline");
                }
            } else {
                telemetry.addData("Target Status", "NO TARGET DETECTED");
            }

            telemetry.update();
        }

        // Cleanup on stop
        if (limelight != null) {
            limelight.stop();
        }
    }
}

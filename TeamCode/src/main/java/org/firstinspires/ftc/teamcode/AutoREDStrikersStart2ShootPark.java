package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * RED alliance first autonomous.
 * Uses the same intake power and gate positions as StrikersConfig.
 *
 * Sequence: gate closed -> intake on -> flywheel on -> wait -> gate open -> wait -> gate closed
 *           -> drive forward -> drive back -> strafe right -> turn
 *           -> drive forward. Intake and flywheel run to the end.
 */
@Config
@Autonomous(name = "AutoREDStrikersStart2-ShootPark", group = "Auto")
public class AutoREDStrikersStart2ShootPark extends LinearOpMode {

    public static double TAG_TIMEOUT_SEC = 8.0;   // give up looking for an AprilTag after this long and run the fallback sequence
    public static double FALLBACK_BACK_IN = 120.0; // fallback: move back
    public static double FALLBACK_TURN_ANTICLOCKWISE_DEG = 180.0; // fallback: anticlockwise turn after moving back
    public static double FALLBACK_WAIT_SEC = 3.0;  // fallback: wait before opening the gate
    public static double FALLBACK_GATE_OPEN_SEC = 3.0; // fallback: gate open, shooting
    public static double FALLBACK_TURN_CLOCKWISE_DEG = 90.0; // fallback: clockwise turn after shooting
    public static double FALLBACK_MOVE_IN = 24.0;  // fallback: forward move after the clockwise turn
    public static double FALLBACK_SECOND_TURN_CLOCKWISE_DEG = 90.0; // fallback: clockwise turn after that move
    public static double FALLBACK_SECOND_MOVE_IN = 100.0; // fallback: last forward move
    public static double FLYWHEEL_RPM = 2700.0;   // auto shooter speed (teleop stays separate)
    public static double SPINUP_WAIT_SEC = 3.0;  // flywheel on, gate closed
    public static double GATE_OPEN_SEC = 3.0;     // gate open, shooting
    public static double DRIVE_DISTANCE_IN = 12.0; // drive back right after intake and flywheel start
    public static double SHAKE_IN = 1.0;          // shake distance: back this far, then forward this far
    public static int SHAKE_COUNT = 2;            // number of back-and-forward shakes at the far point
    public static double A_SHIFT_LEFT_IN = 10.0;  // sideways shift to the left after the shot
    public static double A_SECOND_MOVE_IN = 10.0; // forward move after the shift left, then shake again
    public static double A_BACK_IN = 10.0;        // move back after the second shake
    public static double A_SHIFT_RIGHT_IN = 10.0; // sideways shift to the right after moving back
    public static double SECOND_SHOT_SEC = 3.0;   // gate open, second shot
    public static double A_FINAL_SHIFT_LEFT_IN = 50.0; // last sideways shift to the left

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private double smoothedTargetRPM = 0.0;

    @Override
    public void runOpMode() {
        // Drive (Road Runner + Pinpoint)
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Intake
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, StrikersConfig.INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Gate and shifter, same defaults as teleop (both closed)
        Servo gate = hardwareMap.get(Servo.class, StrikersConfig.GATE_SERVO);
        Servo shifter = hardwareMap.get(Servo.class, StrikersConfig.SHIFTER_SERVO);
        gate.setPosition(StrikersConfig.CLOSED_POSITION);
        shifter.setPosition(StrikersConfig.SHIFTER_CLOSED_POSITION);

        // Right intake lift servo, same as teleop: starts UP. (null if it is not in the robot configuration)
        Servo intakeLiftRight;
        try {
            intakeLiftRight = hardwareMap.get(Servo.class, StrikersConfig.INTAKE_LIFT_RIGHT_NAME);
        } catch (Exception e) {
            intakeLiftRight = null;
        }
        if (intakeLiftRight != null) intakeLiftRight.setPosition(StrikersConfig.INTAKE_LIFT_RIGHT_UP_POS);

        // Flywheels
        shooter1 = hardwareMap.get(DcMotorEx.class, StrikersConfig.MOTOR_1_NAME);
        shooter2 = hardwareMap.get(DcMotorEx.class, StrikersConfig.MOTOR_2_NAME);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(StrikersConfig.MOTOR_2_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // Vision (Limelight 3A or Logitech C270, whichever is connected)
        LimelightShooterSubsystem vision = new LimelightShooterSubsystem(hardwareMap);

        telemetry.addData("Status", "Ready - RED first auto");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            vision.stop();
            return;
        }

        double flywheelRPM = FLYWHEEL_RPM;

        // Step 1: look for an AprilTag for up to TAG_TIMEOUT_SEC. If none is found, run the fallback sequence.
        boolean tagFound = false;
        ElapsedTime tagTimer = new ElapsedTime();
        while (opModeIsActive() && tagTimer.seconds() < TAG_TIMEOUT_SEC) {
            vision.update(true); // RED alliance AprilTags only (blue tags are ignored)
            tagFound = vision.isTargetVisible();
            telemetry.addData("AprilTag", tagFound ? "DETECTED - starting" : "NOT DETECTED - waiting");
            telemetry.addData("Time", "%.1f / %.1f s", tagTimer.seconds(), TAG_TIMEOUT_SEC);
            telemetry.addData("Tags seen", vision.getTagsSeenSummary());
            telemetry.update();
            if (tagFound) break;
            sleep(20);
        }
        vision.stop(); // done with the camera
        if (!opModeIsActive()) return;

        if (!tagFound) {
            runFallbackSequence(drive, intake, gate, flywheelRPM);
            return;
        }

        // Gate is closed. Intake and flywheel ON (the flywheel control runs in every loop below).
        intake.setPower(StrikersConfig.INTAKE_POWER);
        updateFlywheel(flywheelRPM);

        // Drive back (flywheel keeps spinning up while driving).
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .lineToX(here.position.x - DRIVE_DISTANCE_IN)
                            .build());
        }

        // Wait for the flywheel to spin up.
        holdFlywheel(flywheelRPM, SPINUP_WAIT_SEC, "Spinning up");

        // Open gate, wait.
        gate.setPosition(StrikersConfig.OPEN_POSITION);
        holdFlywheel(flywheelRPM, GATE_OPEN_SEC, "Gate open");

        // Close gate. The flywheel keeps running to the end of the sequence.
        gate.setPosition(StrikersConfig.CLOSED_POSITION);

        // Shift left (sideways, heading held). Left of the robot's heading is (-sin, cos).
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x - A_SHIFT_LEFT_IN * Math.sin(heading),
                                    here.position.y + A_SHIFT_LEFT_IN * Math.cos(heading)))
                            .build());
        }

        // Bring the right intake lift DOWN (same position as teleop's bumper-held DOWN).
        if (intakeLiftRight != null) {
            intakeLiftRight.setPosition(StrikersConfig.INTAKE_LIFT_RIGHT_DOWN_POS);
        }

        // Move forward along the robot's current heading.
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x + A_SECOND_MOVE_IN * Math.cos(heading),
                                    here.position.y + A_SECOND_MOVE_IN * Math.sin(heading)))
                            .build());
        }

        // Shake again.
        shake(drive);

        // Move back along the robot's current heading (opposite of forward).
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x - A_BACK_IN * Math.cos(heading),
                                    here.position.y - A_BACK_IN * Math.sin(heading)))
                            .build());
        }

        // Bring the right intake lift back UP.
        if (intakeLiftRight != null) {
            intakeLiftRight.setPosition(StrikersConfig.INTAKE_LIFT_RIGHT_UP_POS);
        }

        // Shift right (sideways, heading held). Right of the robot's heading is (sin, -cos).
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x + A_SHIFT_RIGHT_IN * Math.sin(heading),
                                    here.position.y - A_SHIFT_RIGHT_IN * Math.cos(heading)))
                            .build());
        }

        // Gate open, wait (shooting), then gate closed.
        if (opModeIsActive()) {
            gate.setPosition(StrikersConfig.OPEN_POSITION);
        }
        holdFlywheel(flywheelRPM, SECOND_SHOT_SEC, "Gate open (second shot)");
        gate.setPosition(StrikersConfig.CLOSED_POSITION);

        // Shift left (sideways, heading held). Left of the robot's heading is (-sin, cos).
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x - A_FINAL_SHIFT_LEFT_IN * Math.sin(heading),
                                    here.position.y + A_FINAL_SHIFT_LEFT_IN * Math.cos(heading)))
                            .build());
        }

        // Stop everything: intake, flywheel, drive and gate.
        smoothedTargetRPM = 0.0;
        shooter1.setPower(0.0);
        shooter2.setPower(0.0);
        intake.setPower(0.0);
        gate.setPosition(StrikersConfig.CLOSED_POSITION);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        telemetry.addData("Status", "Done - everything stopped");
        telemetry.update();
    }

    /**
     * Fallback when no AprilTag is found within TAG_TIMEOUT_SEC.
     * Gate closed -> intake and flywheel on -> back up -> turn anticlockwise -> wait -> gate open -> wait
     * -> turn clockwise -> gate closed -> move forward -> turn clockwise -> move forward. Intake and flywheel keep running to the end.
     */
    private void runFallbackSequence(MecanumDrive drive, DcMotorEx intake, Servo gate, double flywheelRPM) {
        // Keep gate closed. Start intake and flywheel.
        gate.setPosition(StrikersConfig.CLOSED_POSITION);
        intake.setPower(StrikersConfig.INTAKE_POWER);

        // Move back.
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .lineToX(here.position.x - FALLBACK_BACK_IN)
                            .build());
        }

        // Turn anticlockwise (Road Runner counterclockwise is positive).
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(FALLBACK_TURN_ANTICLOCKWISE_DEG))
                            .build());
        }

        // Wait with the flywheel running, then open the gate and wait (shooting).
        holdFlywheel(flywheelRPM, FALLBACK_WAIT_SEC, "Fallback: waiting");
        if (opModeIsActive()) {
            gate.setPosition(StrikersConfig.OPEN_POSITION);
        }
        holdFlywheel(flywheelRPM, FALLBACK_GATE_OPEN_SEC, "Fallback: gate open");

        // Turn clockwise (angle negated).
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(-Math.toRadians(FALLBACK_TURN_CLOCKWISE_DEG))
                            .build());
        }

        // Close the gate before the final move.
        gate.setPosition(StrikersConfig.CLOSED_POSITION);

        // Move forward along the robot's current heading.
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x + FALLBACK_MOVE_IN * Math.cos(heading),
                                    here.position.y + FALLBACK_MOVE_IN * Math.sin(heading)))
                            .build());
        }

        // Turn clockwise again (angle negated).
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(-Math.toRadians(FALLBACK_SECOND_TURN_CLOCKWISE_DEG))
                            .build());
        }

        // Move forward along the robot's current heading.
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x + FALLBACK_SECOND_MOVE_IN * Math.cos(heading),
                                    here.position.y + FALLBACK_SECOND_MOVE_IN * Math.sin(heading)))
                            .build());
        }

        // Intake and flywheel keep running until the period ends.
        while (opModeIsActive()) {
            double rpm1 = updateFlywheel(flywheelRPM);
            telemetry.addData("Status", "Fallback done - intake and flywheel running");
            telemetry.addData("Flywheel RPM", "target %.0f | measured %.0f", flywheelRPM, rpm1);
            telemetry.update();
            sleep(20);
        }
    }

    /** Shake effect: SHAKE_IN back, then SHAKE_IN forward, repeated SHAKE_COUNT times (along the field X axis). */
    private void shake(MecanumDrive drive) {
        for (int i = 0; i < SHAKE_COUNT && opModeIsActive(); i++) {
            Pose2d here = drive.localizer.getPose();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .lineToX(here.position.x - SHAKE_IN)
                            .build());
            if (!opModeIsActive()) break;
            Pose2d back = drive.localizer.getPose();
            runWithFlywheel(
                    drive.actionBuilder(back)
                            .lineToX(back.position.x + SHAKE_IN)
                            .build());
        }
    }

    /** Like Actions.runBlocking, but keeps the flywheel control loop running while the action runs. */
    private void runWithFlywheel(Action action) {
        boolean running = true;
        while (opModeIsActive() && running) {
            updateFlywheel(FLYWHEEL_RPM);
            TelemetryPacket packet = new TelemetryPacket();
            running = action.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    /** Runs the flywheel at targetRPM for the given time, using the same control as teleop. */
    private void holdFlywheel(double targetRPM, double seconds, String label) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            double rpm1 = updateFlywheel(targetRPM);
            telemetry.addData("Step", label);
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), seconds);
            telemetry.addData("Flywheel RPM", "target %.0f | measured %.0f", targetRPM, rpm1);
            telemetry.update();
            sleep(20);
        }
    }

    /** One control step, same slew limit and proportional correction as StrikersConfig. */
    private double updateFlywheel(double targetRPM) {
        double maxRPMChangePerLoop = 60.0;
        smoothedTargetRPM += Range.clip(targetRPM - smoothedTargetRPM,
                -maxRPMChangePerLoop, maxRPMChangePerLoop);

        double actualRPM1 = (shooter1.getVelocity() * 60.0) / StrikersConfig.TICKS_PER_REV;

        double power = 0.0;
        if (smoothedTargetRPM > 50.0) {
            double basePower = smoothedTargetRPM / 6000.0;
            double errorRPM = smoothedTargetRPM - actualRPM1;
            double kP = 0.00018;
            power = Range.clip(basePower + (errorRPM * kP), 0.0, 1.0);
        }
        shooter1.setPower(power);
        shooter2.setPower(power);
        return actualRPM1;
    }
}

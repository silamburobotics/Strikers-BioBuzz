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
 * Uses the same intake power and gate positions as WarriorsConfig.
 *
 * Sequence: gate closed -> intake on -> flywheel on -> wait -> gate open -> wait -> gate closed
 *           -> drive forward -> drive back -> strafe right -> turn
 *           -> drive forward. Intake and flywheel run to the end.
 */
@Config
@Autonomous(name = "2BLUEWarriorsUnderHive", group = "Auto")
public class Auto2BLUEWarriorsUnderHive extends LinearOpMode {

    public static double TAG_TIMEOUT_SEC = 8.0;   // give up looking for an AprilTag after this long and run the fallback sequence
    public static double FALLBACK_BACK_IN = 106.0;  // fallback: move back
    public static double FALLBACK_TURN_ANTICLOCKWISE_DEG = 180.0; // fallback: anticlockwise turn after moving back
    public static double FALLBACK_WAIT_SEC = 3.0;  // fallback: wait (shooting) with the gate already open
    public static double FALLBACK_SECOND_MOVE_IN = 100.0;  // fallback: second move back (after the gate closes)
    public static double FALLBACK_SHIFT_RIGHT_IN = 45.0; // fallback: sideways shift to the right after the last forward move
    public static double FLYWHEEL_RPM = 2820.0;   // auto shooter speed (teleop stays separate)
    public static double SPINUP_WAIT_SEC = 3.0;  // flywheel on, gate closed
    public static double GATE_OPEN_SEC = 3.0;     // gate open, shooting
    public static double SHAKE_IN = 1.0;          // shake distance: back this far, then forward this far (done twice, with the lift wiggle in between)
    public static double LIFT_WIGGLE = 0.02;      // right lift wiggle size during the shake (up by this, then back down)
    public static double LIFT_WIGGLE_MS = 200.0;  // time to let the servo move at each end of the wiggle
    public static double A_EARLY_BACK_IN = 14.0;   // move back after the first shot (replaces the earlier 10 in + 4 in), before the shift left
    public static double A_SHIFT_LEFT_IN = 12.0;  // sideways shift to the left after the shot
    public static double A_SECOND_MOVE_IN = 11.0;  // forward move after the shift left, then shake again
    public static double A_BACK_IN = 8.0;        // move back after the second shake
    public static double A_SHIFT_RIGHT_IN = 12.0; // sideways shift to the right after moving back
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
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, WarriorsConfig.INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Gate starts OPEN (sequence B closes it first if it runs). Shifter starts closed, same as teleop.
        Servo gate = hardwareMap.get(Servo.class, WarriorsConfig.GATE_SERVO);
        Servo shifter = hardwareMap.get(Servo.class, WarriorsConfig.SHIFTER_SERVO);
        gate.setPosition(WarriorsConfig.OPEN_POSITION);
        shifter.setPosition(WarriorsConfig.SHIFTER_CLOSED_POSITION);

        // Right intake lift servo, same as teleop: starts UP. (null if it is not in the robot configuration)
        Servo intakeLiftRight;
        try {
            intakeLiftRight = hardwareMap.get(Servo.class, WarriorsConfig.INTAKE_LIFT_RIGHT_NAME);
        } catch (Exception e) {
            intakeLiftRight = null;
        }
        if (intakeLiftRight != null) intakeLiftRight.setPosition(WarriorsConfig.INTAKE_LIFT_RIGHT_UP_POS);

        // Flywheels
        shooter1 = hardwareMap.get(DcMotorEx.class, WarriorsConfig.MOTOR_1_NAME);
        shooter2 = hardwareMap.get(DcMotorEx.class, WarriorsConfig.MOTOR_2_NAME);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(WarriorsConfig.MOTOR_1_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(WarriorsConfig.MOTOR_2_REVERSED
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
            vision.update(false); // BLUE alliance AprilTags only (red tags are ignored)
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

        // Gate is already open. Flywheel ON (the flywheel control runs in every loop below). Intake starts after the spin-up wait.
        updateFlywheel(flywheelRPM);

        // Wait for the flywheel to spin up.
        holdFlywheel(flywheelRPM, SPINUP_WAIT_SEC, "Spinning up");

        // Intake ON, then wait (gate is already open, so this is the shooting time).
        intake.setPower(WarriorsConfig.INTAKE_POWER);
        holdFlywheel(flywheelRPM, GATE_OPEN_SEC, "Shooting (gate already open)");

        // Close gate. The flywheel keeps running to the end of the sequence.
        gate.setPosition(WarriorsConfig.CLOSED_POSITION);

        // Move back along the robot's current heading (opposite of forward).
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x - A_EARLY_BACK_IN * Math.cos(heading),
                                    here.position.y - A_EARLY_BACK_IN * Math.sin(heading)))
                            .build());
        }

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
            intakeLiftRight.setPosition(WarriorsConfig.INTAKE_LIFT_RIGHT_DOWN_POS);
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

        // Shake: 1 in back and forward, right lift up 0.02 then down 0.02, then 1 in back and forward again.
        shakeWithLiftWiggle(drive, intakeLiftRight, flywheelRPM);

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
            intakeLiftRight.setPosition(WarriorsConfig.INTAKE_LIFT_RIGHT_UP_POS);
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
            openGateWithKick(gate, flywheelRPM);
        }
        holdFlywheel(flywheelRPM, SECOND_SHOT_SEC, "Gate open (second shot)");
        gate.setPosition(WarriorsConfig.CLOSED_POSITION);

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
        gate.setPosition(WarriorsConfig.CLOSED_POSITION);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        telemetry.addData("Status", "Done - everything stopped");
        telemetry.update();
    }

    /**
     * Fallback when no AprilTag is found within TAG_TIMEOUT_SEC.
     * Flywheel on (gate stays open, intake off) -> back up -> turn anticlockwise -> intake on -> wait (shooting)
     * -> gate closed -> move back -> shift right. Intake and flywheel keep running to the end.
     */
    private void runFallbackSequence(MecanumDrive drive, DcMotorEx intake, Servo gate, double flywheelRPM) {
        // Gate stays open (as set at INIT). Intake stays off. Flywheel ON.
        updateFlywheel(flywheelRPM);

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

        // Intake ON (runs to the end).
        intake.setPower(WarriorsConfig.INTAKE_POWER);

        // Wait with the flywheel running (the gate is already open, so this is the shooting time).
        holdFlywheel(flywheelRPM, FALLBACK_WAIT_SEC, "Fallback: shooting (gate already open)");

        // Close the gate before the final move.
        gate.setPosition(WarriorsConfig.CLOSED_POSITION);

        // Move back along the robot's current heading (opposite of forward).
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x - FALLBACK_SECOND_MOVE_IN * Math.cos(heading),
                                    here.position.y - FALLBACK_SECOND_MOVE_IN * Math.sin(heading)))
                            .build());
        }

        // Shift right (sideways, heading held). Right of the robot's heading is (sin, -cos).
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x + FALLBACK_SHIFT_RIGHT_IN * Math.sin(heading),
                                    here.position.y - FALLBACK_SHIFT_RIGHT_IN * Math.cos(heading)))
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

    /** One shake: SHAKE_IN back, then SHAKE_IN forward (along the field X axis). */
    private void shakeOnce(MecanumDrive drive) {
        if (!opModeIsActive()) return;
        Pose2d here = drive.localizer.getPose();
        runWithFlywheel(
                drive.actionBuilder(here)
                        .lineToX(here.position.x - SHAKE_IN)
                        .build());
        if (!opModeIsActive()) return;
        Pose2d back = drive.localizer.getPose();
        runWithFlywheel(
                drive.actionBuilder(back)
                        .lineToX(back.position.x + SHAKE_IN)
                        .build());
    }

    /**
     * Shake, then move the right lift up by LIFT_WIGGLE and back down by LIFT_WIGGLE, then shake again.
     * "Up" is toward INTAKE_LIFT_RIGHT_UP_POS from the current DOWN position.
     */
    private void shakeWithLiftWiggle(MecanumDrive drive, Servo intakeLiftRight, double flywheelRPM) {
        shakeOnce(drive);

        if (intakeLiftRight != null && opModeIsActive()) {
            double down = WarriorsConfig.INTAKE_LIFT_RIGHT_DOWN_POS;
            double upDirection = WarriorsConfig.INTAKE_LIFT_RIGHT_UP_POS >= down ? 1.0 : -1.0;
            intakeLiftRight.setPosition(Range.clip(down + upDirection * LIFT_WIGGLE, 0.0, 1.0)); // up by 0.02
            holdFlywheel(flywheelRPM, LIFT_WIGGLE_MS / 1000.0, "Lift wiggle up");
            intakeLiftRight.setPosition(down); // down by 0.02 (back to the DOWN position)
            holdFlywheel(flywheelRPM, LIFT_WIGGLE_MS / 1000.0, "Lift wiggle down");
        }

        shakeOnce(drive);
    }

    /** Opens the gate with a kick: goes to GATE_KICK_POSITION for GATE_KICK_MS, then settles at OPEN_POSITION. */
    private void openGateWithKick(Servo gate, double flywheelRPM) {
        gate.setPosition(WarriorsConfig.GATE_KICK_POSITION);
        holdFlywheel(flywheelRPM, WarriorsConfig.GATE_KICK_MS / 1000.0, "Gate kick");
        gate.setPosition(WarriorsConfig.OPEN_POSITION);
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

    /** One control step, same slew limit and proportional correction as WarriorsConfig. */
    private double updateFlywheel(double targetRPM) {
        double maxRPMChangePerLoop = 60.0;
        smoothedTargetRPM += Range.clip(targetRPM - smoothedTargetRPM,
                -maxRPMChangePerLoop, maxRPMChangePerLoop);

        double actualRPM1 = (shooter1.getVelocity() * 60.0) / WarriorsConfig.TICKS_PER_REV;

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

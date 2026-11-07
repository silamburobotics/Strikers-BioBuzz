package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
 * BLUE alliance, Start 1: shoot, then run Sequence A (blue tag found) or Sequence B (no blue tag).
 *
 * Shared steps: shooter on (gate already open, intake off) -> wait -> intake on, wait (shooting) -> gate closed
 *   -> turn clockwise -> forward -> back -> turn anticlockwise (right lift stays UP).
 * Then look for a BLUE AprilTag for up to TAG_TIMEOUT_SEC (not at the start, where it would always be found).
 * A (tag found): open gate -> wait -> drive back -> shift right.
 * B (no tag):    drive back -> turn 180 -> open gate -> wait -> shift left.
 */
@Config
@Autonomous(name = "1BLUEStrikersUnderHive", group = "Auto")
public class Auto1BLUEStrikersUnderHive extends LinearOpMode {

    public static double TAG_TIMEOUT_SEC = 2.0;       // after the shared steps, look for a blue AprilTag for this long; if none, run sequence B
    public static double FLYWHEEL_RPM = 2820.0;       // auto shooter speed (teleop stays separate)
    public static double SPINUP_WAIT_SEC = 3.0;       // flywheel on, gate already open, intake off
    public static double SHOOT_WAIT_SEC = 3.0;        // intake on, gate open: shooting
    public static double START_TURN_CLOCKWISE_DEG = 89.0;      // turn clockwise after the gate closes
    public static double LIFT_MOVE_FORWARD_IN = 50.0;          // forward move after the turn
    public static double LIFT_MOVE_BACK_IN = 48.0;             // come back the same distance
    public static double END_TURN_ANTICLOCKWISE_DEG = 94.0;    // turn back anticlockwise
    public static double A_WAIT_SEC = 3.0;                     // sequence A: wait after opening the gate
    public static double A_BACK_IN = 100.0;                    // sequence A: drive back
    public static double A_SHIFT_RIGHT_IN = 55.0;              // sequence A: final sideways shift to the right
    public static double B_BACK_IN = 100.0;                    // sequence B: drive back
    public static double B_TURN_ANTICLOCKWISE_DEG = 180.0;     // sequence B: turn after driving back
    public static double B_WAIT_SEC = 3.0;                     // sequence B: wait after opening the gate
    public static double B_SHIFT_LEFT_IN = 55.0;               // sequence B: final sideways shift to the left

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

        // Gate starts OPEN. Shifter starts closed, same as teleop.
        Servo gate = hardwareMap.get(Servo.class, StrikersConfig.GATE_SERVO);
        Servo shifter = hardwareMap.get(Servo.class, StrikersConfig.SHIFTER_SERVO);
        gate.setPosition(StrikersConfig.OPEN_POSITION);
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

        telemetry.addData("Status", "Ready - BLUE Start 1 auto");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            vision.stop();
            return;
        }

        double flywheelRPM = FLYWHEEL_RPM;

        // Step 1: shooter on. Gate is already open and the intake is off.
        updateFlywheel(flywheelRPM);

        // Step 2: wait for the flywheel to spin up.
        holdFlywheel(flywheelRPM, SPINUP_WAIT_SEC, "Spinning up");

        // Step 3: intake on, then wait (gate is already open, so this is the shooting time).
        intake.setPower(StrikersConfig.INTAKE_POWER);
        holdFlywheel(flywheelRPM, SHOOT_WAIT_SEC, "Shooting (gate already open)");

        // Step 4: gate closed.
        gate.setPosition(StrikersConfig.CLOSED_POSITION);

        // Step 5: turn clockwise (Road Runner counterclockwise is positive, so the angle is negated).
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(-Math.toRadians(START_TURN_CLOCKWISE_DEG))
                            .build());
        }

        // Step 6: move forward along the robot's current heading (the right lift stays UP the whole time).
        moveAlongHeading(drive, LIFT_MOVE_FORWARD_IN);

        // Step 7: come back the same distance.
        moveAlongHeading(drive, -LIFT_MOVE_BACK_IN);

        // Step 8: turn anticlockwise.
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(END_TURN_ANTICLOCKWISE_DEG))
                            .build());
        }

        // Step 9: NOW look for a BLUE AprilTag for up to TAG_TIMEOUT_SEC (not at the start, since it would always be found then).
        // The flywheel keeps running while we look. Found -> sequence A, not found -> sequence B.
        boolean tagFound = false;
        ElapsedTime tagTimer = new ElapsedTime();
        while (opModeIsActive() && tagTimer.seconds() < TAG_TIMEOUT_SEC) {
            updateFlywheel(flywheelRPM);
            vision.update(false); // BLUE alliance AprilTags only (red tags are ignored)
            tagFound = vision.isTargetVisible();
            telemetry.addData("AprilTag", tagFound ? "DETECTED - sequence A" : "NOT DETECTED - looking");
            telemetry.addData("Time", "%.1f / %.1f s", tagTimer.seconds(), TAG_TIMEOUT_SEC);
            telemetry.addData("Tags seen", vision.getTagsSeenSummary());
            telemetry.update();
            if (tagFound) break;
            sleep(20);
        }
        vision.stop(); // done with the camera
        if (!opModeIsActive()) return;

        if (tagFound) {
            // ---------- Sequence A: blue tag found ----------
            // Step 11: open gate (with the kick), wait, drive back, then shift right.
            if (opModeIsActive()) {
                openGateWithKick(gate, flywheelRPM);
            }
            holdFlywheel(flywheelRPM, A_WAIT_SEC, "Sequence A: waiting (gate open)");
            moveAlongHeading(drive, -A_BACK_IN);

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
        } else {
            // ---------- Sequence B: no blue tag ----------
            // Drive back, turn 180, open gate (with the kick), wait, then shift left.
            moveAlongHeading(drive, -B_BACK_IN);

            if (opModeIsActive()) {
                runWithFlywheel(
                        drive.actionBuilder(drive.localizer.getPose())
                                .turn(Math.toRadians(B_TURN_ANTICLOCKWISE_DEG))
                                .build());
            }

            if (opModeIsActive()) {
                openGateWithKick(gate, flywheelRPM);
            }
            holdFlywheel(flywheelRPM, B_WAIT_SEC, "Sequence B: waiting (gate open)");

            // Shift left (sideways, heading held). Left of the robot's heading is (-sin, cos).
            if (opModeIsActive()) {
                Pose2d here = drive.localizer.getPose();
                double heading = here.heading.toDouble();
                runWithFlywheel(
                        drive.actionBuilder(here)
                                .strafeTo(new Vector2d(
                                        here.position.x - B_SHIFT_LEFT_IN * Math.sin(heading),
                                        here.position.y + B_SHIFT_LEFT_IN * Math.cos(heading)))
                                .build());
            }
        }

        // Intake and flywheel keep running until the period ends.
        while (opModeIsActive()) {
            double rpm1 = updateFlywheel(flywheelRPM);
            telemetry.addData("Status", tagFound ? "Sequence A done - intake and flywheel running"
                    : "Sequence B done - intake and flywheel running");
            telemetry.addData("Flywheel RPM", "target %.0f | measured %.0f", flywheelRPM, rpm1);
            telemetry.update();
            sleep(20);
        }
    }

    /** Moves the robot along its current heading: positive = forward, negative = back. */
    private void moveAlongHeading(MecanumDrive drive, double inches) {
        if (!opModeIsActive()) return;
        Pose2d here = drive.localizer.getPose();
        double heading = here.heading.toDouble();
        runWithFlywheel(
                drive.actionBuilder(here)
                        .strafeTo(new Vector2d(
                                here.position.x + inches * Math.cos(heading),
                                here.position.y + inches * Math.sin(heading)))
                        .build());
    }

    /** Opens the gate with a kick: goes to GATE_KICK_POSITION for GATE_KICK_MS, then settles at OPEN_POSITION. */
    private void openGateWithKick(Servo gate, double flywheelRPM) {
        gate.setPosition(StrikersConfig.GATE_KICK_POSITION);
        holdFlywheel(flywheelRPM, StrikersConfig.GATE_KICK_MS / 1000.0, "Gate kick");
        gate.setPosition(StrikersConfig.OPEN_POSITION);
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

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
 * BLUE alliance, Start 1: shoot, move, then look for a BLUE AprilTag and park depending on the result.
 *
 * Shooter on (gate already open, intake off) -> wait -> intake on, wait 1 s (shooting) -> gate closed
 *   -> back 8 -> shift right 50 -> forward 10 -> shift left 3 -> shift right 4 -> wait 2 s -> shift left 48
 *   -> look for a BLUE AprilTag (up to 2 s)
 *   Found:     open gate, wait 1 s, then shift right 34 -> drive back 80 -> shift right 18.
 *   Not found: shift right 34 -> drive back 80 -> shift right 18.
 * Intake and flywheel run to the end.
 */
@Config
@Autonomous(name = "1BlueShootGardenShootParkCamera", group = "Auto")
public class Auto1BlueShootGardenShootParkCamera extends LinearOpMode {

    public static double TAG_TIMEOUT_SEC = 2.0;       // look for a blue AprilTag for up to this long (after the shift left 48)
    private double flywheelTargetRPM = StrikersConfig.MID_BATTERY_RPM; // set from the battery at init
    public static double SPINUP_WAIT_SEC = 3.0;       // flywheel on, gate already open, intake off
    public static double SHOOT_WAIT_SEC = 1.0;        // intake on, gate open: shooting
    public static double FIRST_BACK_IN = 8.0;                  // move back after the gate closes
    public static double FIRST_SHIFT_RIGHT_IN = 50.0;          // shift right
    public static double FIRST_FORWARD_IN = 10.0;               // move forward after the shift right
    public static double SMALL_SHIFT_LEFT_IN = 3.0;            // small shift left
    public static double SMALL_SHIFT_RIGHT_IN = 4.0;           // small shift right
    public static double MID_WAIT_SEC = 2.0;                   // wait before the big shift left
    public static double BIG_SHIFT_LEFT_IN = 48.0;             // shift left
    public static double FOUND_WAIT_SEC = 1.0;                 // tag found: wait after opening the gate
    public static double SHIFT_RIGHT_IN = 34.0;      // first shift right (both outcomes)
    public static double BACK_IN = 80.0;             // drive back (both outcomes)
    public static double SECOND_SHIFT_RIGHT_IN = 18.0; // final shift right (both outcomes)

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private double smoothedTargetRPM = 0.0;

    @Override
    public void runOpMode() {
        // Flywheel speed from battery voltage (read once at init; 2860 if unreadable)
        flywheelTargetRPM = StrikersConfig.getBatteryAdjustedRPM(hardwareMap);
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
            if (intakeLiftRight != null) {
                intakeLiftRight.setDirection(Servo.Direction.REVERSE);
            }
        } catch (Exception e) {
            intakeLiftRight = null;
        }
        if (intakeLiftRight != null) intakeLiftRight.setPosition(StrikersConfig.INTAKE_LIFT_UP_POS);

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

        telemetry.addData("Status", "Ready - BLUE Start 1 camera auto");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            vision.stop();
            return;
        }

        double flywheelRPM = flywheelTargetRPM;

        // Step 1: shooter on. Gate is already open and the intake is off.
        updateFlywheel(flywheelRPM);

        // Step 2: wait for the flywheel to spin up.
        holdFlywheel(flywheelRPM, SPINUP_WAIT_SEC, "Spinning up");

        // Step 3: intake on, then wait (gate is already open, so this is the shooting time).
        intake.setPower(StrikersConfig.INTAKE_POWER);
        holdFlywheel(flywheelRPM, SHOOT_WAIT_SEC, "Shooting (gate already open)");

        // Step 4: gate closed.
        gate.setPosition(StrikersConfig.CLOSED_POSITION);

        // Step 5: move back.
        moveAlongHeading(drive, -FIRST_BACK_IN);

        // Step 6: shift right.
        shiftSideways(drive, -FIRST_SHIFT_RIGHT_IN);

        // Step 7: move forward.
        moveAlongHeading(drive, FIRST_FORWARD_IN);

        // Step 8: shift left a little, then right a little.
        shiftSideways(drive, SMALL_SHIFT_LEFT_IN);
        shiftSideways(drive, -SMALL_SHIFT_RIGHT_IN);

        // Step 9: wait (flywheel keeps running).
        holdFlywheel(flywheelRPM, MID_WAIT_SEC, "Waiting");

        // Step 10: shift left.
        shiftSideways(drive, BIG_SHIFT_LEFT_IN);

        // Step 11: look for a BLUE AprilTag for up to TAG_TIMEOUT_SEC. Stops looking as soon as one is found.
        // The flywheel keeps running while we look.
        boolean tagFound = false;
        ElapsedTime tagTimer = new ElapsedTime();
        while (opModeIsActive() && tagTimer.seconds() < TAG_TIMEOUT_SEC) {
            updateFlywheel(flywheelRPM);
            vision.update(false); // BLUE alliance AprilTags only (red tags are ignored)
            tagFound = vision.isTargetVisible();
            telemetry.addData("AprilTag", tagFound ? "DETECTED" : "NOT DETECTED - looking");
            telemetry.addData("Time", "%.1f / %.1f s", tagTimer.seconds(), TAG_TIMEOUT_SEC);
            telemetry.addData("Tags seen", vision.getTagsSeenSummary());
            telemetry.update();
            if (tagFound) break;
            sleep(20);
        }
        vision.stop(); // done with the camera
        if (!opModeIsActive()) return;

        // Tag found: open the gate and wait first. Tag not found: skip this.
        if (tagFound) {
            gate.setPosition(StrikersConfig.OPEN_POSITION);
            holdFlywheel(flywheelRPM, FOUND_WAIT_SEC, "Tag found: waiting (gate open)");
        }

        // Both outcomes: shift right, drive back, shift right.
        shiftSideways(drive, -SHIFT_RIGHT_IN);
        moveAlongHeading(drive, -BACK_IN);
        shiftSideways(drive, -SECOND_SHIFT_RIGHT_IN);

        // Intake and flywheel keep running until the period ends.
        while (opModeIsActive()) {
            double rpm1 = updateFlywheel(flywheelRPM);
            telemetry.addData("Status", tagFound ? "Tag found - done, intake and flywheel running" : "No tag - done, intake and flywheel running");
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

    /** Sideways move with the heading held: positive = LEFT, negative = RIGHT. Left of the heading is (-sin, cos). */
    private void shiftSideways(MecanumDrive drive, double leftInches) {
        if (!opModeIsActive()) return;
        Pose2d here = drive.localizer.getPose();
        double heading = here.heading.toDouble();
        runWithFlywheel(
                drive.actionBuilder(here)
                        .strafeTo(new Vector2d(
                                here.position.x - leftInches * Math.sin(heading),
                                here.position.y + leftInches * Math.cos(heading)))
                        .build());
    }

    /** Like Actions.runBlocking, but keeps the flywheel control loop running while the action runs. */
    private void runWithFlywheel(Action action) {
        boolean running = true;
        while (opModeIsActive() && running) {
            updateFlywheel(flywheelTargetRPM);
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

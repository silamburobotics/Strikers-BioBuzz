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
 * RED alliance, Start 1: shoot, move, look for a RED AprilTag, then run the Tag Found or Tag Not Found path.
 *
 * Opening: flywheel on (gate open) -> wait 3 -> intake on, wait 1 -> gate closed -> back 8 -> shift right 50
 *   -> lift DOWN -> forward 5 -> wait 1 -> lift UP -> shift left 48 -> look for a RED AprilTag (up to 2 s).
 * Found:     gate open, wait 1 -> shift right 34 -> gate closed -> back 30 -> turn cw 90 -> lift DOWN -> forward 12
 *   -> wait 2 -> back 12 -> lift UP -> shift right 70 -> turn 90 -> shift right 18 -> lift DOWN -> forward 8
 *   -> gate open, wait 1 -> forward 3 -> wait 2 -> back 12 -> lift UP -> shift left 36.
 * Not found: shift right 34 -> back 100 -> shift left 18 -> turn 180 -> lift DOWN -> forward 8 -> gate open, wait 1
 *   -> forward 3 -> wait 2 -> back 10 -> lift UP -> shift left 36.
 * Intake and flywheel run to the end. Flywheel RPM comes from the battery voltage (2860 if unreadable).
 */
@Config
@Autonomous(name = "1RedShootGardenShootFlowerShootFlowerShootParkCamera", group = "Auto")
public class Auto1RedShootGardenShootFlowerShootFlowerShootParkCamera extends LinearOpMode {

    private double flywheelTargetRPM = StrikersConfig.MID_BATTERY_RPM; // set from the battery at init

    public static double TAG_TIMEOUT_SEC = 2.0;       // look for a red AprilTag for up to this long

    // Opening (both outcomes)
    public static double SPINUP_WAIT_SEC = 3.0;
    public static double SHOOT_WAIT_SEC = 1.0;
    public static double OPEN_BACK_IN = 8.0;
    public static double OPEN_SHIFT_RIGHT_IN = 50.0;
    public static double OPEN_FORWARD_IN = 5.0;
    public static double OPEN_WAIT_SEC = 1.0;
    public static double OPEN_SHIFT_LEFT_IN = 48.0;

    // Tag FOUND (turn sign: + = anticlockwise, - = clockwise)
    public static double F_GATE_WAIT_SEC = 1.0;
    public static double F_SHIFT_RIGHT_1_IN = 34.0;
    public static double F_BACK_1_IN = 30.0;
    public static double F_TURN_1_DEG = -90.0;        // clockwise 90
    public static double F_FORWARD_1_IN = 12.0;
    public static double F_WAIT_1_SEC = 2.0;
    public static double F_BACK_2_IN = 12.0;
    public static double F_SHIFT_RIGHT_2_IN = 70.0;
    public static double F_TURN_2_DEG = -90.0;        // "rotate 90" (clockwise). Change to +90 for anticlockwise
    public static double F_SHIFT_RIGHT_3_IN = 18.0;
    public static double F_FORWARD_2_IN = 8.0;
    public static double F_GATE_WAIT_2_SEC = 1.0;
    public static double F_FORWARD_3_IN = 3.0;
    public static double F_WAIT_2_SEC = 2.0;
    public static double F_BACK_3_IN = 12.0;
    public static double F_SHIFT_LEFT_IN = 36.0;

    // Tag NOT found
    public static double N_SHIFT_RIGHT_IN = 34.0;
    public static double N_BACK_IN = 100.0;
    public static double N_SHIFT_LEFT_1_IN = 18.0;
    public static double N_TURN_DEG = 180.0;          // rotate 180 (anticlockwise)
    public static double N_FORWARD_1_IN = 8.0;
    public static double N_GATE_WAIT_SEC = 1.0;
    public static double N_FORWARD_2_IN = 3.0;
    public static double N_WAIT_SEC = 2.0;
    public static double N_BACK_2_IN = 10.0;
    public static double N_SHIFT_LEFT_2_IN = 36.0;

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private double smoothedTargetRPM = 0.0;
    private boolean tagFound = false;                    // latched true as soon as a red tag is seen

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

        telemetry.addData("Status", "Ready - RED Start 1 shoot garden shoot flower shoot park");
        telemetry.addData("Flywheel RPM (battery)", "%.0f", flywheelTargetRPM);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            vision.stop();
            return;
        }

        double rpm = flywheelTargetRPM;

        // ---------- Opening (both outcomes) ----------
        updateFlywheel(rpm);                                                  // 1. flywheel on, gate already open
        holdFlywheel(rpm, SPINUP_WAIT_SEC, "Spinning up");                    // 2. wait for spin-up
        intake.setPower(StrikersConfig.INTAKE_POWER);                         // 3. intake on
        holdFlywheel(rpm, SHOOT_WAIT_SEC, "Shooting (gate already open)");    //    wait while shooting
        gate.setPosition(StrikersConfig.CLOSED_POSITION);                     // 4. gate closed
        moveAlongHeading(drive, -OPEN_BACK_IN);                               // 5. back
        shiftSideways(drive, -OPEN_SHIFT_RIGHT_IN);                           // 6. shift right
        liftDown(intakeLiftRight);                                            // 7. right lift DOWN
        moveAlongHeading(drive, OPEN_FORWARD_IN);                             // 8. forward
        holdFlywheel(rpm, OPEN_WAIT_SEC, "Waiting");                          // 9. wait
        liftUp(intakeLiftRight);                                              // 10. right lift UP
        // 11. shift left, looking for the RED AprilTag WHILE driving (the search window starts now).
        ElapsedTime tagTimer = new ElapsedTime();
        shiftSidewaysSearching(drive, OPEN_SHIFT_LEFT_IN, vision);

        // ---------- Keep looking for a RED AprilTag if it was not seen during the move ----------
        // The total search window is TAG_TIMEOUT_SEC from the start of the shift left, so no extra wait if the move took longer.
        while (opModeIsActive() && !tagFound && tagTimer.seconds() < TAG_TIMEOUT_SEC) {
            updateFlywheel(rpm);
            vision.update(true); // RED alliance AprilTags only (blue tags are ignored)
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

        if (tagFound) {
            // ---------- Tag FOUND ----------
            gate.setPosition(StrikersConfig.OPEN_POSITION);
            holdFlywheel(rpm, F_GATE_WAIT_SEC, "Tag found: gate open");
            shiftSideways(drive, -F_SHIFT_RIGHT_1_IN);
            gate.setPosition(StrikersConfig.CLOSED_POSITION);
            moveAlongHeading(drive, -F_BACK_1_IN);
            turnDegrees(drive, F_TURN_1_DEG);
            liftDown(intakeLiftRight);
            moveAlongHeading(drive, F_FORWARD_1_IN);
            holdFlywheel(rpm, F_WAIT_1_SEC, "Waiting");
            moveAlongHeading(drive, -F_BACK_2_IN);
            liftUp(intakeLiftRight);
            shiftSideways(drive, -F_SHIFT_RIGHT_2_IN);
            turnDegrees(drive, F_TURN_2_DEG);
            shiftSideways(drive, -F_SHIFT_RIGHT_3_IN);
            liftDown(intakeLiftRight);
            moveAlongHeading(drive, F_FORWARD_2_IN);
            if (opModeIsActive()) gate.setPosition(StrikersConfig.OPEN_POSITION);
            holdFlywheel(rpm, F_GATE_WAIT_2_SEC, "Gate open");
            moveAlongHeading(drive, F_FORWARD_3_IN);
            holdFlywheel(rpm, F_WAIT_2_SEC, "Waiting");
            moveAlongHeading(drive, -F_BACK_3_IN);
            liftUp(intakeLiftRight);
            shiftSideways(drive, F_SHIFT_LEFT_IN);
        } else {
            // ---------- Tag NOT found ----------
            shiftSideways(drive, -N_SHIFT_RIGHT_IN);
            moveAlongHeading(drive, -N_BACK_IN);
            shiftSideways(drive, N_SHIFT_LEFT_1_IN);
            turnDegrees(drive, N_TURN_DEG);
            liftDown(intakeLiftRight);
            moveAlongHeading(drive, N_FORWARD_1_IN);
            if (opModeIsActive()) gate.setPosition(StrikersConfig.OPEN_POSITION);
            holdFlywheel(rpm, N_GATE_WAIT_SEC, "Gate open");
            moveAlongHeading(drive, N_FORWARD_2_IN);
            holdFlywheel(rpm, N_WAIT_SEC, "Waiting");
            moveAlongHeading(drive, -N_BACK_2_IN);
            liftUp(intakeLiftRight);
            shiftSideways(drive, N_SHIFT_LEFT_2_IN);
        }

        // Intake and flywheel keep running until the period ends.
        while (opModeIsActive()) {
            double rpm1 = updateFlywheel(rpm);
            telemetry.addData("Status", tagFound ? "Tag found - done, intake and flywheel running" : "No tag - done, intake and flywheel running");
            telemetry.addData("Flywheel RPM", "target %.0f | measured %.0f", rpm, rpm1);
            telemetry.update();
            sleep(20);
        }
    }

    private void liftDown(Servo lift) {
        if (lift != null && opModeIsActive()) lift.setPosition(StrikersConfig.INTAKE_LIFT_DOWN_POS);
    }

    private void liftUp(Servo lift) {
        if (lift != null && opModeIsActive()) lift.setPosition(StrikersConfig.INTAKE_LIFT_UP_POS);
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

    /** Turn in place: positive = anticlockwise, negative = clockwise. */
    private void turnDegrees(MecanumDrive drive, double degrees) {
        if (!opModeIsActive()) return;
        runWithFlywheel(
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(degrees))
                        .build());
    }

    /** Shift left/right like shiftSideways, but checks the camera for a RED AprilTag every loop while driving. */
    private void shiftSidewaysSearching(MecanumDrive drive, double leftInches, LimelightShooterSubsystem vision) {
        if (!opModeIsActive()) return;
        Pose2d here = drive.localizer.getPose();
        double heading = here.heading.toDouble();
        runWithFlywheel(
                drive.actionBuilder(here)
                        .strafeTo(new Vector2d(
                                here.position.x - leftInches * Math.sin(heading),
                                here.position.y + leftInches * Math.cos(heading)))
                        .build(),
                vision);
    }

    private void runWithFlywheel(Action action) {
        runWithFlywheel(action, null);
    }

    /** Like Actions.runBlocking, but keeps the flywheel control loop running (and optionally the tag search) while the action runs. */
    private void runWithFlywheel(Action action, LimelightShooterSubsystem vision) {
        boolean running = true;
        while (opModeIsActive() && running) {
            updateFlywheel(flywheelTargetRPM);
            if (vision != null && !tagFound) {
                vision.update(true); // RED alliance AprilTags only
                tagFound = vision.isTargetVisible();
            }
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

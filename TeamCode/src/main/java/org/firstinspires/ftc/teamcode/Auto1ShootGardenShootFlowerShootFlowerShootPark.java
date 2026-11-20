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
 * RED alliance, Start 1: shoot, garden, shoot, flower, shoot, flower, shoot, park. No AprilTag check.
 *
 * Shoot -> turn cw 90 -> forward 54 -> wait -> back 50 -> turn cw 90 -> gate open, wait -> shift right 34
 *   -> back 36 -> turn acw 90 -> gate closed -> lift DOWN -> forward 12 -> shake -> back 12 -> lift UP
 *   -> turn acw 90 -> back 50 -> shift left 24 -> turn 180 -> forward 10 -> wait -> gate open
 *   -> back 18 -> shift left 55. Intake and flywheel run to the end.
 */
@Config
@Autonomous(name = "1ShootGardenShootFlowerShootFlowerShootPark", group = "Auto")
public class Auto1ShootGardenShootFlowerShootFlowerShootPark extends LinearOpMode {

    private double flywheelTargetRPM = StrikersConfig.MID_BATTERY_RPM; // set from the battery at init
    public static double SPINUP_WAIT_SEC = 3.0;       // flywheel on, gate already open, intake off
    public static double SHOOT_WAIT_SEC = 3.0;        // intake on, gate open: shooting
    public static double TURN_CLOCKWISE_DEG = 90.0;   // every clockwise turn
    public static double TURN_ANTICLOCKWISE_DEG = 90.0; // the anticlockwise turn
    public static double TURN_180_DEG = 180.0;        // the half turn (positive = anticlockwise, negative = clockwise)
    public static double FORWARD_1_IN = 54.0;         // first forward move
    public static double FORWARD_1_WAIT_SEC = 3.0;    // wait after the first forward move
    public static double BACK_1_IN = 50.0;            // come back
    public static double GATE_WAIT_SEC = 3.0;         // wait after opening the gate
    public static double SHIFT_RIGHT_IN = 34.0;       // shift right after the gate
    public static double BACK_2_IN = 36.0;            // drive back
    public static double FORWARD_2_IN = 12.0;         // forward move (lift down), then shake
    public static double SHAKE_IN = 2.0;              // shake distance: right this far, then left this far (done twice, with the lift wiggle in between)
    public static double LIFT_WIGGLE = 0.02;          // right lift wiggle size during the shake (up by this, then back down)
    public static double LIFT_WIGGLE_MS = 200.0;      // time to let the servo move at each end of the wiggle
    public static double BACK_3_IN = 12.0;            // move back after the shake
    public static double BACK_4_IN = 50.0;            // drive back after the anticlockwise turn
    public static double SHIFT_LEFT_1_IN = 24.0;      // shift left before the half turn
    public static double FORWARD_3_IN = 10.0;         // forward move after the half turn
    public static double FORWARD_3_WAIT_SEC = 3.0;    // wait after that forward move
    public static double BACK_5_IN = 18.0;            // move back after the last gate opening
    public static double SHIFT_LEFT_2_IN = 55.0;      // final shift left

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

        telemetry.addData("Status", "Ready - RED Start 1 auto");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double flywheelRPM = flywheelTargetRPM;

        // 1. Shooter on. Gate is already open and the intake is off.
        updateFlywheel(flywheelRPM);

        // 2. Wait for the flywheel to spin up.
        holdFlywheel(flywheelRPM, SPINUP_WAIT_SEC, "Spinning up");

        // 3. Intake on, then wait (gate is already open, so this is the shooting time).
        intake.setPower(StrikersConfig.INTAKE_POWER);
        holdFlywheel(flywheelRPM, SHOOT_WAIT_SEC, "Shooting (gate already open)");

        // 4. Gate closed.
        gate.setPosition(StrikersConfig.CLOSED_POSITION);

        turnDegrees(drive, -TURN_CLOCKWISE_DEG);                 // 5. turn clockwise
        moveAlongHeading(drive, FORWARD_1_IN);                    // 6. forward
        holdFlywheel(flywheelRPM, FORWARD_1_WAIT_SEC, "Waiting after forward move"); // 7. wait
        moveAlongHeading(drive, -BACK_1_IN);                      // 8. back
        turnDegrees(drive, -TURN_CLOCKWISE_DEG);                 // 9. turn clockwise

        // 10. Gate open, wait.
        if (opModeIsActive()) gate.setPosition(StrikersConfig.OPEN_POSITION);
        holdFlywheel(flywheelRPM, GATE_WAIT_SEC, "Waiting (gate open)");

        shiftSideways(drive, -SHIFT_RIGHT_IN);                    // 11. shift right
        moveAlongHeading(drive, -BACK_2_IN);                      // 12. drive back

        turnDegrees(drive, TURN_ANTICLOCKWISE_DEG);              // 13. turn anticlockwise
        gate.setPosition(StrikersConfig.CLOSED_POSITION);         // 14. gate closed

        // 15. Right lift DOWN (same position as teleop's bumper-held DOWN).
        if (intakeLiftRight != null) intakeLiftRight.setPosition(StrikersConfig.INTAKE_LIFT_DOWN_POS);

        moveAlongHeading(drive, FORWARD_2_IN);                    // 16. forward
        shakeWithLiftWiggle(drive, intakeLiftRight, flywheelRPM); // 17. shake with lift wiggle
        moveAlongHeading(drive, -BACK_3_IN);                      // 18. back

        // 19. Right lift UP.
        if (intakeLiftRight != null) intakeLiftRight.setPosition(StrikersConfig.INTAKE_LIFT_UP_POS);

        turnDegrees(drive, TURN_ANTICLOCKWISE_DEG);              // 20. turn anticlockwise
        moveAlongHeading(drive, -BACK_4_IN);                      // 21. drive back
        shiftSideways(drive, SHIFT_LEFT_1_IN);                    // 22. shift left
        turnDegrees(drive, TURN_180_DEG);                        // 23. turn 180
        moveAlongHeading(drive, FORWARD_3_IN);                    // 24. forward
        holdFlywheel(flywheelRPM, FORWARD_3_WAIT_SEC, "Waiting after forward move"); // 25. wait

        // 26. Gate open.
        if (opModeIsActive()) gate.setPosition(StrikersConfig.OPEN_POSITION);

        moveAlongHeading(drive, -BACK_5_IN);                      // 27. back
        shiftSideways(drive, SHIFT_LEFT_2_IN);                    // 28. shift left

        // Intake and flywheel keep running until the period ends.
        while (opModeIsActive()) {
            double rpm1 = updateFlywheel(flywheelRPM);
            telemetry.addData("Status", "Done - intake and flywheel running");
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

    /** Turns in place. Positive = anticlockwise, negative = clockwise (Road Runner counts anticlockwise as positive). */
    private void turnDegrees(MecanumDrive drive, double degrees) {
        if (!opModeIsActive()) return;
        runWithFlywheel(
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(degrees))
                        .build());
    }

    /** One shake: SHAKE_IN to the right, then SHAKE_IN to the left (sideways, heading held). */
    private void shakeOnce(MecanumDrive drive) {
        shiftSideways(drive, -SHAKE_IN); // right
        shiftSideways(drive, SHAKE_IN);  // left
    }

    /** Shake, then move the right lift up by LIFT_WIGGLE and back down by LIFT_WIGGLE, then shake again. */
    private void shakeWithLiftWiggle(MecanumDrive drive, Servo intakeLiftRight, double flywheelRPM) {
        shakeOnce(drive);

        if (intakeLiftRight != null && opModeIsActive()) {
            double down = StrikersConfig.INTAKE_LIFT_DOWN_POS;
            double upDirection = StrikersConfig.INTAKE_LIFT_UP_POS >= down ? 1.0 : -1.0;
            intakeLiftRight.setPosition(Range.clip(down + upDirection * LIFT_WIGGLE, 0.0, 1.0)); // up by LIFT_WIGGLE
            holdFlywheel(flywheelRPM, LIFT_WIGGLE_MS / 1000.0, "Lift wiggle up");
            intakeLiftRight.setPosition(down); // back to the DOWN position
            holdFlywheel(flywheelRPM, LIFT_WIGGLE_MS / 1000.0, "Lift wiggle down");
        }

        shakeOnce(drive);
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

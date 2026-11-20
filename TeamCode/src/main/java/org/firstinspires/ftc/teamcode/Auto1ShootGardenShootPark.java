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
 * Start 1: shoot, then run the Shoot Garden Shoot Park sequence. No AprilTag check.
 *
 * Shooter on (gate already open, intake off) -> wait -> intake on, wait (shooting) -> gate closed
 *   -> back 8 -> shift right 50 -> forward 4 -> shift left 3 -> shift right 4 -> wait 2 s -> shift left 48 (right lift stays UP)
 *   -> open gate -> wait -> shift right -> drive back -> shift right.
 */
@Config
@Autonomous(name = "1ShootGardenShootPark", group = "Auto")
public class Auto1ShootGardenShootPark extends LinearOpMode {

    public static double FLYWHEEL_RPM = 3000.0;       // auto shooter speed (teleop stays separate)
    public static double SPINUP_WAIT_SEC = 3.0;       // flywheel on, gate already open, intake off
    public static double SHOOT_WAIT_SEC = 3.0;        // intake on, gate open: shooting
    public static double FIRST_BACK_IN = 8.0;                  // move back after the gate closes
    public static double FIRST_SHIFT_RIGHT_IN = 50.0;          // shift right
    public static double FIRST_FORWARD_IN = 4.0;               // move forward after the shift right
    public static double SMALL_SHIFT_LEFT_IN = 3.0;            // small shift left
    public static double SMALL_SHIFT_RIGHT_IN = 4.0;           // small shift right
    public static double MID_WAIT_SEC = 2.0;                   // wait before the big shift left
    public static double BIG_SHIFT_LEFT_IN = 48.0;             // shift left
    public static double A_WAIT_SEC = 3.0;                     // wait after opening the gate
    public static double A_BACK_IN = 80.0;                   // sequence A: drive back
    public static double A_SHIFT_RIGHT_IN = 34.0;              // sequence A: first sideways shift to the right (before driving back)
    public static double A_SECOND_SHIFT_RIGHT_IN = 18.0;       // sequence A: final sideways shift to the right

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

        // Step 11: open gate, wait, shift right, drive back, then shift right again.
        if (opModeIsActive()) {
            gate.setPosition(StrikersConfig.OPEN_POSITION);
        }
        holdFlywheel(flywheelRPM, A_WAIT_SEC, "Waiting (gate open)");
        shiftSideways(drive, -A_SHIFT_RIGHT_IN);        // shift right
        moveAlongHeading(drive, -A_BACK_IN);            // drive back
        shiftSideways(drive, -A_SECOND_SHIFT_RIGHT_IN); // shift right again

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

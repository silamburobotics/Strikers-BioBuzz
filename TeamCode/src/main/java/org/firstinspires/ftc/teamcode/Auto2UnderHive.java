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
 * Start 2: wait, then shoot twice with a shake in between. No AprilTag check.
 *
 * Wait 7 s -> flywheel on (gate already open) -> wait 3 s -> intake on, wait 3 s (shooting) -> gate closed
 *   -> back 14 -> left 12 -> lift DOWN -> forward 12 -> shake (with lift wiggle) -> back 8 -> lift UP
 *   -> right 12 -> gate open with kick, wait 3 s (second shot) -> gate closed -> left 55 -> stop everything.
 */
@Config
@Autonomous(name = "2UnderHive", group = "Auto")
public class Auto2UnderHive extends LinearOpMode {

    public static double START_WAIT_SEC = 7.0;    // wait at the start before doing anything
    public static double FLYWHEEL_RPM = 2900.0;   // auto shooter speed (teleop stays separate)
    public static double SPINUP_WAIT_SEC = 3.0;   // flywheel on, gate already open, intake off
    public static double SHOOT_WAIT_SEC = 3.0;    // intake on, gate open: shooting
    public static double EARLY_BACK_IN = 14.0;    // move back after the first shot
    public static double SHIFT_LEFT_IN = 12.0;    // shift left after moving back
    public static double SECOND_MOVE_IN = 12.0;   // forward move after the shift left, then shake
    public static double SHAKE_IN = 2.0;          // shake distance: right this far, then left this far (done twice, with the lift wiggle in between)
    public static double LIFT_WIGGLE = 0.02;      // right lift wiggle size during the shake (up by this, then back down)
    public static double LIFT_WIGGLE_MS = 200.0;  // time to let the servo move at each end of the wiggle
    public static double BACK_IN = 8.0;           // move back after the shake
    public static double SHIFT_RIGHT_IN = 12.0;   // shift right after moving back
    public static double SECOND_SHOT_SEC = 3.0;   // gate open, second shot
    public static double FINAL_SHIFT_LEFT_IN = 50.0; // last sideways shift to the left

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

        telemetry.addData("Status", "Ready - 2 UnderHive");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double flywheelRPM = FLYWHEEL_RPM;

        // Wait at the start (everything stays off).
        ElapsedTime startTimer = new ElapsedTime();
        while (opModeIsActive() && startTimer.seconds() < START_WAIT_SEC) {
            telemetry.addData("Step", "Waiting to start");
            telemetry.addData("Time", "%.1f / %.1f s", startTimer.seconds(), START_WAIT_SEC);
            telemetry.update();
            sleep(20);
        }
        if (!opModeIsActive()) return;

        // Flywheel on. The gate is already open and the intake is off.
        updateFlywheel(flywheelRPM);
        holdFlywheel(flywheelRPM, SPINUP_WAIT_SEC, "Spinning up");

        // Intake on, then wait (gate is already open, so this is the shooting time).
        intake.setPower(StrikersConfig.INTAKE_POWER);
        holdFlywheel(flywheelRPM, SHOOT_WAIT_SEC, "Shooting (gate already open)");

        // Close the gate.
        gate.setPosition(StrikersConfig.CLOSED_POSITION);

        moveAlongHeading(drive, -EARLY_BACK_IN);   // move back
        shiftSideways(drive, SHIFT_LEFT_IN);       // shift left

        // Right lift DOWN (same position as teleop's bumper-held DOWN).
        if (intakeLiftRight != null) intakeLiftRight.setPosition(StrikersConfig.INTAKE_LIFT_RIGHT_DOWN_POS);

        moveAlongHeading(drive, SECOND_MOVE_IN);   // move forward

        shakeWithLiftWiggle(drive, intakeLiftRight, flywheelRPM);

        moveAlongHeading(drive, -BACK_IN);         // move back

        // Right lift UP.
        if (intakeLiftRight != null) intakeLiftRight.setPosition(StrikersConfig.INTAKE_LIFT_RIGHT_UP_POS);

        shiftSideways(drive, -SHIFT_RIGHT_IN);     // shift right

        // Gate open with a kick, wait (second shot), then gate closed.
        if (opModeIsActive()) openGateWithKick(gate, flywheelRPM);
        holdFlywheel(flywheelRPM, SECOND_SHOT_SEC, "Gate open (second shot)");
        gate.setPosition(StrikersConfig.CLOSED_POSITION);

        shiftSideways(drive, FINAL_SHIFT_LEFT_IN); // shift left

        stopEverything(drive, intake, gate);
    }

    /** Moves along the robot's current heading: positive = forward, negative = back. */
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

    /** One shake: SHAKE_IN to the right, then SHAKE_IN to the left (sideways, heading held). */
    private void shakeOnce(MecanumDrive drive) {
        shiftSideways(drive, -SHAKE_IN); // right
        shiftSideways(drive, SHAKE_IN);  // left
    }

    /** Shake, then move the right lift up by LIFT_WIGGLE and back down by LIFT_WIGGLE, then shake again. */
    private void shakeWithLiftWiggle(MecanumDrive drive, Servo intakeLiftRight, double flywheelRPM) {
        shakeOnce(drive);

        if (intakeLiftRight != null && opModeIsActive()) {
            double down = StrikersConfig.INTAKE_LIFT_RIGHT_DOWN_POS;
            double upDirection = StrikersConfig.INTAKE_LIFT_RIGHT_UP_POS >= down ? 1.0 : -1.0;
            intakeLiftRight.setPosition(Range.clip(down + upDirection * LIFT_WIGGLE, 0.0, 1.0)); // up by 0.02
            holdFlywheel(flywheelRPM, LIFT_WIGGLE_MS / 1000.0, "Lift wiggle up");
            intakeLiftRight.setPosition(down); // back to the DOWN position
            holdFlywheel(flywheelRPM, LIFT_WIGGLE_MS / 1000.0, "Lift wiggle down");
        }

        shakeOnce(drive);
    }

    /** Opens the gate with a kick: goes to GATE_KICK_POSITION for GATE_KICK_MS, then settles at OPEN_POSITION. */
    private void openGateWithKick(Servo gate, double flywheelRPM) {
        gate.setPosition(StrikersConfig.GATE_KICK_POSITION);
        holdFlywheel(flywheelRPM, StrikersConfig.GATE_KICK_MS / 1000.0, "Gate kick");
        gate.setPosition(StrikersConfig.OPEN_POSITION);
    }

    /** Stop everything: intake, flywheel, drive and gate. */
    private void stopEverything(MecanumDrive drive, DcMotorEx intake, Servo gate) {
        smoothedTargetRPM = 0.0;
        shooter1.setPower(0.0);
        shooter2.setPower(0.0);
        intake.setPower(0.0);
        gate.setPosition(StrikersConfig.CLOSED_POSITION);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        telemetry.addData("Status", "Done - everything stopped");
        telemetry.update();
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

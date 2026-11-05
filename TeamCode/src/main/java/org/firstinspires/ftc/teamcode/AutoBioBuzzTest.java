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
 * RED alliance first autonomous.
 * Uses the same intake power and gate positions as StrikersConfig.
 *
 * Sequence: gate closed -> intake on -> flywheel on -> wait -> gate open -> wait -> gate closed
 *           -> drive forward -> drive back -> strafe right -> turn
 *           -> drive forward. Intake and flywheel run to the end.
 */
@Config
@Autonomous(name = "AutoBioBuzzTest", group = "Auto")
public class AutoBioBuzzTest extends LinearOpMode {

    public static double FLYWHEEL_RPM = 2800.0;   // auto shooter speed (teleop stays separate)
    public static double SPINUP_WAIT_SEC = 3.0;  // flywheel on, gate closed
    public static double GATE_OPEN_SEC = 2.0;     // gate open, shooting
    public static double SECOND_SHOT_SEC = 3.0;   // gate open, second shot (after the turn)
    public static double DRIVE_DISTANCE_IN = 24.0; // forward move after shooting, then back the same distance
    public static double STRAFE_RIGHT_IN = 12.0;  // sideways move to the right at the end
    public static double TURN_CLOCKWISE_DEG = 90.0; // clockwise turn after the strafe
    public static double MOVE_AFTER_TURN_IN = 50.0; // forward move (robot's front) after the turn
    public static double SECOND_TURN_CLOCKWISE_DEG = 90.0; // clockwise turn after that move
    public static double EXTRA_TURN_CLOCKWISE_DEG = 90.0; // clockwise turn after the 12 in move
    public static double EXTRA_MOVE_IN = 50.0;    // forward move after that turn
    public static double FINAL_MOVE_IN = 12.0;    // last forward move (robot's front)

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

        telemetry.addData("Status", "Ready - RED first auto");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double flywheelRPM = FLYWHEEL_RPM;

        // Gate is closed. Start intake and flywheel, wait.
        intake.setPower(StrikersConfig.INTAKE_POWER);
        holdFlywheel(flywheelRPM, SPINUP_WAIT_SEC, "Spinning up");

        // Open gate, wait.
        gate.setPosition(StrikersConfig.OPEN_POSITION);
        holdFlywheel(flywheelRPM, GATE_OPEN_SEC, "Gate open");

        // Close gate. The flywheel keeps running to the end of the sequence.
        gate.setPosition(StrikersConfig.CLOSED_POSITION);

        // Move forward.
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DRIVE_DISTANCE_IN)
                            .build());
        }

        // Drive back to the start line.
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToX(0)
                            .build());
        }

        // Strafe right. In Road Runner +Y is left, so right is -Y.
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(here.position.x, here.position.y - STRAFE_RIGHT_IN))
                            .build());
        }

        // Turn clockwise. In Road Runner counterclockwise is positive, so the angle is negated.
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(-Math.toRadians(TURN_CLOCKWISE_DEG))
                            .build());
        }

        // Move forward along the robot's current heading (after the turn this is not field +X).
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x + MOVE_AFTER_TURN_IN * Math.cos(heading),
                                    here.position.y + MOVE_AFTER_TURN_IN * Math.sin(heading)))
                            .build());
        }

        // Turn clockwise. In Road Runner counterclockwise is positive, so the angle is negated.
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(-Math.toRadians(SECOND_TURN_CLOCKWISE_DEG))
                            .build());
        }

        // Open the gate and shoot again, then close it.
        if (opModeIsActive()) {
            gate.setPosition(StrikersConfig.OPEN_POSITION);
        }
        holdFlywheel(flywheelRPM, SECOND_SHOT_SEC, "Gate open (second shot)");
        gate.setPosition(StrikersConfig.CLOSED_POSITION);

        // Move forward along the robot's current heading.
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x + FINAL_MOVE_IN * Math.cos(heading),
                                    here.position.y + FINAL_MOVE_IN * Math.sin(heading)))
                            .build());
        }

        // Turn clockwise 90. In Road Runner counterclockwise is positive, so the angle is negated.
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(-Math.toRadians(EXTRA_TURN_CLOCKWISE_DEG))
                            .build());
        }

        // Move forward along the robot's current heading.
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x + EXTRA_MOVE_IN * Math.cos(heading),
                                    here.position.y + EXTRA_MOVE_IN * Math.sin(heading)))
                            .build());
        }

        // End of sequence. Returning from runOpMode ends the OpMode and the SDK powers every
        // motor off, so stay here with the intake and flywheel running until the period ends.
        while (opModeIsActive()) {
            double rpm1 = updateFlywheel(flywheelRPM);
            telemetry.addData("Status", "Done - intake and flywheel running");
            telemetry.addData("Flywheel RPM", "target %.0f | measured %.0f", flywheelRPM, rpm1);
            telemetry.update();
            sleep(20);
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

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
 * Uses the same intake power and gate positions as BioBuzzRoboCentricRed.
 *
 * Sequence: gate closed -> intake on -> flywheel on -> wait -> gate open -> wait -> gate closed
 *           -> drive forward -> wait -> drive back -> gate open -> wait -> strafe right -> turn
 *           -> drive forward. Intake and flywheel run to the end.
 */
@Config
@Autonomous(name = "BB-Auto-Red-First", group = "Auto")
public class BBAutoRedFirst extends LinearOpMode {

    public static double FLYWHEEL_RPM = 2800.0;   // auto shooter speed (teleop stays separate)
    public static double SPINUP_WAIT_SEC = 3.0;  // flywheel on, gate closed
    public static double GATE_OPEN_SEC = 3.0;     // gate open, shooting
    public static double DRIVE_DISTANCE_IN = 40.0; // forward move after shooting, then back the same distance
    public static double WAIT_AFTER_DRIVE_SEC = 3.0; // pause at the far point before driving back
    public static double WAIT_AFTER_GATE_OPEN_SEC = 3.0; // pause with the gate open back at the start line
    public static double STRAFE_RIGHT_IN = 40.0;  // sideways move to the right at the end
    public static double TURN_ANTICLOCKWISE_DEG = 90.0; // anticlockwise turn after the strafe
    public static double MOVE_AFTER_TURN_IN = 40.0; // forward move (robot's front) after the turn
    public static double SECOND_TURN_ANTICLOCKWISE_DEG = 90.0; // anticlockwise turn after that move
    public static double THIRD_TURN_ANTICLOCKWISE_DEG = 90.0; // anticlockwise turn before the final move
    public static double FINAL_MOVE_IN = 40.0;    // last forward move (robot's front)

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private double smoothedTargetRPM = 0.0;

    @Override
    public void runOpMode() {
        // Drive (Road Runner + Pinpoint)
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Intake
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, BioBuzzRoboCentricRed.INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Gate and shifter, same defaults as teleop (both closed)
        Servo gate = hardwareMap.get(Servo.class, BioBuzzRoboCentricRed.GATE_SERVO);
        Servo shifter = hardwareMap.get(Servo.class, BioBuzzRoboCentricRed.SHIFTER_SERVO);
        gate.setPosition(BioBuzzRoboCentricRed.CLOSED_POSITION);
        shifter.setPosition(BioBuzzRoboCentricRed.SHIFTER_CLOSED_POSITION);

        // Flywheels
        shooter1 = hardwareMap.get(DcMotorEx.class, BioBuzzRoboCentricRed.MOTOR_1_NAME);
        shooter2 = hardwareMap.get(DcMotorEx.class, BioBuzzRoboCentricRed.MOTOR_2_NAME);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(BioBuzzRoboCentricRed.MOTOR_2_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Ready - RED first auto");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double flywheelRPM = FLYWHEEL_RPM;

        // Gate is closed. Start intake and flywheel, wait.
        intake.setPower(BioBuzzRoboCentricRed.INTAKE_POWER);
        holdFlywheel(flywheelRPM, SPINUP_WAIT_SEC, "Spinning up");

        // Open gate, wait.
        gate.setPosition(BioBuzzRoboCentricRed.OPEN_POSITION);
        holdFlywheel(flywheelRPM, GATE_OPEN_SEC, "Gate open");

        // Close gate. The flywheel keeps running to the end of the sequence.
        gate.setPosition(BioBuzzRoboCentricRed.CLOSED_POSITION);

        // Move forward.
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DRIVE_DISTANCE_IN)
                            .build());
        }

        // Wait, then drive back to the start line.
        holdFlywheel(flywheelRPM, WAIT_AFTER_DRIVE_SEC, "Waiting at far point");

        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToX(0)
                            .build());
        }

        // Back at the start line: open the gate, wait.
        if (opModeIsActive()) {
            gate.setPosition(BioBuzzRoboCentricRed.OPEN_POSITION);
        }
        holdFlywheel(flywheelRPM, WAIT_AFTER_GATE_OPEN_SEC, "Gate open at start line");

        // Strafe right. In Road Runner +Y is left, so right is -Y.
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            runWithFlywheel(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(here.position.x, here.position.y - STRAFE_RIGHT_IN))
                            .build());
        }

        // Turn anticlockwise. In Road Runner counterclockwise is positive.
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(TURN_ANTICLOCKWISE_DEG))
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

        // Turn anticlockwise again.
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(SECOND_TURN_ANTICLOCKWISE_DEG))
                            .build());
        }

        // Turn anticlockwise a third time.
        if (opModeIsActive()) {
            runWithFlywheel(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(THIRD_TURN_ANTICLOCKWISE_DEG))
                            .build());
        }

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

    /** One control step, same slew limit and proportional correction as BioBuzzRoboCentricRed. */
    private double updateFlywheel(double targetRPM) {
        double maxRPMChangePerLoop = 60.0;
        smoothedTargetRPM += Range.clip(targetRPM - smoothedTargetRPM,
                -maxRPMChangePerLoop, maxRPMChangePerLoop);

        double actualRPM1 = (shooter1.getVelocity() * 60.0) / BioBuzzRoboCentricRed.TICKS_PER_REV;

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

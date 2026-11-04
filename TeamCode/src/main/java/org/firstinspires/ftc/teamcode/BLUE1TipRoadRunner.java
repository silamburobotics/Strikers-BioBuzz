package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * BLUE 1 TIP Autonomous converted to RoadRunner 1.0.
 *
 * Hardware names, servo positions, and shooter control match BioBuzzRoboCentricBlue.
 * Poses are mirrored horizontally across the field (x -> 144 - x) for the BLUE Alliance.
 * Uses Pinpoint odometry through MecanumDrive.
 */
@Config
@Autonomous(name = "BLUE 1 TIP - RoadRunner", group = "Auto")
public class BLUE1TipRoadRunner extends LinearOpMode {

    // ---------- Hardware names (same as BLUE TeleOp) ----------
    public static String INTAKE_NAME = BioBuzzRoboCentricBlue.INTAKE_NAME;
    public static String GATE_NAME = BioBuzzRoboCentricBlue.GATE_SERVO;
    public static String SHIFTER_NAME = BioBuzzRoboCentricBlue.SHIFTER_SERVO;
    public static String SHOOTER_1_NAME = BioBuzzRoboCentricBlue.MOTOR_1_NAME;
    public static String SHOOTER_2_NAME = BioBuzzRoboCentricBlue.MOTOR_2_NAME;
    public static String TURRET_NAME = BioBuzzRoboCentricBlue.TURRET_NAME;

    // ---------- Servo / Intake positions (same as BLUE TeleOp) ----------
    public static double GATE_OPEN = BioBuzzRoboCentricBlue.OPEN_POSITION;            // 0.5
    public static double GATE_CLOSED = BioBuzzRoboCentricBlue.CLOSED_POSITION;        // 0.7
    public static double SHIFTER_OPEN = BioBuzzRoboCentricBlue.SHIFTER_OPEN_POSITION;  // 0.25
    public static double SHIFTER_CLOSED = BioBuzzRoboCentricBlue.SHIFTER_CLOSED_POSITION; // 0.95
    public static double INTAKE_POWER = BioBuzzRoboCentricBlue.INTAKE_POWER;           // 1.0

    // ---------- Shooter (feedforward + PID control) ----------
    public static double FLYWHEEL_OPEN_RPM = BioBuzzRoboCentricBlue.FLYWHEEL_OPEN_RPM;     // 2400.0
    public static double FLYWHEEL_CLOSED_RPM = BioBuzzRoboCentricBlue.FLYWHEEL_CLOSED_RPM; // 2400.0
    public static boolean AUTO_SHIFTER_OPEN = false;
    public static double TICKS_PER_REV = BioBuzzRoboCentricBlue.TICKS_PER_REV;
    public static boolean MOTOR_2_REVERSED = BioBuzzRoboCentricBlue.MOTOR_2_REVERSED;
    public static double SHOOTER_kF = 0.00036;
    public static double SHOOTER_kP = 0.0005;
    public static double SHOOTER_kI = 0.0002;
    public static double SHOOTER_kD = 0.0;
    public static double SHOOTER_I_MAX = 0.25;
    public static double SHOOTER_VEL_FILTER_ALPHA = 0.6;
    public static double SHOOTER_NOMINAL_VOLTAGE = 12.0;
    public static double RPM_READY_TOLERANCE = 50.0;
    public static double SPIN_UP_TIMEOUT_SEC = 3.0;
    public static double SECONDS_PER_BALL = 0.7;
    public static double PARK_AT_SECONDS = 26.0;

    public static boolean ASSUME_HIVE_TIPPED = true;
    public static boolean DOUBLE_TIP = true;
    public static double COLLECT_DWELL_SEC = 1.0;

    // ---------- Turret Configuration ----------
    public static double TURRET_TICKS_PER_REV = BioBuzzRoboCentricBlue.TURRET_TICKS_PER_REV;
    public static double TURRET_GEAR_RATIO = BioBuzzRoboCentricBlue.TURRET_GEAR_RATIO; // 2.2727
    public static double TURRET_MAX_POWER = BioBuzzRoboCentricBlue.TURRET_MAX_POWER;
    public static double TURRET_MAX_ANGLE_DEG = BioBuzzRoboCentricBlue.TURRET_MAX_ANGLE_DEG;
    public static boolean TURRET_REVERSED = BioBuzzRoboCentricBlue.TURRET_REVERSED;
    public static double AUTO_TURRET_ANGLE_DEG = 0.0;

    // ---------- Hardware Declarations ----------
    private DcMotorEx intake, shooter1, shooter2, turret;
    private Servo gate, shifter;
    private VoltageSensor batteryVoltage;

    // ---------- Field Poses & Calculation (Blue Alliance Mirrored: x -> 144 - x) ----------
    public static double HIVE_X = 86.0; // 144 - 58
    public static double HIVE_Y = 73.0;

    public static double shooterHeading(double x, double y) {
        return Math.atan2(HIVE_Y - y, HIVE_X - x) + Math.PI;
    }

    public static double calculateTurretAngleForPose(Pose2d pose) {
        double fieldBearingToHive = Math.atan2(HIVE_Y - pose.position.y, HIVE_X - pose.position.x);
        double rearBearing = pose.heading.toDouble() + Math.PI;
        double deltaRad = fieldBearingToHive - rearBearing;
        while (deltaRad > Math.PI) deltaRad -= 2 * Math.PI;
        while (deltaRad < -Math.PI) deltaRad += 2 * Math.PI;
        return Math.toDegrees(deltaRad);
    }

    // Manual Setup: Point front INTAKE at garden pollen (0 deg) and manually aim TURRET at Blue Hive before hitting INIT.
    // Encoder zeroes at this manually-aimed turret position.
    private final Pose2d startPose = new Pose2d(113, 9, Math.toRadians(0));
    private final Pose2d shootPose = startPose;
    private final Pose2d collectPose = new Pose2d(104, 36, Math.toRadians(0));
    private final Pose2d parkPose = new Pose2d(135, 108, Math.toRadians(90));
    private final Pose2d otherSidePose = new Pose2d(84, 132, shooterHeading(84, 132));
    private final Pose2d collect2Pose = new Pose2d(94, 130, Math.toRadians(90));

    // ---------- Internal State ----------
    private final ElapsedTime opTimer = new ElapsedTime();
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private double shooterTargetRPM = 0.0;
    private double shooterIntegral = 0.0;
    private double lastShooterError = 0.0;
    private double filteredTPS = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. Initialize Mechanisms
        initMechanisms();

        // 2. Initialize RoadRunner MecanumDrive
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addData("Status", "Initialized BLUE Auto.");
        telemetry.addData("Setup Reminder", "Intake faces garden (0°), Turret manually aimed at Blue Hive.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        opTimer.reset();
        shooterTimer.reset();

        // Spin up flywheel & keep turret centered home (0 deg) to prevent intake collision
        shifter.setPosition(AUTO_SHIFTER_OPEN ? SHIFTER_OPEN : SHIFTER_CLOSED);
        shooterTargetRPM = AUTO_SHIFTER_OPEN ? FLYWHEEL_OPEN_RPM : FLYWHEEL_CLOSED_RPM;
        setTurretAngle(0.0);

        // --- STEP 1: Shoot 4 preloaded pollen into Blue Hive ---
        shoot(4);

        // --- STEP 2: Drive FORWARD to get garden pollen ---
        intake.setPower(INTAKE_POWER);
        Pose2d poseAtStart = drive.localizer.getPose();
        Action driveForwardToGarden = drive.actionBuilder(poseAtStart)
                .lineToX(poseAtStart.position.x + 24.0) // Drive forward 24 inches to garden pollen
                .build();
        runAction(driveForwardToGarden);

        // Dwell at garden to intake pollen
        collectDwell(COLLECT_DWELL_SEC);

        // --- STEP 3: Come BACK 1 foot (12 inches) ---
        Pose2d poseAtGarden = drive.localizer.getPose();
        Action backOneFoot = drive.actionBuilder(poseAtGarden)
                .lineToX(poseAtGarden.position.x - 12.0) // Move back 1 foot (12 inches)
                .build();
        runAction(backOneFoot);

        // --- STEP 4: Drive 5 feet (60 inches) & TURN to face hive ---
        Pose2d poseAfterBack = drive.localizer.getPose();
        Action driveToOtherSide = drive.actionBuilder(poseAfterBack)
                .lineToY(poseAfterBack.position.y + 60.0) // Strafe 5 feet (60 inches)
                .strafeToLinearHeading(otherSidePose.position, otherSidePose.heading) // Turn to face hive
                .build();
        runAction(driveToOtherSide);

        // --- STEP 5: Shoot 4 pollen into second cell ---
        shoot(4);

        if (DOUBLE_TIP && opTimer.seconds() < PARK_AT_SECONDS) {
            // --- STEP 6: Drive to flower (collect2Pose) ---
            intake.setPower(INTAKE_POWER);
            Action toCollect2Action = drive.actionBuilder(otherSidePose)
                    .strafeToLinearHeading(collect2Pose.position, collect2Pose.heading)
                    .build();
            runAction(toCollect2Action);

            // Dwell at flower
            collectDwell(COLLECT_DWELL_SEC);

            // Drive back to otherSidePose
            Action backToOtherAction = drive.actionBuilder(collect2Pose)
                    .strafeToLinearHeading(otherSidePose.position, otherSidePose.heading)
                    .build();
            runAction(backToOtherAction);

            // Shoot 4 pollen into second cell again
            shoot(4);
        }

        // --- STEP 7: Park in loading zone ---
        startPark(drive);

        stopMechanisms();
        telemetry.addData("Status", "Blue Auto Complete");
        telemetry.update();
    }

    // ---------- Helper Methods ----------

    private void initMechanisms() {
        intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gate = hardwareMap.get(Servo.class, GATE_NAME);
        shifter = hardwareMap.get(Servo.class, SHIFTER_NAME);
        gate.setPosition(GATE_CLOSED);
        shifter.setPosition(SHIFTER_CLOSED);

        shooter1 = hardwareMap.get(DcMotorEx.class, SHOOTER_1_NAME);
        shooter2 = hardwareMap.get(DcMotorEx.class, SHOOTER_2_NAME);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(MOTOR_2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        turret = hardwareMap.get(DcMotorEx.class, TURRET_NAME);
        turret.setDirection(TURRET_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(TURRET_MAX_POWER);
    }

    /** Runs a RoadRunner action while maintaining flywheel PID updates on every loop. */
    private void runAction(Action action) {
        TelemetryPacket packet = new TelemetryPacket();
        while (opModeIsActive() && action.run(packet)) {
            if (opTimer.seconds() > PARK_AT_SECONDS) break;
            updateShooter();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            packet = new TelemetryPacket();
        }
    }

    /** Non-blocking shoot procedure: waits for RPM ready, opens gate, feeds pollen, then closes gate. */
    private void shoot(int balls) {
        ElapsedTime timer = new ElapsedTime();
        // Spin-up phase
        while (opModeIsActive() && !shooterAtSpeed() && timer.seconds() < SPIN_UP_TIMEOUT_SEC) {
            if (opTimer.seconds() > PARK_AT_SECONDS) return;
            updateShooter();
            sleep(10);
        }

        // Feeding phase
        gate.setPosition(GATE_OPEN);
        intake.setPower(INTAKE_POWER);
        timer.reset();
        double duration = balls * SECONDS_PER_BALL;
        while (opModeIsActive() && timer.seconds() < duration) {
            if (opTimer.seconds() > PARK_AT_SECONDS) break;
            updateShooter();
            sleep(10);
        }

        // Close gate
        gate.setPosition(GATE_CLOSED);
    }

    /** Intake dwell helper for picking up pollen while updating flywheel. */
    private void collectDwell(double durationSec) {
        intake.setPower(INTAKE_POWER);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < durationSec) {
            if (opTimer.seconds() > PARK_AT_SECONDS) break;
            updateShooter();
            sleep(10);
        }
    }

    /** Park in loading zone. */
    private void startPark(MecanumDrive drive) {
        stopMechanisms();
        Pose2d currentPose = drive.localizer.getPose();
        Action parkAction = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                .build();
        runAction(parkAction);
        stopMechanisms();
    }

    /** Flywheel feedforward + PID control. */
    private void updateShooter() {
        double dt = shooterTimer.seconds();
        shooterTimer.reset();

        double targetTPS = (shooterTargetRPM * TICKS_PER_REV) / 60.0;
        double actualTPS = shooter1.getVelocity();
        filteredTPS = SHOOTER_VEL_FILTER_ALPHA * actualTPS + (1.0 - SHOOTER_VEL_FILTER_ALPHA) * filteredTPS;

        double power = 0.0;
        if (targetTPS > 0.0 && dt > 0.0) {
            double error = targetTPS - filteredTPS;

            shooterIntegral += error * dt;
            if (SHOOTER_kI > 0.0) {
                double maxIntegral = SHOOTER_I_MAX / SHOOTER_kI;
                shooterIntegral = Math.max(-maxIntegral, Math.min(maxIntegral, shooterIntegral));
            }
            double derivative = (error - lastShooterError) / dt;
            lastShooterError = error;

            double voltage = Math.max(batteryVoltage.getVoltage(), 1.0);
            double feedforward = SHOOTER_kF * targetTPS * (SHOOTER_NOMINAL_VOLTAGE / voltage);

            power = feedforward + SHOOTER_kP * error + SHOOTER_kI * shooterIntegral + SHOOTER_kD * derivative;
            power = Math.max(0.0, Math.min(1.0, power));
        } else {
            shooterIntegral = 0.0;
            lastShooterError = 0.0;
        }
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    private boolean shooterAtSpeed() {
        double rpm = (shooter1.getVelocity() * 60.0) / TICKS_PER_REV;
        return shooterTargetRPM > 0 && Math.abs(rpm - shooterTargetRPM) < RPM_READY_TOLERANCE;
    }

    private void setTurretAngle(double degrees) {
        double clamped = Math.max(-TURRET_MAX_ANGLE_DEG, Math.min(TURRET_MAX_ANGLE_DEG, degrees));
        double ticksPerDegree = (TURRET_TICKS_PER_REV * TURRET_GEAR_RATIO) / 360.0;
        turret.setTargetPosition((int) Math.round(clamped * ticksPerDegree));
        turret.setPower(TURRET_MAX_POWER);
    }

    private void stopMechanisms() {
        shooterTargetRPM = 0.0;
        shooter1.setPower(0.0);
        shooter2.setPower(0.0);
        intake.setPower(0.0);
        gate.setPosition(GATE_CLOSED);
        setTurretAngle(0.0);
    }

    private boolean detectAprilTag() {
        // Placeholder for vision detection (Blue Alliance tags: 39, 40, 43, 44)
        return false;
    }
}

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
 * BLUE alliance, Start 1: shoot, move, look for a BLUE AprilTag, then run the Tag Found or Tag Not Found path.
 *
 * Opening: flywheel on (gate open) -> wait 2.5 -> intake on, wait 1 -> gate closed -> back 10 -> shift right 50
 *   -> lift DOWN -> forward 8 -> wait 1 -> lift UP -> stop intake -> shift left 48 -> start intake -> look for a BLUE AprilTag during that move (up to 2 s from its start).
 * Found:     gate open, wait 1 -> shift right 34 -> gate closed -> back 30 -> turn cw 90 -> lift DOWN -> forward 12
 *   -> wait 2 -> back 12 -> lift UP -> shift right 50 -> forward 16.
 * Not found (turret motor moves to a fixed 30 deg left, no auto-aim): shift right 34 -> back 100 -> shift left 20 -> turn 180 -> lift DOWN -> forward 12 -> flywheel -100 RPM -> gate open, wait 1
 *   -> forward 6 -> wait 1 -> back 20 -> lift UP -> turn cw 60 -> back 30.
 * The turret auto-aims at the MIDDLE of the blue LEFT (39,40) and RIGHT (43,44) tag sets when both are visible (re-centers to 0 deg if the tags are lost). Turret must start centered.
 * Intake and flywheel run to the end. Flywheel RPM comes from the battery voltage (2860 if unreadable).
 */
@Config
@Autonomous(name = "1BlueCamera", group = "Auto")
public class Auto1BlueCamera extends LinearOpMode {

    private double flywheelTargetRPM = StrikersConfig.MID_BATTERY_RPM; // set from the battery at init

    public static double TAG_TIMEOUT_SEC = 2.0;       // look for a blue AprilTag for up to this long
    public static boolean AIM_AT_MIDPOINT = true;     // aim between the LEFT and RIGHT blue tag sets (false = aim at the single closest tag)
    public static double TAG_LOST_RECENTER_SEC = 0.3;  // tag lost this long -> turret re-centers to 0 deg (like teleop)
    public static boolean AUTO_AIM_ENABLED = true;    // turret auto-aims at the BLUE AprilTag whenever it is visible (set false to disable)

    // Opening (both outcomes)
    public static double SPINUP_WAIT_SEC = 2.5;
    public static double SHOOT_WAIT_SEC = 1.0;
    public static double OPEN_BACK_IN = 10.0;
    public static double OPEN_SHIFT_RIGHT_IN = 50.0;
    public static double OPEN_FORWARD_IN = 8.0;
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
    public static double F_SHIFT_RIGHT_2_IN = 50.0;
    public static double F_FORWARD_2_IN = 16.0;

    // Tag NOT found
    public static double N_SHIFT_RIGHT_IN = 34.0;
    public static double N_BACK_IN = 100.0;
    public static double N_SHIFT_LEFT_1_IN = 20.0;
    public static double N_TURN_DEG = 180.0;          // rotate 180 (anticlockwise)
    public static double N_FORWARD_1_IN = 12.0;
    public static double N_GATE_WAIT_SEC = 1.0;
    public static double N_FORWARD_2_IN = 6.0;
    public static double N_WAIT_SEC = 1.0;
    public static double N_BACK_2_IN = 20.0;
    public static double N_TURN_2_DEG = -60.0;        // clockwise 60
    public static double N_BACK_3_IN = 30.0;
    public static double NOT_FOUND_TURRET_ANGLE_DEG = 30.0; // not-found path: the turret MOTOR moves to this fixed angle from 0 (+ = left, - = right), instead of auto-aim. If it turns the wrong way, make it negative
    public static double SECOND_SHOT_RPM_REDUCTION = 100.0; // flywheel RPM lowered by this much from the gate open in this path onward

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private double smoothedTargetRPM = 0.0;
    private boolean tagFound = false;                    // latched true as soon as a blue tag is seen (only during the search window)
    private boolean searchingTag = false;                // true while the tag search window is open
    private LimelightShooterSubsystem vision;
    private DcMotorEx turret;                            // null if the turret is not in the robot configuration
    private boolean manualTurret = false;               // true = turret motor goes to a fixed angle (not-found path), auto-aim is ignored
    private double targetTurretAngle = 0.0;              // degrees, 0 = center home
    private final ElapsedTime tagLostTimer = new ElapsedTime(); // time since the aim target was last seen

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

        // Turret, same setup as teleop. The turret must be at its CENTER home position when the auto starts.
        try {
            turret = hardwareMap.get(DcMotorEx.class, StrikersConfig.TURRET_NAME);
            turret.setDirection(StrikersConfig.TURRET_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setTargetPosition(0);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(StrikersConfig.TURRET_MAX_POWER);
        } catch (Exception e) {
            turret = null;
        }

        // Vision (Limelight 3A or Logitech C270, whichever is connected). Stays on for the whole auto so the turret can aim.
        vision = new LimelightShooterSubsystem(hardwareMap);

        telemetry.addData("Status", "Ready - BLUE Start 1 shoot garden shoot flower shoot park");
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
        intake.setPower(0.0);                                                 //     stop intake
        // 11. shift left, looking for the BLUE AprilTag WHILE driving (the search window starts now).
        ElapsedTime tagTimer = new ElapsedTime();
        searchingTag = true;
        shiftSideways(drive, OPEN_SHIFT_LEFT_IN);
        if (opModeIsActive()) intake.setPower(StrikersConfig.INTAKE_POWER); // start intake

        // ---------- Keep looking for a BLUE AprilTag if it was not seen during the move ----------
        // The total search window is TAG_TIMEOUT_SEC from the start of the shift left, so no extra wait if the move took longer.
        while (opModeIsActive() && !tagFound && tagTimer.seconds() < TAG_TIMEOUT_SEC) {
            updateFlywheel(rpm); // also updates the camera and turret aim (BLUE tags only)
            telemetry.addData("AprilTag", tagFound ? "DETECTED" : "NOT DETECTED - looking");
            telemetry.addData("Time", "%.1f / %.1f s", tagTimer.seconds(), TAG_TIMEOUT_SEC);
            telemetry.addData("Tags seen", vision.getTagsSeenSummary());
            telemetry.addData("Tag sets seen", vision.getTagSetSummary());
            telemetry.update();
            if (tagFound) break;
            sleep(20);
        }
        searchingTag = false;
        if (!opModeIsActive()) {
            vision.stop();
            return;
        }

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
            moveAlongHeading(drive, F_FORWARD_2_IN);
        } else {
            // ---------- Tag NOT found ----------
            shiftSideways(drive, -N_SHIFT_RIGHT_IN);
            moveAlongHeading(drive, -N_BACK_IN);
            shiftSideways(drive, N_SHIFT_LEFT_1_IN);
            turnDegrees(drive, N_TURN_DEG);
            liftDown(intakeLiftRight);
            moveAlongHeading(drive, N_FORWARD_1_IN);
            manualTurret = true; // turret MOTOR moves to the fixed angle from 0 (no hand turning); auto-aim is off for the rest of the auto
            // Second shot: flywheel runs SECOND_SHOT_RPM_REDUCTION slower, for the rest of the auto.
            flywheelTargetRPM -= SECOND_SHOT_RPM_REDUCTION;
            rpm = flywheelTargetRPM;
            if (opModeIsActive()) gate.setPosition(StrikersConfig.OPEN_POSITION);
            holdFlywheel(rpm, N_GATE_WAIT_SEC, "Gate open (second shot)");
            moveAlongHeading(drive, N_FORWARD_2_IN);
            holdFlywheel(rpm, N_WAIT_SEC, "Waiting");
            moveAlongHeading(drive, -N_BACK_2_IN);
            liftUp(intakeLiftRight);
            turnDegrees(drive, N_TURN_2_DEG);
            moveAlongHeading(drive, -N_BACK_3_IN);
        }

        // Intake and flywheel keep running until the period ends.
        while (opModeIsActive()) {
            double rpm1 = updateFlywheel(rpm);
            telemetry.addData("Status", tagFound ? "Tag found - done, intake and flywheel running" : "No tag - done, intake and flywheel running");
            telemetry.addData("Flywheel RPM", "target %.0f | measured %.0f", rpm, rpm1);
            telemetry.update();
            sleep(20);
        }
        vision.stop(); // done with the camera
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

    /** Like Actions.runBlocking, but keeps the flywheel control loop (and camera / turret aim) running while the action runs. */
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

    /**
     * Updates the camera (BLUE tags only) and points the turret between the LEFT and RIGHT tag sets (AIM_AT_MIDPOINT),
     * or at the single closest tag. If it cannot aim (tags lost for TAG_LOST_RECENTER_SEC) the turret RE-CENTERS to 0 deg, like teleop.
     * During the search window it also latches tagFound.
     */
    private void updateAim() {
        if (vision == null) return;
        vision.update(false); // BLUE alliance AprilTags only (red tags are ignored)
        boolean visible = vision.isTargetVisible();
        if (searchingTag && !tagFound && visible) tagFound = true;

        if (turret == null) return;
        double tpd = (StrikersConfig.TURRET_TICKS_PER_REV * StrikersConfig.TURRET_GEAR_RATIO) / 360.0;
        if (manualTurret) {
            targetTurretAngle = Range.clip(NOT_FOUND_TURRET_ANGLE_DEG, -StrikersConfig.TURRET_MAX_ANGLE_DEG, StrikersConfig.TURRET_MAX_ANGLE_DEG);
            turret.setTargetPosition((int) Math.round(targetTurretAngle * tpd));
            turret.setPower(StrikersConfig.TURRET_MAX_POWER);
            return;
        }
        if (!AUTO_AIM_ENABLED) return;
        double ticksPerDegree = (StrikersConfig.TURRET_TICKS_PER_REV * StrikersConfig.TURRET_GEAR_RATIO) / 360.0;
        double currentAngle = turret.getCurrentPosition() / ticksPerDegree;
        boolean canAim;
        if (AIM_AT_MIDPOINT) {
            // Aim at the middle of the LEFT and RIGHT tag sets (needs both sets visible).
            canAim = vision.hasBothTagSets();
            if (canAim) targetTurretAngle = vision.getTurretTargetAngleToMidpoint(currentAngle);
        } else {
            canAim = visible;
            if (canAim && Math.abs(vision.getTxError()) > LimelightShooterSubsystem.AIM_DEADBAND_DEG) {
                targetTurretAngle = vision.getTurretTargetAngle(currentAngle);
            }
        }
        if (canAim) {
            tagLostTimer.reset();
        } else if (tagLostTimer.seconds() > TAG_LOST_RECENTER_SEC) {
            targetTurretAngle = 0.0; // tag lost: re-center to 0 deg, same as teleop
        }
        targetTurretAngle = Range.clip(targetTurretAngle, -StrikersConfig.TURRET_MAX_ANGLE_DEG, StrikersConfig.TURRET_MAX_ANGLE_DEG);
        turret.setTargetPosition((int) Math.round(targetTurretAngle * ticksPerDegree));
        turret.setPower(StrikersConfig.TURRET_MAX_POWER);
    }

    /** One control step, same slew limit and proportional correction as StrikersConfig. */
    private double updateFlywheel(double targetRPM) {
        updateAim(); // camera + turret aim runs in every loop (moves, waits and the final loop)
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

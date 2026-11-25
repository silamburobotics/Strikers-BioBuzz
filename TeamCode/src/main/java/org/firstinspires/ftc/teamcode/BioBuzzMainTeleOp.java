package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * BioBuzz Main TeleOp OpMode.
 * <p>
 * Complete standalone TeleOp combining full Mecanum Drive, Synchronized Intake Lift Arms,
 * Axon Gate & Shifter Servos, Tuned Shooter Velocity PIDF, Software Turret Position PID,
 * Distance-Based RPM Latching, and Limelight AprilTag Auto-Alignment.
 * <p>
 * Controls:
 * GAMEPAD 1 (Driver 1 - Chassis & Intake):
 * - Left Stick Y / X: Drive & Strafe
 * - Right Stick X: Turning / Rotation
 * - Left Bumper (Hold): Lower Intake Lift Arms (Release = Arms UP)
 * - Button A: Toggle Intake Motor Forward
 * - Button B: Toggle Intake Motor Reverse
 * <p>
 * GAMEPAD 2 (Operator - Shooter & Turret Controls):
 * - Button B (Hold): Gate Kick & Open (Release = Close Gate & Latch Shot RPM)
 * - Button Y: Toggle Flywheel ON / OFF
 * - Button A: Toggle Shifter Servo OPEN (Nectar) / CLOSED (Pollen)
 * - Button X: Reset PID Controllers (Clears integral sums & errors)
 * - Right Bumper (RB): Toggle Limelight Auto-Aim ON / OFF
 * - Left Bumper (LB): Hold Limelight Auto-Aim ON
 * - Right Stick X: Analog Turret Manual Aiming (-90° to +90°)
 * - Left Stick Click (LS): Toggle Limelight Auto-Aim
 * - D-Pad Up / Down: Fine-Tune Target RPM (+/- 50 RPM; Hold Right Trigger for +/- 10 RPM)
 * - D-Pad Left / Right: Cycle Preset Target RPMs
 */
@Config
public class BioBuzzMainTeleOp extends LinearOpMode {

    public enum IntakeMode {
        STOPPED,
        FORWARD,
        REVERSE
    }

    // ==========================================
    // 1. MECANUM DRIVE CONFIGURATION
    // ==========================================
    public static String LEFT_FRONT_NAME = "leftFront";
    public static String LEFT_BACK_NAME = "leftBack";
    public static String RIGHT_FRONT_NAME = "rightFront";
    public static String RIGHT_BACK_NAME = "rightBack";
    public static double DRIVE_POWER_SCALE = 1.0;
    public static boolean INVERT_DRIVE_Y = false;
    public static boolean INVERT_STRAFE_X = false;
    public static boolean INVERT_TURN_JOYSTICK = true;

    // ==========================================
    // 2. INTAKE MOTOR & LIFT SERVOS CONFIGURATION
    // ==========================================
    public static String INTAKE_NAME = "intake";
    public static double INTAKE_POWER = 1.0;
    public static String INTAKE_LIFT_LEFT_NAME = "intakeLiftLeft";
    public static String INTAKE_LIFT_RIGHT_NAME = "intakeLiftRight";
    public static double INTAKE_LIFT_UP_POS = 0.375;
    public static double INTAKE_LIFT_DOWN_POS = 0.77;
    public static double INTAKE_LIFT_RIGHT_UP_POS = 0.375;
    public static double INTAKE_LIFT_RIGHT_DOWN_POS = 0.78;
    public static boolean INTAKE_LIFT_RIGHT_REVERSED = true;

    // ==========================================
    // 3. AXON GATE & SHIFTER SERVOS CONFIGURATION
    // ==========================================
    public static String GATE_SERVO = "gateServo";
    public static double OPEN_POSITION = 0.4;
    public static double CLOSED_POSITION = 0.7;
    public static double GATE_KICK_POSITION = 0.4;
    public static double GATE_KICK_MS = 200.0;

    public static String SHIFTER_SERVO = "shifterServo";
    public static double SHIFTER_OPEN_POSITION = 0.25;
    public static double SHIFTER_CLOSED_POSITION = 0.95;

    // ==========================================
    // 4. FLYWHEEL MOTORS CONFIGURATION
    // ==========================================
    public static String MOTOR_1_NAME = "flyWheel1";
    public static String MOTOR_2_NAME = "flyWheel2";
    public static double FLYWHEEL_OPEN_RPM = 2625.0;
    public static double FLYWHEEL_CLOSED_RPM = 2375.0;
    public static double TICKS_PER_REV = 28.0;
    public static boolean MOTOR_1_REVERSED = false;
    public static boolean MOTOR_2_REVERSED = true;

    // ==========================================
    // 5. TURRET MOTOR CONFIGURATION
    // ==========================================
    public static String TURRET_NAME = "turret";
    public static double TURRET_TICKS_PER_REV = 537.7;
    public static double TURRET_GEAR_RATIO = 2.2727;
    public static double TURRET_MAX_ANGLE_DEG = 90.0;
    public static boolean TURRET_REVERSED = false;

    // ==========================================
    // 6. TUNED PIDF & PID CONTROLLER PARAMETERS
    // ==========================================
    public static double kP = 0.12;
    public static double kI = 0.01;
    public static double kD = 0.01;
    public static double kF = 0.00032;
    public static double maxIntegralSum = 0.25;

    public static double turretkP = 0.12;
    public static double turretkI = 0.02;
    public static double turretkD = 0.0015;
    public static double turretMaxPower = 0.4;

    // ==========================================
    // 7. DISTANCE-BASED RPM PARAMETERS
    // ==========================================
    public static double BASE_RPM = 2250.0;
    public static double BASE_DISTANCE_INCHES = 40.13;
    public static double RPM_PER_INCH = 12.00; //default value 12.39
    public static double MIN_RPM_LIMIT = 2000.0;
    public static double MAX_RPM_LIMIT = 3200.0;
    public static double RPM_TOLERANCE = 50.0;           // RPM tolerance window for speed ready check
    public static double RPM_WAIT_TIMEOUT_MS = 300.0;     // Max wait time (300ms) for RPM to reach target before forcing gate open

    // ==========================================
    // 8. SUBSYSTEM STATE VARIABLES
    // ==========================================
    public static double TARGET_RPM = 2350.0;
    public static boolean FLYWHEEL_ENABLED = false;
    public static boolean AUTO_AIM_ENABLED = false;
    public static boolean IS_RED_ALLIANCE = false;
    public static boolean AUTO_RPM_ENABLED = true;
    public static double TARGET_TURRET_ANGLE_DEG = 0.0;

    private static final double[] RPM_PRESETS = {2000.0, 2350.0, 2500.0, 2800.0, 3000.0};
    private int presetIndex = 1;

    private IntakeMode intakeMode = IntakeMode.STOPPED;
    private boolean isIntakeLiftUp = true;
    private final ElapsedTime gateOpenTimer = new ElapsedTime();
    private final ElapsedTime rpmWaitTimer = new ElapsedTime();
    private boolean isGateOpen = false;
    private boolean isShifterOpen = false;
    private boolean lastGatePressedState = false;
    private double latchedShotRPM = 2350.0;

    // Edge detection variables
    private boolean lastDpadUp1 = false, lastDpadUp2 = false;
    private boolean lastDpadDown1 = false, lastDpadDown2 = false;
    private boolean lastDpadLeft1 = false, lastDpadLeft2 = false;
    private boolean lastDpadRight1 = false, lastDpadRight2 = false;
    private boolean lastShooterY1 = false, lastShooterY2 = false;
    private boolean lastShifterA1 = false, lastShifterA2 = false;
    private boolean lastGamepadX1 = false, lastGamepadX2 = false;
    private boolean lastAutoAimRB1 = false, lastAutoAimRB2 = false;
    private boolean lastLeftStickBtn1 = false, lastLeftStickBtn2 = false;
    private boolean lastIntakeA1 = false, lastIntakeB1 = false;

    /**
     * Calculates target flywheel RPM from distance in inches using formula:
     * Target RPM = 2350 + 12.39 * (distance - 40.13)
     */
    public static double calculateDistanceRPM(double distInches) {
        if (distInches <= 0.0) return BASE_RPM;
        double calcRPM = BASE_RPM + RPM_PER_INCH * (distInches - BASE_DISTANCE_INCHES);
        return Range.clip(calcRPM, MIN_RPM_LIMIT, MAX_RPM_LIMIT);
    }

    /**
     * Custom Software Position PID Controller for Turret Alignment.
     */
    public static class CustomTurretPIDController {
        public double kP, kI, kD;
        public double maxPower;

        private double integralSum = 0.0;
        private double lastError = 0.0;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean isFirstRun = true;

        public CustomTurretPIDController(double kP, double kI, double kD, double maxPower) {
            setPID(kP, kI, kD, maxPower);
            timer.reset();
        }

        public void setPID(double kP, double kI, double kD, double maxPower) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.maxPower = maxPower;
        }

        public void reset() {
            integralSum = 0.0;
            lastError = 0.0;
            timer.reset();
            isFirstRun = true;
        }

        public double calculate(double currentAngleDeg, double targetAngleDeg) {
            double rawDt = timer.seconds();
            timer.reset();

            double dt = (rawDt <= 0.0 || rawDt > 0.2) ? 0.02 : rawDt;
            double error = targetAngleDeg - currentAngleDeg;

            double pTerm = kP * error;

            if (kI != 0 && Math.abs(error) < 20.0) {
                integralSum += error * dt;
                double maxIContribution = 0.25;
                double maxILimit = maxIContribution / kI;
                integralSum = Range.clip(integralSum, -maxILimit, maxILimit);
            } else {
                integralSum = 0.0;
            }
            double iTerm = kI * integralSum;

            double dTerm;
            if (isFirstRun) {
                dTerm = 0.0;
                isFirstRun = false;
            } else {
                double derivative = (error - lastError) / dt;
                dTerm = kD * derivative;
            }
            lastError = error;

            double totalPower = pTerm + iTerm + dTerm;
            return Range.clip(totalPower, -maxPower, maxPower);
        }
    }

    /**
     * External PIDF Controller for Flywheel Velocity Control.
     */
    public static class CustomPIDFController {
        public double kP, kI, kD, kF;
        public double maxIntegralSum;

        private double integralSum = 0.0;
        private double lastError = 0.0;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean isFirstRun = true;

        public CustomPIDFController(double kP, double kI, double kD, double kF, double maxIntegralSum) {
            setPIDF(kP, kI, kD, kF, maxIntegralSum);
            timer.reset();
        }

        public void setPIDF(double kP, double kI, double kD, double kF, double maxIntegralSum) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            this.maxIntegralSum = maxIntegralSum;
        }

        public void reset() {
            integralSum = 0.0;
            lastError = 0.0;
            timer.reset();
            isFirstRun = true;
        }

        public double calculate(double currentRPM, double targetRPM) {
            double rawDt = timer.seconds();
            timer.reset();

            double dt = (rawDt <= 0.0 || rawDt > 0.2) ? 0.02 : rawDt;
            double error = targetRPM - currentRPM;

            double pTerm = kP * error;

            if (kI != 0) {
                integralSum += error * dt;
                if (maxIntegralSum > 0) {
                    double maxIntegralLimit = maxIntegralSum / kI;
                    integralSum = Range.clip(integralSum, -maxIntegralLimit, maxIntegralLimit);
                }
            } else {
                integralSum = 0.0;
            }
            double iTerm = kI * integralSum;

            double dTerm;
            if (isFirstRun) {
                dTerm = 0.0;
                isFirstRun = false;
            } else {
                double derivative = (error - lastError) / dt;
                dTerm = kD * derivative;
            }
            lastError = error;

            double fTerm = kF * targetRPM;
            double totalOutput = pTerm + iTerm + dTerm + fTerm;
            return Range.clip(totalOutput, 0.0, 1.0);
        }
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ==========================================
        // 1. MECANUM DRIVE MOTORS SETUP
        // ==========================================
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT_NAME);
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, LEFT_BACK_NAME);
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_NAME);
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, RIGHT_BACK_NAME);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ==========================================
        // 2. INTAKE MOTOR & DUAL LIFT SERVOS SETUP
        // ==========================================
        DcMotorEx intake = null;
        try {
            intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception ignored) {}

        Servo intakeLiftLeft = null;
        Servo intakeLiftRight = null;
        try {
            intakeLiftLeft = hardwareMap.get(Servo.class, INTAKE_LIFT_LEFT_NAME);
        } catch (Exception ignored) {}

        try {
            intakeLiftRight = hardwareMap.get(Servo.class, INTAKE_LIFT_RIGHT_NAME);
            if (intakeLiftRight != null) {
                intakeLiftRight.setDirection(INTAKE_LIFT_RIGHT_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            }
        } catch (Exception ignored) {}

        // Set initial intake lift position to UP
        isIntakeLiftUp = true;
        if (intakeLiftLeft != null) intakeLiftLeft.setPosition(INTAKE_LIFT_UP_POS);
        if (intakeLiftRight != null) intakeLiftRight.setPosition(INTAKE_LIFT_RIGHT_UP_POS);

        // ==========================================
        // 3. SHOOTER FLYWHEEL MOTORS SETUP
        // ==========================================
        DcMotorEx shooter1 = null;
        DcMotorEx shooter2 = null;
        try {
            shooter1 = hardwareMap.get(DcMotorEx.class, MOTOR_1_NAME);
            shooter2 = hardwareMap.get(DcMotorEx.class, MOTOR_2_NAME);

            shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            shooter1.setDirection(MOTOR_1_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            shooter2.setDirection(MOTOR_2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        } catch (Exception ignored) {}

        // ==========================================
        // 4. AXON GATE & SHIFTER SERVOS SETUP
        // ==========================================
        Servo axonGateServo = null;
        Servo shifterServo = null;
        try {
            axonGateServo = hardwareMap.get(Servo.class, GATE_SERVO);
            axonGateServo.setPosition(CLOSED_POSITION);
        } catch (Exception ignored) {}

        try {
            shifterServo = hardwareMap.get(Servo.class, SHIFTER_SERVO);
            shifterServo.setPosition(SHIFTER_CLOSED_POSITION);
        } catch (Exception ignored) {}

        // ==========================================
        // 5. TURRET MOTOR SETUP
        // ==========================================
        DcMotorEx turret = null;
        try {
            turret = hardwareMap.get(DcMotorEx.class, TURRET_NAME);
            turret.setDirection(TURRET_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception ignored) {}

        // ==========================================
        // 6. LIMELIGHT 3A VISION SUBSYSTEM SETUP
        // ==========================================
        LimelightShooterSubsystem visionSubsystem = null;
        try {
            visionSubsystem = new LimelightShooterSubsystem(hardwareMap);
        } catch (Exception ignored) {}

        // Initialize External Software Controllers
        CustomPIDFController pid = new CustomPIDFController(kP, kI, kD, kF, maxIntegralSum);
        CustomTurretPIDController turretPid = new CustomTurretPIDController(turretkP, turretkI, turretkD, turretMaxPower);

        telemetry.addLine("=== BioBuzz Main TeleOp Initialized ===");
        telemetry.addLine("Drive: Gamepad 1 Sticks | Lift Arms: Gamepad 1 LB");
        telemetry.addLine("Intake Motor: Gamepad 1 A (Forward) / B (Reverse)");
        telemetry.addLine("Gate & Shooter: Gamepad 2 B (Gate) / Y (Flywheel) / A (Shifter)");
        telemetry.update();

        waitForStart();

        double lastIntakePower = Double.NaN;

        while (opModeIsActive()) {

            // ==========================================
            // A. GAMEPAD 1: MECANUM DRIVE & LIFT ARMS
            // ==========================================
            double y = INVERT_DRIVE_Y ? gamepad1.left_stick_y : -gamepad1.left_stick_y;
            double x = INVERT_STRAFE_X ? -gamepad1.left_stick_x : gamepad1.left_stick_x;
            double rx = INVERT_TURN_JOYSTICK ? -gamepad1.right_stick_x : gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double leftFrontPower = ((y + x + rx) / denominator) * DRIVE_POWER_SCALE;
            double leftBackPower = ((y - x + rx) / denominator) * DRIVE_POWER_SCALE;
            double rightFrontPower = ((y - x - rx) / denominator) * DRIVE_POWER_SCALE;
            double rightBackPower = ((y + x - rx) / denominator) * DRIVE_POWER_SCALE;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            // --- Intake Lift Arms Control (Gamepad 1 - Left Bumper: hold = DOWN, release = UP) ---
            isIntakeLiftUp = !gamepad1.left_bumper;
            double targetLeftLiftPos = isIntakeLiftUp ? INTAKE_LIFT_UP_POS : INTAKE_LIFT_DOWN_POS;
            double targetRightLiftPos = isIntakeLiftUp ? INTAKE_LIFT_RIGHT_UP_POS : INTAKE_LIFT_RIGHT_DOWN_POS;

            if (intakeLiftLeft != null) intakeLiftLeft.setPosition(targetLeftLiftPos);
            if (intakeLiftRight != null) intakeLiftRight.setPosition(targetRightLiftPos);

            // ==========================================
            // B. INTAKE MOTOR CONTROLS (Gamepad 1 - Button A: Forward, Button B: Reverse)
            // ==========================================
            if (gamepad1.a && !lastIntakeA1) {
                intakeMode = (intakeMode == IntakeMode.FORWARD) ? IntakeMode.STOPPED : IntakeMode.FORWARD;
            }
            lastIntakeA1 = gamepad1.a;

            if (gamepad1.b && !lastIntakeB1) {
                intakeMode = (intakeMode == IntakeMode.REVERSE) ? IntakeMode.STOPPED : IntakeMode.REVERSE;
            }
            lastIntakeB1 = gamepad1.b;

            double currentIntakePower = 0.0;
            if (intakeMode == IntakeMode.FORWARD) {
                currentIntakePower = INTAKE_POWER;
            } else if (intakeMode == IntakeMode.REVERSE) {
                currentIntakePower = -INTAKE_POWER;
            }

            if (intake != null && currentIntakePower != lastIntakePower) {
                intake.setPower(currentIntakePower);
                lastIntakePower = currentIntakePower;
            }

            // ==========================================
            // C. SHOOTER, TURRET & VISION CONTROLS (Gamepad 2)
            // ==========================================

            // --- Auto-Aim Controls (Gamepad 2: RB = Toggle ON/OFF, LB = Hold-to-Aim, LS = Toggle) ---
            if ((gamepad2.right_bumper && !lastAutoAimRB1) || (gamepad1.right_bumper && !lastAutoAimRB2)) {
                AUTO_AIM_ENABLED = !AUTO_AIM_ENABLED;
                if (!AUTO_AIM_ENABLED && visionSubsystem != null) visionSubsystem.resetLock();
            }
            lastAutoAimRB1 = gamepad2.right_bumper;
            lastAutoAimRB2 = gamepad1.right_bumper;

            if ((gamepad1.left_stick_button && !lastLeftStickBtn1) || (gamepad2.left_stick_button && !lastLeftStickBtn2)) {
                AUTO_AIM_ENABLED = !AUTO_AIM_ENABLED;
                if (!AUTO_AIM_ENABLED && visionSubsystem != null) visionSubsystem.resetLock();
            }
            lastLeftStickBtn1 = gamepad1.left_stick_button;
            lastLeftStickBtn2 = gamepad2.left_stick_button;

            // Gate Button Press check (Hold Gamepad 2 B)
            boolean isGatePressed = gamepad2.b || (gamepad1.b && gamepad1.right_trigger > 0.3);

            // Rising edge detection for Gate Press: LATCH target RPM & RESET 300ms speed wait timer
            if (isGatePressed && !lastGatePressedState) {
                rpmWaitTimer.reset();
                if (visionSubsystem != null && visionSubsystem.isTargetVisible()) {
                    latchedShotRPM = calculateDistanceRPM(visionSubsystem.getCalculatedDistanceInches());
                } else {
                    latchedShotRPM = isShifterOpen ? FLYWHEEL_OPEN_RPM : FLYWHEEL_CLOSED_RPM;
                }
            }
            lastGatePressedState = isGatePressed;

            boolean isAutoAimEngaged = AUTO_AIM_ENABLED || (gamepad2.left_bumper) || (gamepad2.left_trigger > 0.3) || (gamepad1.left_trigger > 0.3) || isGatePressed;

            // --- Toggle Flywheel ON / OFF (Gamepad 2: Button Y) ---
            if ((gamepad1.y && !lastShooterY1) || (gamepad2.y && !lastShooterY2)) {
                FLYWHEEL_ENABLED = !FLYWHEEL_ENABLED;
                if (!FLYWHEEL_ENABLED) pid.reset();
            }
            lastShooterY1 = gamepad1.y;
            lastShooterY2 = gamepad2.y;

            // --- Turret & Limelight Vision Alignment ---
            double currentTurretAngle = 0.0;
            int currentTurretTicks = 0;
            if (turret != null) {
                double ticksPerDegree = (TURRET_TICKS_PER_REV * TURRET_GEAR_RATIO) / 360.0;
                currentTurretTicks = turret.getCurrentPosition();
                currentTurretAngle = currentTurretTicks / ticksPerDegree;
            }

            if (visionSubsystem != null) {
                visionSubsystem.setCameraActive(isAutoAimEngaged);
                if (isAutoAimEngaged) {
                    visionSubsystem.update(IS_RED_ALLIANCE);
                    if (visionSubsystem.isTargetVisible()) {
                        if (Math.abs(visionSubsystem.getTxError()) > LimelightShooterSubsystem.AIM_DEADBAND_DEG) {
                            TARGET_TURRET_ANGLE_DEG = visionSubsystem.getTurretTargetAngle(currentTurretAngle);
                        }
                    } else {
                        TARGET_TURRET_ANGLE_DEG = 0.0;
                    }
                } else {
                    visionSubsystem.resetLock();
                }
            }

            // Calculate manual base RPM from shifter position
            double manualBaseRPM = isShifterOpen ? FLYWHEEL_OPEN_RPM : FLYWHEEL_CLOSED_RPM;

            // Target RPM: Locked at latchedShotRPM while gate is open, otherwise uses vision or manual base RPM
            if (isGatePressed) {
                FLYWHEEL_ENABLED = true;
                TARGET_RPM = latchedShotRPM;
            } else if (AUTO_RPM_ENABLED && visionSubsystem != null && visionSubsystem.isTargetVisible()) {
                TARGET_RPM = visionSubsystem.getFlywheelTargetRPM(isShifterOpen, manualBaseRPM);
            } else {
                TARGET_RPM = manualBaseRPM;
            }

            // Manual Turret Tuning overrides auto-aim if Right Stick is moved
            double stickX = (Math.abs(gamepad2.right_stick_x) > 0.1) ? gamepad2.right_stick_x : (Math.abs(gamepad1.right_stick_x) > 0.1 && gamepad1.right_trigger > 0.3 ? gamepad1.right_stick_x : 0.0);
            if (Math.abs(stickX) > 0.1) {
                TARGET_TURRET_ANGLE_DEG += stickX * 1.5;
            }
            TARGET_TURRET_ANGLE_DEG = Range.clip(TARGET_TURRET_ANGLE_DEG, -TURRET_MAX_ANGLE_DEG, TURRET_MAX_ANGLE_DEG);

            // Software Turret Position PID Calculation
            double turretPower = 0.0;
            if (turret != null) {
                turretPid.setPID(turretkP, turretkI, turretkD, turretMaxPower);
                turretPower = turretPid.calculate(currentTurretAngle, TARGET_TURRET_ANGLE_DEG);
                turret.setPower(turretPower);
            }

            // Reset PID Controllers (Button X)
            if ((gamepad1.x && !lastGamepadX1) || (gamepad2.x && !lastGamepadX2)) {
                pid.reset();
                turretPid.reset();
            }
            lastGamepadX1 = gamepad1.x;
            lastGamepadX2 = gamepad2.x;

            // Tune Target RPM with D-Pad Up / Down
            double rpmStep = (gamepad1.right_trigger > 0.3 || gamepad2.right_trigger > 0.3) ? 10.0 : 50.0;
            if ((gamepad1.dpad_up && !lastDpadUp1) || (gamepad2.dpad_up && !lastDpadUp2)) {
                TARGET_RPM += rpmStep;
            }
            lastDpadUp1 = gamepad1.dpad_up;
            lastDpadUp2 = gamepad2.dpad_up;

            if ((gamepad1.dpad_down && !lastDpadDown1) || (gamepad2.dpad_down && !lastDpadDown2)) {
                TARGET_RPM -= rpmStep;
            }
            lastDpadDown1 = gamepad1.dpad_down;
            lastDpadDown2 = gamepad2.dpad_down;

            TARGET_RPM = Range.clip(TARGET_RPM, 0.0, 5000.0);

            // Cycle RPM Presets with D-Pad Left / Right
            if ((gamepad1.dpad_right && !lastDpadRight1) || (gamepad2.dpad_right && !lastDpadRight2)) {
                presetIndex = (presetIndex + 1) % RPM_PRESETS.length;
                TARGET_RPM = RPM_PRESETS[presetIndex];
            }
            lastDpadRight1 = gamepad1.dpad_right;
            lastDpadRight2 = gamepad2.dpad_right;

            if ((gamepad1.dpad_left && !lastDpadLeft1) || (gamepad2.dpad_left && !lastDpadLeft2)) {
                presetIndex = (presetIndex - 1 + RPM_PRESETS.length) % RPM_PRESETS.length;
                TARGET_RPM = RPM_PRESETS[presetIndex];
            }
            lastDpadLeft1 = gamepad1.dpad_left;
            lastDpadLeft2 = gamepad2.dpad_left;

            // ==========================================
            // D. SERVOS & FLYWHEEL EXECUTIONS
            // ==========================================

            // Read current measured RPM for speed check
            double actualRPM1 = 0.0;
            if (shooter1 != null) {
                double actualTPS1 = shooter1.getVelocity();
                actualRPM1 = Math.abs((actualTPS1 * 60.0) / TICKS_PER_REV);
            }

            // --- Axon Gate Servo (RPM Speed Ready or 300ms Max Timeout) ---
            boolean isRpmReady = (actualRPM1 > 50.0) && (Math.abs(actualRPM1 - latchedShotRPM) <= RPM_TOLERANCE);
            boolean isRpmTimeout = rpmWaitTimer.milliseconds() >= RPM_WAIT_TIMEOUT_MS;
            boolean shouldOpenGate = isGatePressed && (isRpmReady || isRpmTimeout);

            boolean wasGateOpen = isGateOpen;
            isGateOpen = shouldOpenGate;

            if (isGateOpen && !wasGateOpen) {
                gateOpenTimer.reset();
            }

            double targetGatePos;
            if (!isGateOpen) {
                targetGatePos = CLOSED_POSITION;
            } else if (gateOpenTimer.milliseconds() < GATE_KICK_MS) {
                targetGatePos = GATE_KICK_POSITION;
            } else {
                targetGatePos = OPEN_POSITION;
            }

            if (axonGateServo != null) {
                axonGateServo.setPosition(targetGatePos);
            }

            // --- Shifter Servo (Gamepad 2: Button A) ---
            if ((gamepad2.a && !lastShifterA1) || (gamepad1.a && !lastShifterA2 && gamepad1.right_trigger > 0.3)) {
                isShifterOpen = !isShifterOpen;
            }
            lastShifterA1 = gamepad2.a;
            lastShifterA2 = gamepad1.a;

            double targetShifterPos = isShifterOpen ? SHIFTER_OPEN_POSITION : SHIFTER_CLOSED_POSITION;
            if (shifterServo != null) {
                shifterServo.setPosition(targetShifterPos);
            }

            // --- External PID Flywheel Velocity Calculation ---
            double motorPower = 0.0;
            if (shooter1 != null) {
                pid.setPIDF(kP, kI, kD, kF, maxIntegralSum);

                if (FLYWHEEL_ENABLED && TARGET_RPM > 50.0) {
                    motorPower = pid.calculate(actualRPM1, TARGET_RPM);
                    shooter1.setPower(motorPower);
                    if (shooter2 != null) shooter2.setPower(motorPower);
                } else {
                    pid.reset();
                    shooter1.setPower(0.0);
                    if (shooter2 != null) shooter2.setPower(0.0);
                }
            }

            // ==========================================
            // E. TELEMETRY
            // ==========================================
            double errorRPM = TARGET_RPM - actualRPM1;

            telemetry.addData("--- Alliance ---", IS_RED_ALLIANCE ? "RED ALLIANCE (Tags 31, 32, 35, 36)" : "BLUE ALLIANCE (Tags 39, 40, 43, 44)");
            telemetry.addData("Mecanum Drive Powers", "LF:%.2f LB:%.2f RF:%.2f RB:%.2f",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.addData("Intake Motor (G1.A/B)", "%s (%.2f)", intakeMode.name(), currentIntakePower);
            telemetry.addData("Intake Lift Arms (G1.LB)", isIntakeLiftUp ? "UP (L:%.2f | R:%.2f)" : "DOWN (L:%.2f | R:%.2f)",
                    targetLeftLiftPos, targetRightLiftPos);

            telemetry.addData("--- Vision & Shooter Status ---", "");
            if (visionSubsystem != null) {
                telemetry.addData("AprilTags in View", visionSubsystem.getTagsSeenSummary());
                if (visionSubsystem.isTargetVisible()) {
                    boolean isAimed = visionSubsystem.isAimed();
                    telemetry.addData("Limelight Target", "%s Tag #%d | tx: %.1f° | ty: %.1f° | Dist: %.1fin",
                            isAimed ? "[LOCKED ON TARGET - READY TO SHOOT!]" : "[ALIGNING...]",
                            visionSubsystem.getDetectedTagId(), visionSubsystem.getTx(), visionSubsystem.getTy(), visionSubsystem.getCalculatedDistanceInches());
                } else {
                    telemetry.addData("Limelight Target", "[SEARCHING] Looking for %s tags...", IS_RED_ALLIANCE ? "RED" : "BLUE");
                }
            }

            telemetry.addData("Turret Auto-Aim", "G2.RB Toggle: %s | Status: %s",
                    AUTO_AIM_ENABLED ? "ON" : "OFF",
                    isAutoAimEngaged ? "ACTIVE" : "OFF (Center Home)");
            telemetry.addData("Turret Position", "Target: %.1f° | Actual: %.1f° (%d ticks)", TARGET_TURRET_ANGLE_DEG, currentTurretAngle, currentTurretTicks);
            telemetry.addData("Shooter Flywheel (G2.Y)", FLYWHEEL_ENABLED ? "RUNNING (Target: %.0f RPM)" : "STOPPED", TARGET_RPM);
            telemetry.addData("Shooter Measured", "M1: %.1f RPM (Error: %.1f) | Pwr: %.0f%%", actualRPM1, errorRPM, motorPower * 100.0);

            String gateStatus;
            if (isGateOpen) {
                String openReason = isRpmReady ? "RPM READY" : "300ms TIMEOUT";
                gateStatus = String.format("OPEN & FIRING [%s] (Pos: %.2f | RPM Latched: %.0f)", openReason, targetGatePos, latchedShotRPM);
            } else if (isGatePressed) {
                gateStatus = String.format("WAITING FOR RPM... (%.0fms / 300ms | Target: %.0f | Current: %.0f)",
                        rpmWaitTimer.milliseconds(), latchedShotRPM, actualRPM1);
            } else {
                gateStatus = String.format("CLOSED (Pos: %.2f)", targetGatePos);
            }
            telemetry.addData("Axon Gate (G2.B)", gateStatus);
            telemetry.addData("Shifter Servo (G2.A)", isShifterOpen ? "OPEN / Nectar (Pos: %.2f)" : "CLOSED / Pollen (Pos: %.2f)", targetShifterPos, targetShifterPos);

            telemetry.update();
        }

        // Safe Shutdown
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        if (shooter1 != null) shooter1.setPower(0.0);
        if (shooter2 != null) shooter2.setPower(0.0);
        if (intake != null) intake.setPower(0.0);
        if (turret != null) turret.setPower(0.0);
        if (visionSubsystem != null) visionSubsystem.stop();
    }
}

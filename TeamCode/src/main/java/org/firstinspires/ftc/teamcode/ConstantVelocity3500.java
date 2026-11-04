package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "GoBilda Dual Flywheel 5000 RPM", group = "Shooter")
public class ConstantVelocity3500 extends LinearOpMode {

    // 1:1 Yellow Jacket (6000 RPM bare motor) encoder CPR = 28 ticks/rev
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM = 5000.0;
    private static final double TARGET_TICKS_PER_SEC = (TARGET_RPM / 60.0) * TICKS_PER_REV; // ~2333.33 ticks/sec

    private DcMotorEx flywheelMotor1;
    private DcMotorEx flywheelMotor2;

    @Override
    public void runOpMode() {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");

        // Uncomment to reverse Motor 2 if motors face each other in dual setup:
        // flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset and prepare encoder feedback for both motors
        flywheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Coasting allows smooth inertia decay; use BRAKE if rapid stops are required
        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Dual Flywheel Initialized. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Command closed-loop velocity via onboard REV Hub PIDF to both motors
            flywheelMotor1.setVelocity(TARGET_TICKS_PER_SEC);
            flywheelMotor2.setVelocity(TARGET_TICKS_PER_SEC);

            // Read actual telemetry back from both motors
            double currentTicksPerSec1 = flywheelMotor1.getVelocity();
            double currentRPM1 = (currentTicksPerSec1 / TICKS_PER_REV) * 60.0;

            double currentTicksPerSec2 = flywheelMotor2.getVelocity();
            double currentRPM2 = (currentTicksPerSec2 / TICKS_PER_REV) * 60.0;

            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Target Ticks/sec", TARGET_TICKS_PER_SEC);
            telemetry.addData("Motor 1 RPM", currentRPM1);
            telemetry.addData("Motor 2 RPM", currentRPM2);
            telemetry.addData("Motor 1 Ticks/sec", currentTicksPerSec1);
            telemetry.addData("Motor 2 Ticks/sec", currentTicksPerSec2);
            telemetry.update();
        }

        // Shut down both motors safely on stop
        flywheelMotor1.setVelocity(0);
        flywheelMotor2.setVelocity(0);
    }
}

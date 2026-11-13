package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Drive tuning check. Runs one loop LOOPS times and reports how far the robot ends from the start.
 *
 * One loop: forward 48 -> turn 90 anticlockwise -> forward 24 -> strafe left 48
 *           -> turn 90 clockwise -> strafe right 24.
 *
 * On paper the loop ends exactly where it started (x 0, y 0, heading 0), so the robot should return to the
 * same spot every time. Each loop is planned from the ideal start pose (0, 0, 0), so any drift that is left
 * at the end of a loop is what the feedback gains could not remove. Mark the start on the floor with tape.
 */
@Config
@Autonomous(name = "AutoTuningTest", group = "Tuning")
public class AutoTuningTest extends LinearOpMode {

    public static int LOOPS = 4;
    public static double FIRST_FORWARD_IN = 48.0;
    public static double FIRST_TURN_ANTICLOCKWISE_DEG = 90.0;
    public static double SECOND_FORWARD_IN = 24.0;
    public static double STRAFE_LEFT_IN = 48.0;
    public static double SECOND_TURN_CLOCKWISE_DEG = 90.0;
    public static double STRAFE_RIGHT_IN = 24.0;

    @Override
    public void runOpMode() {
        // Drive (Road Runner + Pinpoint). The robot's starting spot is (0, 0) facing heading 0.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addData("Status", "Ready - AutoTuningTest (%d loops)", LOOPS);
        telemetry.addData("Start", "Place the robot on the tape mark, facing forward");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        StringBuilder results = new StringBuilder();

        for (int loop = 1; loop <= LOOPS && opModeIsActive(); loop++) {
            Pose2d start = new Pose2d(0, 0, 0);

            // Positions after each move, worked out for heading 0 at the start:
            //   forward 48            -> (48, 0), heading 0
            //   turn 90 anticlockwise -> heading 90 (the robot now faces +y)
            //   forward 24            -> (48, 24)
            //   strafe left 48        -> (0, 24)   (left of a robot facing +y is -x)
            //   turn 90 clockwise     -> heading 0
            //   strafe right 24       -> (0, 0)    (right of a robot facing +x is -y)
            double afterForwardX = FIRST_FORWARD_IN;
            double afterSecondForwardY = SECOND_FORWARD_IN;
            double afterStrafeLeftX = FIRST_FORWARD_IN - STRAFE_LEFT_IN;
            double afterStrafeRightY = SECOND_FORWARD_IN - STRAFE_RIGHT_IN;

            Action path = drive.actionBuilder(start)
                    .lineToX(afterForwardX)
                    .turn(Math.toRadians(FIRST_TURN_ANTICLOCKWISE_DEG))
                    .lineToY(afterSecondForwardY)
                    .strafeTo(new Vector2d(afterStrafeLeftX, afterSecondForwardY))
                    .turn(-Math.toRadians(SECOND_TURN_CLOCKWISE_DEG))
                    .strafeTo(new Vector2d(afterStrafeLeftX, afterStrafeRightY))
                    .build();

            runAction(path);
            if (!opModeIsActive()) break;

            // Where did the robot really end up? The ideal end is x = afterStrafeLeftX, y = afterStrafeRightY, heading 0
            // (that is (0, 0, 0) with the default values).
            Pose2d end = drive.localizer.getPose();
            double xErr = end.position.x - afterStrafeLeftX;
            double yErr = end.position.y - afterStrafeRightY;
            double headingErrDeg = Math.toDegrees(end.heading.toDouble());
            // Wrap into -180..180 so a tiny error just under 360 does not show as a huge number.
            while (headingErrDeg > 180) headingErrDeg -= 360;
            while (headingErrDeg < -180) headingErrDeg += 360;

            results.append(String.format("Loop %d: x %+.2f in | y %+.2f in | heading %+.2f deg\n",
                    loop, xErr, yErr, headingErrDeg));
            telemetry.addData("Loop", "%d / %d done", loop, LOOPS);
            telemetry.addData("End error (should be 0)", "\n" + results);
            telemetry.update();
        }

        // Keep the results on the screen until the op mode is stopped.
        while (opModeIsActive()) {
            telemetry.addData("Status", "Done");
            telemetry.addData("End error per loop (should be 0)", "\n" + results);
            telemetry.update();
            sleep(50);
        }
    }

    /** Like Actions.runBlocking, but also sends the path to the Dashboard. */
    private void runAction(Action action) {
        boolean running = true;
        while (opModeIsActive() && running) {
            TelemetryPacket packet = new TelemetryPacket();
            running = action.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}

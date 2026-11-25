package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * BioBuzz Main TeleOp OpMode - Dedicated BLUE Alliance.
 * Filtered strictly to Blue Alliance AprilTags: Left (39, 40) | Right (43, 44).
 */
@Config
@TeleOp(name = "BioBuzz TeleOp - BLUE Alliance", group = "TeleOp")
public class BioBuzzTeleOpBlue extends BioBuzzMainTeleOp {

    @Override
    public void runOpMode() {
        IS_RED_ALLIANCE = false;
        super.runOpMode();
    }
}

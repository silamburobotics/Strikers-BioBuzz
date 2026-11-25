package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * BioBuzz Main TeleOp OpMode - Dedicated RED Alliance.
 * Filtered strictly to Red Alliance AprilTags: Left (31, 32) | Right (35, 36).
 */
@Config
@TeleOp(name = "BioBuzz TeleOp - RED Alliance", group = "TeleOp")
public class BioBuzzTeleOpRed extends BioBuzzMainTeleOp {

    @Override
    public void runOpMode() {
        IS_RED_ALLIANCE = true;
        super.runOpMode();
    }
}

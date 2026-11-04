package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

public interface Localizer {
    void setPose(Pose2d pose);
    Pose2d getPose();
    PoseVelocity2d update();
}

package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public final class MecanumLocalizerInputsMessage {
    public long timestamp = System.nanoTime();
    public PositionVelocityPair leftFront;
    public PositionVelocityPair leftBack;
    public PositionVelocityPair rightBack;
    public PositionVelocityPair rightFront;
    public YawPitchRollAngles angles;

    public MecanumLocalizerInputsMessage(PositionVelocityPair leftFront, PositionVelocityPair leftBack, PositionVelocityPair rightBack, PositionVelocityPair rightFront, YawPitchRollAngles angles) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.rightFront = rightFront;
        this.angles = angles;
    }
}

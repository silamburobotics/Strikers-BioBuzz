package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;

public final class DriveCommandMessage {
    public long timestamp = System.nanoTime();
    public double forward;
    public double side;
    public double turn;

    public DriveCommandMessage(PoseVelocity2dDual<Time> poseVelocity) {
        this.forward = poseVelocity.linearVel.x.value();
        this.side = poseVelocity.linearVel.y.value();
        this.turn = poseVelocity.angVel.value();
    }
}

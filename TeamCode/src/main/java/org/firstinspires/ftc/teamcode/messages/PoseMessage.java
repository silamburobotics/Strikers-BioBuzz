package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.Pose2d;

public final class PoseMessage {
    public long timestamp = System.nanoTime();
    public double x;
    public double y;
    public double heading;

    public PoseMessage(Pose2d pose) {
        this.x = pose.position.x;
        this.y = pose.position.y;
        this.heading = pose.heading.toDouble();
    }
}

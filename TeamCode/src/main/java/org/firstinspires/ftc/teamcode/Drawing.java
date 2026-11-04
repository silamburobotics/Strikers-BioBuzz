package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;

public class Drawing {
    public static void drawRobot(Canvas c, Pose2d t) {
        c.setStroke("#3F51B5");
        c.strokeCircle(t.position.x, t.position.y, 9);
        c.strokeLine(t.position.x, t.position.y,
                t.position.x + 9 * Math.cos(t.heading.toDouble()),
                t.position.y + 9 * Math.sin(t.heading.toDouble()));
    }
}

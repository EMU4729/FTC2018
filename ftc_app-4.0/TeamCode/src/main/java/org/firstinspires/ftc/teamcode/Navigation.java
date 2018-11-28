package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Navigation {
    public Tracking tracking;
    private HardwareMap hardwareMap;
    private Motors motors;

    public Navigation(HardwareMap hardwareMap, Motors motors) {
        this.hardwareMap = hardwareMap;
        this.motors = motors;
    }

    public void init() {
        tracking = new Tracking(hardwareMap, motors);
        tracking.init();
    }

    public void run() {
        tracking.run();
    }

    public double[] navigate(double x, double y) {
        double move = 0;
        double turn = 0;

        final double speedScalingThreshold = 1000;
        final double angleScalingThreshold = 45;
        final double moveForwardThreshold = 10;

        double distance = getDistance(x, y);
        double angle = -Math.toDegrees(Math.atan2(-(y-tracking.y), x-tracking.x)) + 90;
        double distanceAngle = angleDifference(angle, tracking.rotation);

        if (Math.abs(distanceAngle) < moveForwardThreshold) {
            move = Math.min(speedScalingThreshold, distance) / speedScalingThreshold;
        } else {
            if (Math.abs(distanceAngle) > angleScalingThreshold) {
                turn = Math.signum(distanceAngle);
            } else {
                turn = distanceAngle / angleScalingThreshold;
            }
        }

        double[] result = {move, turn};
        return result;
    }

    public double angleDifference(double angle1, double angle2) {
        if (angle1 > 360) angle1 -= 360;
        if (angle1 < 0) angle1 += 360;
        if (angle2 > 360) angle2 -= 360;
        if (angle2 < 0) angle2 += 360;
        double difference = angle2 - angle1;
        if (Math.abs(difference) > 180) difference = (360 - Math.abs(difference)) * Math.signum(-difference);
        return difference;
    }

    public double getDistance(double x, double y) {
        double xDistance = x - tracking.x;
        double yDistance = y - tracking.y;

        double distance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        return distance;
    }
}

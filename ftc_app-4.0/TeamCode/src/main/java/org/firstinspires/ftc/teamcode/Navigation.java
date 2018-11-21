package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Navigation {
    public Tracking tracking;
    private HardwareMap hardwareMap;

    public Navigation(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        tracking = new Tracking(hardwareMap);
        tracking.init();
    }

    public void run() {
        tracking.run();
    }

    public double[] navigate(double x, double y) {
        double move = 0;
        double turn = 0;

        final double speedScalingThreshold = 1000;
        final double angleScalingThreshold = 10;

        double distance = getDistance(x, y);
        double angle = Math.toDegrees(Math.atan2(x, y));

        if (angleDifference(angle, tracking.rotation) < 1) {
            move = Math.min(speedScalingThreshold, distance) / speedScalingThreshold;
//        } else if (angleDifference(angle, tracking.rotation) < 45) {
//            move = Math.min(speedScalingThreshold, distance) / speedScalingThreshold;
//            turn = Math.min(angleScalingThreshold, Math.abs(angle)) / angleScalingThreshold * Math.signum(angle);
        } else {
            turn = Math.min(angleScalingThreshold, Math.abs(angle)) / angleScalingThreshold * Math.signum(angle);
        }

        double[] result = {move, turn};
        return result;
    }

    public double angleDifference(double angle1, double angle2) {
        if (angle1 > 180) {
            angle1 -= 360;
        }
        if (angle2 > 180) {
            angle2 -= 360;
        }

        return Math.abs(angle1 - angle2);
    }

    public double getDistance(double x, double y) {
        double xDistance = x - tracking.x;
        double yDistance = y - tracking.y;

        double distance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        return distance;
    }
}

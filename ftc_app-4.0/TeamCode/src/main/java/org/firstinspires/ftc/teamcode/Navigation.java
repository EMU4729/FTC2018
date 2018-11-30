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

    public double[] go(double x, double y) {
        double power = 0;
        double turn = 0;

        final double angleMoveThreshold = 10; // Angle difference between current and target at which move begins
        final double speedScalingThreshold = 1000; // Distance to target at which move starts slowing down
        final double angleScalingThreshold = 45; // Angle difference between current and target at which turn starts slowing down

        double distance = distanceTo(x, y);
        double angle = -Math.toDegrees(Math.atan2(-(y-tracking.y), x-tracking.x)) + 90;
        double angleDifference = calculateAngleDifference(angle, tracking.rotation);

        if (Math.abs(angleDifference) < angleMoveThreshold) {
            if (distance > speedScalingThreshold) {
                power = Math.signum(distance);
            } else {
                power = distance / speedScalingThreshold;
            }
        }

        if (Math.abs(angleDifference) > angleScalingThreshold) {
            turn = Math.signum(angleDifference);
        } else {
            turn = angleDifference / angleScalingThreshold;
        }

        motors.arcadeDrive(power, turn);

        double[] result = {power, turn};
        return result;
    }

    public void stop() {
        motors.arcadeDrive(0, 0);
    }

    public double distanceTo(double x, double y) {
        return Math.sqrt(Math.pow(x - tracking.x, 2) + Math.pow(y - tracking.y, 2));
    }

    private double calculateAngleDifference(double angle1, double angle2) {
        if (angle1 > 360) angle1 -= 360;
        if (angle1 < 0) angle1 += 360;
        if (angle2 > 360) angle2 -= 360;
        if (angle2 < 0) angle2 += 360;
        double difference = angle2 - angle1;
        if (Math.abs(difference) > 180) difference = (360 - Math.abs(difference)) * Math.signum(-difference);
        return -difference;
    }
}

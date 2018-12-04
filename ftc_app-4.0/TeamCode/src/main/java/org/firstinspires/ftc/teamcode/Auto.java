package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Auto
{
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private Motors motors;
    private Mechanism mechanism;
    private Navigation navigation;
    private Tracking tracking;

    private ElapsedTime time = new ElapsedTime();

    private final double distanceThreshold = 500; // mm
    private static final double ROBOT_FIELD = 3657.6; // mm
    private double squareWidth = ROBOT_FIELD / 6;//596.9; // mm
    private int autoStage = 1;

    private double ARM_TIME = 2000;
    private double OUTTAKE_TIME = 2000;

    private double ARM_POWER = 0.5;
    private double ELBOW_POWER = 0.5;

    private double[][] path;
    private double[] startRotations = {
            315,
            225,
            45,
            135
    };

    private final double[][][] paths = {
        { // blue top
            {squareWidth * 2.5, squareWidth * 2.5},
            {squareWidth * 0.5, squareWidth * 2.5},
            {squareWidth * 0.5, squareWidth * 5.5}
        },
        { // blue bottom
            {squareWidth * 2.5, squareWidth * 3.5},
            {squareWidth * 1, squareWidth * 5.5},
            {squareWidth * 1, squareWidth * 5.5}
        },
        { // red top
            {squareWidth * 3.5, squareWidth * 2.5},
            {squareWidth * 3.5, squareWidth * 1},
            {squareWidth * 5.5, squareWidth * 1}
        },
        { // red bottom
            {squareWidth * 3.5, squareWidth * 3.5},
            {squareWidth * 5, squareWidth * 3.5},
            {squareWidth * 5, squareWidth * 0.5}
        }
    };

    public Auto(HardwareMap hardwareMap, Telemetry telemetry, int startPosition) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        telemetry.addData("Status", "Auto ready");
        motors = new Motors(hardwareMap);
        mechanism = new Mechanism(hardwareMap);
        navigation = new Navigation(hardwareMap, motors);
        path = paths[startPosition];
        navigation.init(path[0][0], path[0][1], startRotations[startPosition]);
        tracking = navigation.tracking;
    }

    public void run() {
        navigation.run();
        telemetry.addData("X", tracking.x);
        telemetry.addData("Y", tracking.y);
        telemetry.addData("Z", tracking.z);
        telemetry.addData("Rotation", tracking.rotation);
        telemetry.addData("AutoStage", autoStage);

        // go to box
        if (autoStage < path.length) {
            double[] output = navigation.go(path[autoStage][0], path[autoStage][1]);
            double power = output[0];
            double turn = output[1];

            telemetry.addData("Power", power);
            telemetry.addData("Turn", turn);

            if (navigation.distanceTo(path[autoStage][0], path[autoStage][1]) < distanceThreshold) {
                navigation.stop();
                autoStage += 1;
                if (autoStage >= path.length) time.reset();
            }
        } else if (autoStage >= path.length) {
            if (time.milliseconds() <= ARM_TIME) {
                mechanism.armPower(ARM_POWER);
                mechanism.elbowPower(ELBOW_POWER);
            } else if (time.milliseconds() <= ARM_TIME + OUTTAKE_TIME) {
                mechanism.manipulatorPower(1);
            } else {
                mechanism.manipulatorPower(0);
            }
        }
    }

    public void stop() {
        navigation.stop();
        mechanism.manipulatorPower(0);
    }
}

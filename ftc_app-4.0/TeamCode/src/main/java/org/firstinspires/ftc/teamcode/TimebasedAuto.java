/**
 *  This is code just in case vision doesn't work we will use this
 *  as it is just time-based auto.
 *
 *  TimeBasedAuto.java
 **/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TimebasedAuto extends OpMode {
    private Motors motors;
    private ElapsedTime runtime = new ElapsedTime();
    private Grabber grabber;

    private static final double ROBOT_FIELD = 358.14; //cm
    private static final double ROBOT_FIELD_HALF = 179.07; //cm
    private double squareWidth = 596.9; //mm

    private final double distanceThreshold = 100; //mm

    @Override
    public void init() {
        telemetry.addData("Status", "Auto ready");
        motors = new Motors(hardwareMap);
        grabber = new Grabber(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        runtime.startTime();
    }

    @Override
    public void loop() {
        forward();
        if (runtime.time() == 10){
            turnLeft();
        }
        if (runtime.time() == 15) {
            forward();
        }
        if (runtime.time() == 24) {
            //drop thing in lander
        }
    }

    private void forward() {
        motors.arcadeDrive(1, 0);
    }

    private void turnLeft() {
        motors.arcadeDrive(0, -1);
    }

    private void turnRight() {
        motors.arcadeDrive(0, 1);
    }

    private void stopMotors() {
        motors.arcadeDrive(0,0);
    }

    @Override
    public void stop() {
        stopMotors();
        grabber.stopSpin();
    }
}

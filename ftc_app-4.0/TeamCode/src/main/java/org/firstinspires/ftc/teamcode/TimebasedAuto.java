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

    private static final double START = 5;
    private static final double GET_READY_TO_TURN = 10;
    private static final double GOING_TOWARDS_DEPOT = 15;
    private static final double DROPPING_THING = 24;




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
        if (runtime.time() <= START) {
            forward();
        } else if (runtime.time() <= GET_READY_TO_TURN){
            turnLeft();
        } else if (runtime.time() <= GOING_TOWARDS_DEPOT) {
            forward();
        } else if (runtime.time() <= DROPPING_THING) {
            stop();
            //drop thing in depot
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

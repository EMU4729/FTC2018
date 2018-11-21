package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto", group="Iterative Opmode")
public class Auto extends OpMode
{
    private Motors motors;
    private ElapsedTime runtime = new ElapsedTime();
    private Grabber grabber;
    private Navigation navigation;
    private Tracking tracking;

    private static final double ROBOT_FIELD = 365.76; //cm
    private static final double ROBOT_FIELD_HALF = ROBOT_FIELD / 2; //cm
    private double squareWidth = 596.9; //mm

    private final double[][] blueTop = {{squareWidth*0.5, squareWidth*2.5}, {squareWidth*0.5, squareWidth*5.5}};
    private final double[][] blueBottom = {{squareWidth*2.5, squareWidth*5.5}, {squareWidth*0.5, squareWidth*5.5}};
    private final double[][] redTop = {{squareWidth*3.5, squareWidth*0.5}, {squareWidth*5.5, squareWidth*0.5}};
    private final double[][] redBottom = {{squareWidth*5.5, squareWidth*3.5}, {squareWidth*5.5, squareWidth*0.5}};
    private double[][] actualPosition;

    private int autoStage = 0;

    private final double distanceThreshold = 100; //mm

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Auto ready");
        motors = new Motors(hardwareMap);
        grabber = new Grabber(hardwareMap);
        navigation = new Navigation(hardwareMap, motors);
        actualPosition = blueTop;
        navigation.init();
        tracking = navigation.tracking;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();

//        release(); //release from wall
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        navigation.run();
        telemetry.addData("X", tracking.x);
        telemetry.addData("Y", tracking.y);
        telemetry.addData("Z", tracking.z);
        telemetry.addData("Rotation", tracking.rotation);

        telemetry.addData("AutoStage", autoStage);

        //go to box
        if (autoStage < actualPosition.length) {
            double[] output = navigation.navigate(actualPosition[autoStage][0], actualPosition[autoStage][1]);
            double power = output[0];
            double turn = output[1];

            telemetry.addData("Power", power);
            telemetry.addData("Turn", turn);

            motors.arcadeDrive(power, turn);
            if (navigation.getDistance(actualPosition[autoStage][0], actualPosition[autoStage][1]) < distanceThreshold) {
                stopMotors();
                autoStage += 1;
            }
        } else if (autoStage <= actualPosition.length) {
            grabber.outake();
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

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        stopMotors();
        grabber.stopSpin();
    }
}

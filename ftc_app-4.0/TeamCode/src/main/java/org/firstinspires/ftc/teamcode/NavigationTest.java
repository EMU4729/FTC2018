package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Navigation Test", group="Iterative Opmode")
public class NavigationTest extends OpMode
{
    private Motors motors;
    private Navigation navigation;
//    private Tracking tracking;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        motors = new Motors(hardwareMap);
        navigation = new Navigation(hardwareMap, motors);
        navigation.init();
//        tracking = new Tracking(hardwareMap, motors);
//        tracking.init();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {}

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        navigation.run();
//        motors.arcadeDrive(1, 0);
        telemetry.addData("x", navigation.tracking.x);
        telemetry.addData("y", navigation.tracking.y);
        telemetry.addData("z", navigation.tracking.z);
//        telemetry.addData("Square x", tracking.x/596.9);
//        telemetry.addData("Square y", tracking.y/596.9);
        telemetry.addData("rotation", navigation.tracking.rotation);
        double[] result = navigation.navigate(0, 1828);
        double move = result[0];
        double turn = result[1];
        telemetry.addData("Move", move);
        telemetry.addData("Turn", turn);
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        motors.arcadeDrive(0, 0);
    }
}

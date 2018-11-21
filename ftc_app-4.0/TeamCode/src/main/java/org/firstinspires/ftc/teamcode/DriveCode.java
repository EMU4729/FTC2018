package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class DriveCode extends OpMode
{
    Motors motors;
    Tracking trackingTest;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Tele ready");
        motors = new Motors(hardwareMap);
        trackingTest = new Tracking(hardwareMap);
        trackingTest.init();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        //leds
    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
//        leftPower  = -gamepad1.left_stick_y ;
//        rightPower = -gamepad1.right_stick_y ;

        // Show the elapsed game time and wheel power.
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        trackingTest.run();
        telemetry.addData("vision", trackingTest.vision);
        telemetry.addData("x, y, z, rotation", "%.2f, %.2f, %.2f, %.2f",
                trackingTest.x, trackingTest.y, trackingTest.z, trackingTest.rotation);
        telemetry.addData("ax, ay, az", "%.2f, %.2f, %.2f",
                trackingTest.ax, trackingTest.ay, trackingTest.az);
        telemetry.addData("vx, vy, vz", "%.2f, %.2f, %.2f",
                trackingTest.vx, trackingTest.vy, trackingTest.vz);
        telemetry.addData("gx, gy, gz", "%.2f, %.2f, %.2f",
                trackingTest.gx, trackingTest.gy, trackingTest.gz);

        motors.arcadeDrive(drive, turn);
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        motors.arcadeDrive(0,0);
    }

}

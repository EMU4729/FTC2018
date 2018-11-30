package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Teleoperated", group="Iterative Opmode")
public class Teleoperated extends OpMode
{
    Motors motors;
    Mechanism mechanism;

    double deadzone = 0.2;

    /*
     * Code to run ONCE when the driver hits INIT
     * This is for debugging tracking, luke. go it.
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Tele ready");
        motors = new Motors(hardwareMap);
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
    public void start() {}

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        double forwards = map(gamepad1.left_stick_y);
        double turn = map(gamepad1.right_stick_x);
        motors.arcadeDrive(forwards, turn);
        mechanism.armPower(map(gamepad2.left_stick_y));
        mechanism.elbowPower(map(gamepad2.right_stick_y));
        if (gamepad2.b) {
            mechanism.intake();
        } else if (gamepad2.y) {
            mechanism.outtake();
        }
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        motors.arcadeDrive(0,0);
    }

    private double map(double input) {
        double sign = Math.signum(input);
        input = Math.abs(input);
        double value = (input - deadzone) / (1 - deadzone);
        if (value < 0) value = 0;
        return value * sign;
    }
}

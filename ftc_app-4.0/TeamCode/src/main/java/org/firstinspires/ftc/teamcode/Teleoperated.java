package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name="Teleoperated", group="Iterative Opmode")
public class Teleoperated extends OpMode
{
    Motors motors;
    Mechanism mechanism;

    double deadzone = 0.2;
    private boolean twoController;

    /*
     * Code to run ONCE when the driver hits INIT
     * This is for debugging tracking, luke. go it.
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Tele ready");
        motors = new Motors(hardwareMap);
        mechanism = new Mechanism(hardwareMap);
        twoController = false;
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
        double forwards;
        double turn;
        if (twoController) {
            motors.arcadeDrive(map(-gamepad1.left_stick_y), map(gamepad1.right_stick_x));
            mechanism.armPower(map(-gamepad2.left_stick_y));
            mechanism.elbowPower(map(-gamepad2.right_stick_y));
            mechanism.manipulatorPower(digital(gamepad2.y, gamepad2.b));
        } else {
            motors.arcadeDrive(map(-gamepad1.left_stick_y), map(gamepad1.left_stick_x));
            mechanism.armPower(map(-gamepad1.right_stick_y));
            mechanism.elbowPower(digital(gamepad1.x, gamepad1.a));
            mechanism.manipulatorPower(digital(gamepad1.y, gamepad1.b));
        }

        telemetry.addData("forwards, turn", "%.2f, %.2f", motors.forwards, motors.turn);
        telemetry.addData("left, right", "%.2f, %.2f", motors.leftPower, motors.rightPower);
        telemetry.addData("arm, elbow, manipulator", "%.2f, %.2f, %.2f", mechanism.armPower, mechanism.elbowPower, mechanism.manipulatorPower);
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        motors.arcadeDrive(0,0);
    }

    private double digital(boolean up, boolean down) {
        double power = 0;
        if (up ^ down) {
            if (up) power = 1;
            if (down) power = -1;
        }
        return power;
    }

    private double map(double input) {
        double sign = Math.signum(input);
        input = Math.abs(input);
        double value = (input - deadzone) / (1 - deadzone);
        if (value < 0) value = 0;
        return value * sign;
    }
}

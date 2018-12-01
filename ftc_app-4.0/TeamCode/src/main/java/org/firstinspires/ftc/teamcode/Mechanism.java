/**
 * This is used for grabbing the minerals and placing them
 **/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Mechanism {
    private DcMotor arm1;
    private DcMotor arm2;
    private DcMotor elbow;
    private DcMotor manipulator;

    public double armPower;
    public double elbowPower;
    public double manipulatorPower;

    private final double ARM_MIN_SPEED = 0.2;
    private final double ARM_MAX_AT = 0.8;
    private final double ELBOW_MIN_SPEED = 0.2;
    private final double ELBOW_MAX_AT = 0.8;
    private final double INTAKE_SPEED = 1;
    private final double OUTTAKE_SPEED = 1;

    public Mechanism(HardwareMap hardwareMap){
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        manipulator = hardwareMap.get(DcMotor.class, "manipulator");
    }

    public void armPower(double power) {
        armPower = transform(power, ARM_MIN_SPEED, ARM_MAX_AT);
        arm1.setPower(power);
        arm2.setPower(power);
    }

    public void elbowPower(double power) {
        elbowPower = transform(power, ELBOW_MIN_SPEED, ELBOW_MAX_AT);
        elbow.setPower(power);
    }

    public void manipulatorPower(double power) {
        if (power > 0) {
            power *= OUTTAKE_SPEED;
        } else {
            power *= INTAKE_SPEED;
        }
        manipulatorPower = power;
        manipulator.setPower(power);
    }

    private double transform(double input, double min, double maxAt) {
        double sign = Math.signum(input);
        input = Math.abs(input);
        input = input * (1 - min) + min;
        input = Math.min(input / maxAt, 1);
        input *= sign;
        return input;
    }
}
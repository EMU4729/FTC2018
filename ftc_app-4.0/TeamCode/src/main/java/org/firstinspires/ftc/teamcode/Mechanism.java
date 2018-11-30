/**
 * This is used for grabbing the minerals and placing them
 **/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Mechanism {
    private DcMotor arm;
    private DcMotor elbow;
    private DcMotor manipulator;
    private static final double INTAKE_SPEED = 1;
    private static final double OUTTAKE_SPEED = -1;

    public Mechanism(HardwareMap hardwareMap){
        arm = hardwareMap.get(DcMotor.class, "arm");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        manipulator = hardwareMap.get(DcMotor.class, "manipulator");
    }

    public void armPower(double power) {
        arm.setPower(power);
    }

    public void elbowPower(double power) {
        elbow.setPower(power);
    }

    public void manipulatorPower(double power) {
        manipulator.setPower(power);
    }

    public void intake() {
        manipulatorPower(INTAKE_SPEED);
    }

    public void outtake() {
        manipulatorPower(OUTTAKE_SPEED);
    }

    public void stopManipulator() {
        spin(0);
    }
}
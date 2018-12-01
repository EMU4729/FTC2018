package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Motors {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    public double forwards;
    public double turn;
    public double leftPower;
    public double rightPower;

    public Motors(HardwareMap hardwareMap) {
//        Log.i("class", DcMotor.class.getName());
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void arcadeDrive(double forwards, double turn) {
        this.forwards = forwards;
        this.turn = turn;
        //setting speed for left and right side of robot
        leftPower = Range.clip(forwards + turn, -1.0, 1.0);
        rightPower = Range.clip(forwards - turn, -1.0, 1.0);

        //getting it on robot
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}

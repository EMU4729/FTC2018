package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Motors {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    public Motors(HardwareMap hardwareMap) {
//        Log.i("class", DcMotor.class.getName());
        leftDrive  = null;//hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = null;//hardwareMap.get(DcMotor.class, "right_drive");

        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void arcadeDrive(double forwards, double turn) {
        double leftPower;
        double rightPower;

        leftPower = Range.clip(forwards + turn, -1.0, 1.0);
        rightPower = Range.clip(forwards - turn, -1.0, 1.0);

        // leftDrive.setPower(leftPower);
        // rightDrive.setPower(rightPower);
    }
}

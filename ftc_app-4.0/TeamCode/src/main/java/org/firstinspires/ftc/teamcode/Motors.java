package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Motors {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    public Motors(HardwareMap hardwareMap) {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void arcadeDrive(double forwards, double turn) {
        double leftPower;
        double rightPower;

        leftPower = Range.clip(forwards + turn, -1.0, 1.0) ;
        rightPower = Range.clip(forwards - turn, -1.0, 1.0) ;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(right);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Grabber {

    public DcMotor roller = null;

    public Grabber(HardwareMap hardwareMap){
        roller  = hardwareMap.get(DcMotor.class, "left_drive");

        roller.setDirection(DcMotor.Direction.REVERSE);
    }

    public void spin(double speed) {

        leftPower = Range.clip(forwards + turn, -1.0, 1.0) ;
        rightPower = Range.clip(forwards - turn, -1.0, 1.0) ;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(right);
    }
}
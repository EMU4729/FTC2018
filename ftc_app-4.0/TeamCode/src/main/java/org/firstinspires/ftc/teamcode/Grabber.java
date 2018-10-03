package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Grabber {

    public DcMotor roller;
    private static final double INTAKE_SPEED = 1;
    private static final double OUTAKE_SPEED = -1;

    public Grabber(HardwareMap hardwareMap){
        roller = hardwareMap.get(DcMotor.class, "left_drive");

        roller.setDirection(DcMotor.Direction.REVERSE);
    }

    private void spin(double speed) {

        double rollerPower = Range.clip(speed, -1.0, 1.0);

        roller.setPower(rollerPower);
    }

    public void intake() {
        spin(INTAKE_SPEED);
    }

    public void outake() {
        spin(OUTAKE_SPEED);
    }
}
///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Autonomous(name="Auto", group="Iterative Opmode")
//@Disabled
//public class Auto extends OpMode
//{
//    private Motors motors;
//    private Tracking tracking;
//    private ElapsedTime runtime = new ElapsedTime();
//    private Navigation navigation;
//    private static final double ROBOT_FIELD = 358.14; //cm
//    private static final double ROBOT_FIELD_HALF = 179.07; //cm
//
//    private final double[][] blueTop = [[], []];
//    private final double[][] blueBottom = [[], []];
//    private final double[][] redTop = [[], []];
//    private final double[][] redBottom = [[], []];
//    private double[][] actualPosition;
//
//    private double autoStage = 0;
//
//    private const double distanceThreshold = 100; //mm
//
//    /*
//     * Code to run ONCE when the driver hits INIT
//     */
//    @Override
//    public void init() {
//        telemetry.addData("Status", "Auto ready");
//        tracking = new Tracking();
//        tracking.init();
//        motors = new Motors();
//        navigation = new Navigation();
//        actualPosition = blueTop;
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//     */
//    @Override
//    public void init_loop() {
//        //leds
//    }
//
//    //Code to run ONCE when the driver hits PLAY
//    @Override
//    public void start() {
//        runtime.reset();
//
//        release(); //release from wall
//    }
//
//    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
//    @Override
//    public void loop() {
//        tracking.run();
//        telemetry.addData("X", tracking.x);
//        telemetry.addData("Y", tracking.y);
//        telemetry.addData("Z", tracking.z);
//        telemetry.addData("Rotation", tracking.rotation);
//
//        //go to box
//        if (tracking.available) {
//            if (autoStage < actualPosition.length) {
//                double[] output = navigation.navigate(actualPosition[autoStage][0], actualPosition[autoStage][1]);
//                double power = output[0];
//                double turn = output[1];
//
//                motors.arcadeDrive(power, turn);
//                if (navigation.getDistance < distanceThreshold) {
//                    stop();
//                    autoStage += 1;
//                }
//            }
//        } else {
//            stop();
//        }
//    }
//
//    private void forward() {
//        motors.arcadeDrive(1, 0);
//    }
//
//    private void turnLeft() {
//        motors.arcadeDrive(0, -1);
//    }
//
//    private void turnRight() {
//        motors.arcadeDrive(0, 1);
//    }
//
//    private void stopMotors() {
//        motors.arcadeDrive(0, 0);
//    }
//
//    // Code to run ONCE after the driver hits STOP
//    @Override
//    public void stop() {
//        motors.arcadeDrive(0,0);
//    }
//}

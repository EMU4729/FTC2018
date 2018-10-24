/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

public class AutoNavigation {
    private Tracking tracking;

    public void init() {
        tracking = new Tracking();
        tracking.init();
    }

    public void run() {
        tracking.run()
    }

    public double[] navigate(double x, double y) {
        double move = 0;
        double turn = 0;

        const double speedScalingThreshold = 1000;
        const double angleScalingThreshold = 45;

        double distance = getDistance(x, y);
        double angle = Math.toDegrees(Math.atan2(xDistance, yDistance));

        if (angleDifference(angle, tracking.angle) < 1) {
            move = Math.min(speedScalingThreshold, distance)/speedScalingThreshold;
        } else if (angleDifference(angle, tracking.angle) < 45) {
            move = Math.min(speedScalingThreshold, distance)/speedScalingThreshold;
            turn = Math.min(angleScalingThreshold, angle)/angleScalingThreshold;
        } else {
            turn = Math.min(angleScalingThreshold, angle)/angleScalingThreshold;
        }

        return [move, turn];
    }

    public double angleDifference(double angle1, double angle2) {
        if (angle1 > 180) {
            angle1 -= 360;
        }
        if (angle2 > 180) {
            angle2 -= 360;
        }

        return Math.abs(angle1-angle2):
    }

    public double getDistance(double x, double y) {
        double xDistance = x-tracking.x;
        double yDistance = y-tracking.y;

        double distance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        return distance;
    }
}

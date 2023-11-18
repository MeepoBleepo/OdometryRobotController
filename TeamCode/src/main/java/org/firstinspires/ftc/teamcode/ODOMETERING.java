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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="ODOMETERING") //, group="Linear Opmode"
@Disabled
public class ODOMETERING extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Motors
    private DcMotor deadwheelRobotX = null;
    private DcMotor deadwheelRobotY = null;
    private DcMotor deadwheelRobotR = null;


    private double posX = 0; // Horizontal value of the robot on the field
    private double posY = 0; // Vertical value of the robot on the field
    private double locX = 0; // Horizontal value of the robot relative to its current rotation and location
    private double locY = 0; // Vertical value of the robot relative to its current rotation and location
    private double robotAngle = 0; // Degrees, rotation of the robot on the field. Robot always starts at 0
    private double locRobotAngle = 0; // Degrees, rotation of the robot relative to its starting rotation

    //Ratios
    private double DISTANCE_TO_ROTATION_WHEEL = 0; // CHANGE THIS VARIABLE - measure the distance from your robot's CENTER OF ROTATION to the center of the rotational dead wheel. Be as accurate as possible. Like seriously. If this measurement is inaccurate then your odometry will also be inaccurate
    private double TICKS_PER_ROTATION = 0;
    private double WHEEL_CIRCUMFERENCE = 0;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // put initialization code here

        deadwheelRobotX = hardwareMap.get(DcMotor.class, "deadwheelRobotX");
        deadwheelRobotY = hardwareMap.get(DcMotor.class, "deadwheelRobotY");
        deadwheelRobotR = hardwareMap.get(DcMotor.class,"deadwheelRobotR");

        while (opModeIsActive()) {
            // run until the end of the match (driver presses STOP)




            telemetry.update();
        }
    }
}

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

import java.lang.Math;

/*
For new coders:

This class is for the easy setup and use of a Mecanum drive.
To use it, simply import "org.firstinspires.ftc.teamcode.MecanumDrive" and make a new object in RunOpMode.

The initialization arguments for this class go as follows:
    Front left motor, front right motor, back right motor, back left motor.

The method run() will allow you to input your joystick values and
a speed multiplier from 0 to 1 (in case you want your robot to go slow for some reason)
    Its arguments go: left joystick x value, left joystick y value, right joystick x value, speed multiplier.

Example:
    MecanumDrive drive = new MecanumDrive(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);
    drive.run(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 0.45);

Other things:
The method toggleTelemetry can be used to turn motor power telemetry data on or off. It is on by default.
If you want to know the math behind a mecanum drive: https://seamonsters-2605.github.io/archive/mecanum/
 */


@TeleOp(name="Basic: Linear OpMode") //, group="Linear Opmode"
@Disabled
public class MecanumDrive extends LinearOpMode {

    // Fields
    private boolean telemetryEnabled = true; // Whether to output telemetry data or not

    private final double PI = Math.PI;

    private double angle; // This stores the angle that the joystick produces (in radians).
    private double magnitude; // This is a multiplier for the wheel speed that compensates for strafing diagonally.
    private double flbrPower, frblPower;
    private double powerFL, powerFR, powerBL, powerBR; // Used for storing individual powers so that they can be returned

    // Motors
    private DcMotor motorFL, motorFR, motorBL, motorBR = null;

    // Constructors
    public MecanumDrive(DcMotor mFL, DcMotor mFR, DcMotor mBL, DcMotor mBR)
    {
        motorFL = mFL;
        motorFR = mFR;
        motorBL = mBL;
        motorBR = mBR;
    }

    // Methods
    public void toggleTelemetry()
    {
        if (telemetryEnabled) {telemetryEnabled = false;}
        else {telemetryEnabled = true;}
    }

    public void run(double x, double y, double turn, double speedMultiplier) // This method takes your joystick inputs and a speed multiplier and spins the motors in the correct order.
    {
        // Find angle and magnitude
        angle = Math.atan2(y,x); // Find the angle in radians that the joystick makes
        magnitude = Math.sqrt(Math.pow(x,2) + Math.pow(y,2)); // Finds the magnitude, which allows the robot to go slower or faster based on how far the stick is pushed

        // Find diagonal wheel powers
        flbrPower = Math.sin(angle + .25 * PI); // These equations input the desired angle and output a power value for the wheels based on overlapping sin waves.
        frblPower = Math.sin(angle - .25 * PI); // You can view these sin waves in Desmos with the equations f(x) = sin(x+.25*pi) and f(x) = sin(x-.25*pi)

        // Adding turning, magnitude, and slowing
        flbrPower = flbrPower * magnitude * speedMultiplier + turn;
        frblPower = frblPower * magnitude * speedMultiplier + turn;

        //Scaling (Motor powers can't exceed 1, so this checks to see if any power values are larger than 1 and scales all of the power values so that the highest power value is 1.
        if (Math.abs(flbrPower) > 1)
        {
            flbrPower /= Math.abs(flbrPower);
            frblPower /= Math.abs(flbrPower);
        }

        if (Math.abs(frblPower) > 1)
        {
            flbrPower /= Math.abs(frblPower);
            frblPower /= Math.abs(frblPower);
        }

        //Set powers to motors
        powerFL = flbrPower;
        powerFR = frblPower;
        powerBL = frblPower;
        powerBR = flbrPower;

        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);

        // Telemetry
        if (telemetryEnabled)
        {
            telemetry.addData("Front Left Motor Power: ", powerFL);
            telemetry.addData("Front Right Motor Power: ", powerFR);
            telemetry.addData("Back Left Motor Power: ", powerBL);
            telemetry.addData("Back Right Motor Power: ", powerBR);
        }
    }

    public void runOpMode(){} // This line does nothing, but don't delete it (it fixes an error)
}
/* Copyright (c) 2023 FIRST. All rights reserved.
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




package org.firstinspires.ftc.teamcode.Auto.Tests;




import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;




import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;




import java.util.List;
import java.util.Vector;


public class TTT extends LinearOpMode {
    DcMotor rightBackDrive, rightFrontDrive, leftFrontDrive, leftBackDrive, kickstand;



    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

//    public class ShooterConstants {
//        // coordinate goal variable for red and mirror it with variable for blue
//        public double Score_Height = 26;
//        public double Score_Angle = Math.toRadians(30);
//        public double Pass_Through_Point = 5;
//
//
//    }

public static double flywheelspeed(double goaldist) {

    return 0; // :(((((((((( import math functions
}


    @Override
    public void runOpMode() {



    }

            void mecanumDrive ( double forward, double strafe, double rotate){




                /* the denominator is the largest motor power (absolute value) or 1
                 * This ensures all the powers maintain the same ratio,
                 * but only if at least one is out of the range [-1, 1]
                 */
                double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);


                leftFrontPower = (forward + strafe + rotate) / denominator;
                rightFrontPower = (forward - strafe - rotate) / denominator;
                leftBackPower = (forward - strafe + rotate) / denominator;
                rightBackPower = (forward + strafe - rotate) / denominator;


                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);


            }
        }

























































































// end method telemetryAprilTag()












// end class





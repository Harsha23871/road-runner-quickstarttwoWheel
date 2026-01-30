package org.firstinspires.ftc.teamcode.TeleOp.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class VelocityTest extends LinearOpMode {
    DcMotorEx motor;
    ElapsedTime TestTimer = new ElapsedTime();
    double currentVelocity;
    double maxVelocity = 0.0;
    private double kP = 5.7;
    private double kI = 0;
    private double kD = 0.1;
    private double kF = 2;


    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "outtake");
        motor.setDirection(DcMotorEx.Direction.FORWARD);


        waitForStart();
        TestTimer.reset();



        while (opModeIsActive()) {
            int TargetVelocity = 1300*28/60;


            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(kP, kI, kD, kF));
            motor.setVelocity(TargetVelocity);
            currentVelocity = motor.getVelocity()*60/28;
            /*if (TestTimer.milliseconds() > 500) {*/


                telemetry.addData("current velocity", currentVelocity);
                telemetry.addData("maximum velocity", maxVelocity);
                telemetry.update();
              //  TestTimer.reset();
            //}


//            motor.setPower(1);
        }
    }
}

















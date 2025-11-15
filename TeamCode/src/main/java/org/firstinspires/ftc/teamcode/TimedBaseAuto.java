package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name="Team4008Auto1ParkingRed", group="4008")
public class TimedBaseAuto extends LinearOpMode{

   // Team4008HM2025 robot = new Team4008HM2025();
    ElapsedTime Time = new ElapsedTime();
    double multy = 0.3;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx outtake = null;
    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        waitForStart();

        /*
        facing the right to start
        parking (parking autos)
         */
        moveForward(0.4,2200);
        /*
        Other autos we could do:
        */



    }
    public void moveForward (double power, int time){
        rightFrontDrive.setPower(-power);//forward is -direction?
        leftFrontDrive.setPower(-power);
        //robot.DriveRightBack.setPower(-power);
       // robot.DriveLeftBack.setPower(-power);
        sleep(time);
//    //    robot.DriveRightFront.setPower(0);
//        robot.DriveLeftFront.setPower(0);
//        robot.DriveRightBack.setPower(0);
//        robot.DriveLeftBack.setPower(0);
//    }

}}
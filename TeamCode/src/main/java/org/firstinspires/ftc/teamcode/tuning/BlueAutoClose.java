package org.firstinspires.ftc.teamcode.tuning;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name="BlueAutoClose", group="4008")
public class BlueAutoClose extends LinearOpMode{

    // Team4008HM2025 robot = new Team4008HM2025();
    ElapsedTime Time = new ElapsedTime();
    double multy = 0.3;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx outtake = null;
    private DcMotor intake = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Servo diverter = null;
    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        diverter = hardwareMap.get(Servo.class, "diverter");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);


        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        outtake.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);


        waitForStart();
        leftFrontDrive.setPower(-0.8);
        leftBackDrive.setPower(-0.8);
        rightFrontDrive.setPower(-0.8);
        rightBackDrive.setPower(-0.8);
        outtake.setPower(0.55);

        sleep(450); //400
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);


    /*    sleep(4000);
        rightFeeder.setPower(1);
        sleep(3000);
        rightFeeder.setPower(0);
        sleep(3000); */
        sleep(3000);

        leftFeeder.setPower(1);
        sleep(3000);
        leftFeeder.setPower(0);
        intake.setPower(0.5);
        outtake.setPower(0.5);

        sleep(4000);

        rightFeeder.setPower(1);
        sleep(3000);
        rightFeeder.setPower(0);


        sleep(3000);
        outtake.setPower(0.55);

        leftFrontDrive.setPower(0.8);
        leftBackDrive.setPower(0.8);
        rightFrontDrive.setPower(0.8);
        rightBackDrive.setPower(0.8);
        sleep(100); //400
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);


        rightFeeder.setPower(1);
        sleep(3000);
        rightFeeder.setPower(0);
        outtake.setPower(0);
        intake.setPower(0);
        sleep(3000);

        leftFrontDrive.setPower(0.8);
        leftBackDrive.setPower(0.8);
        rightFrontDrive.setPower(-0.8);
        rightBackDrive.setPower(-0.8);

        sleep(250);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        sleep(500);

        leftFrontDrive.setPower(-0.8);
        leftBackDrive.setPower(-0.8);
        rightFrontDrive.setPower(-0.8);
        rightBackDrive.setPower(-0.8);

        sleep(500);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);






    }}
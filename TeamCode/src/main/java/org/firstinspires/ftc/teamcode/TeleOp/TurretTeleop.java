
package org.firstinspires.ftc.teamcode.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTag;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;


@TeleOp(name = "Shooting Regression Testing", group = "StarterBot")
//@Disabled
public class TurretTeleop extends OpMode  {
    final double FEED_TIME_SECONDS = 0.80; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double LEFT_POSITION = 0.2962; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0;


    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx outtake = null;
//
    private DcMotor intake = null;
//    private CRServo leftFeeder = null;
//    private CRServo rightFeeder = null;
//    private Servo diverter = null;
//    private Servo Angle = null;
//    private CRServo Aim = null;


    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState leftLaunchState;
    private LaunchState rightLaunchState;

    private enum DiverterDirection {
        LEFT,
        RIGHT;
    }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private enum IntakeState {
        ON,
        OFF;
    }

    private IntakeState intakeState = IntakeState.OFF;

    private enum LauncherDistance {
        CLOSE,
        FAR;
    }


    ConceptAprilTag ConceptAprilTag =  new ConceptAprilTag();


    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        leftLaunchState = LaunchState.IDLE;
        rightLaunchState = LaunchState.IDLE;

        ConceptAprilTag.init();
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LF");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RF");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LB");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RB");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
       intake = hardwareMap.get(DcMotor.class, "intake");
//        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
//        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
//      //  diverter = hardwareMap.get(Servo.class, "diverter");
//        Angle = hardwareMap.get(Servo.class,"Angle");
//        Aim = hardwareMap.get(CRServo.class, "Aim");
        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        outtake.setDirection(DcMotorEx.Direction.FORWARD);
       intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        outtake.setZeroPowerBehavior(BRAKE);
       intake.setZeroPowerBehavior(BRAKE);


        /*
         * set Feeders to an initial value to initialize the servo controller
         */
//        leftFeeder.setPower(STOP_SPEED);
//        rightFeeder.setPower(STOP_SPEED);

        //outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.137743055555556, 0.1137743055555556, 0, 11.37743055555556))
        outtake.setVelocityPIDFCoefficients(1.137743055555556, 0.1137743055555556, 0, 11.37743055555556);//VelocityPIDF is temporary haven't te did it cuz it was on guide    sted yet

//        outtake.setPositionPIDFCoefficients(5);


        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
//        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad2.y) {
            outtake.setPower(-0.8);

        } else if (gamepad2.b) { // stop flywheel
            outtake.setPower(0);

        }

        if (gamepad2.x) {
            outtake.setPower(0.8);
        } else if (gamepad2.b) { // stop flywheel
            outtake.setPower(0);

        }



/*        if (gamepad2.dpadDownWasPressed()) {
            switch (diverterDirection){


                case LEFT:
                    diverterDirection = DiverterDirection.RIGHT;
                    diverter.setPosition(RIGHT_POSITION);
                    break;
                case RIGHT:
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    break;
            }
        }*/

        if (gamepad1.aWasPressed()){
            switch (intakeState){
                case ON:
                    intakeState = IntakeState.OFF;
                    intake.setPower(0);
                    break;
                case OFF:
                    intakeState = IntakeState.ON;
                    intake.setPower(1);
                    break;
            }
        }
        if (gamepad1.right_trigger > 0.2) {
            intake.setPower(1);

        } else if (gamepad1.left_trigger > 0.2) {
            intake.setPower(-1);
        } else{
            intake.setPower(0);
        }




        //   boolean reverse = false;
 /*       if (gamepad1.left_bumper) {
        leftFeeder.setPower(1);
       // reverse = true;
        } else {
            leftFeeder.setPower(0);
        }

    //    if (gamepad1.right_bumper) {
   //         rightFeeder.setPower(1);

    //    } else {
    //        rightFeeder.setPower(0);
     //   } */






//
//        if (gamepad2.left_trigger > 0.2) {
//            leftFeeder.setPower(1);
//            rightFeeder.setPower(1);
//
//        } else if (gamepad2.left_bumper) {
//            leftFeeder.setPower(-0.5);
//            leftFeeder.setPower(-0.5);
//        } else {
//            leftFeeder.setPower(0);
//            rightFeeder.setPower(0);
//        }
//
//        if (gamepad1.a) {
//            Aim.setPower(0.5);
//        } else if (gamepad1.b) {
//            Aim.setPower(-0.5);
//        }
//
//        if (gamepad1.x) {
//            Aim.setPower(0.5);
//        } else if (gamepad1.y) {
//            Aim.setPower(-0.5);
//        } else {
//            Aim.setPower(0);
//        }



        //   while (gamepad1.leftBumperWasPressed()) {
        //      leftFeeder.setPower(FULL_SPEED);
        //     }
        //        leftFeeder.setPower(STOP_SPEED);

        //    while (gamepad1.rightBumperWasPressed()) {
        //         rightFeeder.setPower(FULL_SPEED);
        //     }
        //     rightFeeder.setPower(STOP_SPEED);






//        if (gamepad1.leftBumperWasPressed()){
//            switch (intakeState){
//                case ON:
//
//                    leftFeeder.setPower(STOP_SPEED);
//
//                case OFF:
//
//                    leftFeeder.setPower(FULL_SPEED);
//
//            }
//        }
//
//        if (gamepad1.rightBumperWasPressed()){
//            switch (intakeState){
//                case ON:
//
//                    rightFeeder.setPower(STOP_SPEED);
//                    break;
//                case OFF:
//
//                    rightFeeder.setPower(FULL_SPEED);
//                    break;
//            }
//        }


//        if (gamepad1.dpadUpWasPressed()) {
//            switch (launcherDistance) {
//                case CLOSE:
//                    launcherDistance = LauncherDistance.FAR;
//                    launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
//                    launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
//                    break;
//                case FAR:
//                    launcherDistance = LauncherDistance.CLOSE;
//                    launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
//                    launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
//                    break;
//            }
//        }

        /*
         * Now we call our "Launch" function.
         */
//        launchLeft(gamepad1.leftBumperWasPressed());
//        launchRight(gamepad1.rightBumperWasPressed());

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", leftLaunchState);
        telemetry.addData("launch distance", launcherDistance);
        telemetry.addData("Left Launcher Velocity", outtake.getVelocity());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate){

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
//
//    void launchLeft(boolean shotRequested) {
//        switch (leftLaunchState) {
//            case IDLE:
//                if (shotRequested) {
//                    leftLaunchState = LaunchState.SPIN_UP;
//                }
//                break;
//            case SPIN_UP:
//                outtake.setVelocity(launcherTarget);
//
//                if (outtake.getVelocity() > launcherMin) {
//                    leftLaunchState = LaunchState.LAUNCH;
//                }
//                break;
//            case LAUNCH:
//                leftFeeder.setPower(FULL_SPEED);
//                leftFeederTimer.reset();
//                leftLaunchState = LaunchState.LAUNCHING;
//                break;
//            case LAUNCHING:
//                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
//                    leftLaunchState = LaunchState.IDLE;
//                    leftFeeder.setPower(STOP_SPEED);
//                }
//                break;
//        }
//    }
//
//    void launchRight(boolean shotRequested) {
//        switch (rightLaunchState) {
//            case IDLE:
//                if (shotRequested) {
//                    rightLaunchState = LaunchState.SPIN_UP;
//                }
//                break;
//            case SPIN_UP:
//                outtake.setVelocity(launcherTarget);
//
//                if (outtake.getVelocity() > launcherMin) {
//                    rightLaunchState = LaunchState.LAUNCH;
//                }
//                break;
//            case LAUNCH:
//                rightFeeder.setPower(FULL_SPEED);
//                rightFeederTimer.reset();
//                rightLaunchState = LaunchState.LAUNCHING;
//                break;
//            case LAUNCHING:
//                if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
//                    rightLaunchState = LaunchState.IDLE;
//                    rightFeeder.setPower(STOP_SPEED);
//                }
//                break;
}


package org.firstinspires.ftc.teamcode.TeleOp.Tests;//package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TeleOp.AprilTagsWebCam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.external.samples.externalhardware.AprilWebcam;


import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "TestPaths", group = "Autonomous")
public class AutoTest extends LinearOpMode{


    public class Intake {
        private DcMotor intake;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "Intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);

        }
        public class IntakeIn implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(0.8);
                    initialized = true;
                    return true;
                }
                else {
                     intake.setPower(0);
                        return false;
                }

            }
        }
        public Action intakeIn() {
            return new IntakeIn();
        }
    }
    public class Shooter {
        private DcMotor outtake;

        public Shooter(HardwareMap hardwareMap) {
            outtake = hardwareMap.get(DcMotor.class, "outtake");
            outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outtake.setDirection(DcMotor.Direction.FORWARD);
            outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
        public class ShooterOut implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtake.setPower(0.8);
                    initialized = true;
                    return true;
                }
                else {
                    outtake.setPower(0);
                    return false;
                }

            }
        }
        public Action ShootOut() {
            return new ShooterOut();
        }
    }
    public class Feeder {
        private CRServo rightFeeder;
        private CRServo leftFeeder;

        public Feeder(HardwareMap hardwareMap) {
            leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
            rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        }


        public class leftFeed implements Action {

            private boolean initialized = false;

            public boolean run(@NonNull TelemetryPacket packet) {

             if (!initialized) {
                 leftFeeder.setPower(1);
                 initialized = true;
                 return true;
             } else {
                 leftFeeder.setPower(0);
                 return false;
             }
            }

            public Action feedLeft() {
                return new leftFeed();
            }
        }

        public class rightFeed implements Action {
            private boolean initialized = false;

            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    rightFeeder.setPower(1);
                    initialized = true;
                    return true;
                } else {
                    rightFeeder.setPower(0);
                    return false;
                }
            }
            public Action feedRight() {
                return new rightFeed();
            }
        }







    }

    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap,initialPose);

       // Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
//        Feeder
//                feeder = new Feeder(hardwareMap);



        int visionOutputPosition = 1;



                TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                        .turn(45)
                        .lineToYConstantHeading(25)
                        .waitSeconds(1);


            /*    TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                        .turn(45)
                         .lineToYConstantHeading(25)
                        .waitSeconds(1);
                TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                        .turn(45)
                        .lineToYConstantHeading(25)
                        .waitSeconds(1);*/
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .lineToYConstantHeading(0)
                .build();



                  int startPosition = visionOutputPosition;
                 telemetry.addData("Starting Position", startPosition);
                 telemetry.update();
                 waitForStart();
                 if (isStopRequested()) return;

               Action trajectoryActionChosen;
                 if (startPosition == 1) {
                trajectoryActionChosen = tab1.build();
               } else if (startPosition == 2) {
                     trajectoryActionChosen = tab1.build();
                 } else {
                   trajectoryActionChosen = tab1.build();


                 }
                      Actions.runBlocking(
                              new SequentialAction(



                              trajectoryActionChosen,
                              trajectoryActionCloseOut,
                                      shooter.ShootOut()




                      )
                      );




        }



    }

//                        .splineToConstantHeading(new Vector2d(42, 40), Math.toRadians(180))
//                        .splineToConstantHeading(new Vector2d (10,55),Math.toRadians(270))
//                        .waitSeconds(0.1)//50-y
//                        .lineToX(60)
//                        .lineToX(40)


//                    .splineToConstantHeading(new Vector2d (52,50),Math.toRadians(270))//50




//                    .splineToConstantHeading(new Vector2d(36, -25), Math.toRadians(270))
//                    .lineToX(49)
//                    .lineToY(-59)



//}



//public class ServoAction implements Action {
//    Servo servo;
//    double position;
//
//    public ServoAction(Servo s, double p) {
//
//        this.servo = s;
//        this.position = p;
//    }
//
//    @Override
//    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//        servo.setPosition(position);
//        return (false);
//
//
//    }
//
//
//}}
//@Disabled
//public class April extends LinearOpMode {
//
//    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//
//    private AprilTagProcessor aprilTag;
//
//    private VisionPortal visionPortal;
//
//    @Override
//    public void runOpMode() {
//
//        initAprilTag();
//
//        // Wait for the DS start button to be touched.
//        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch START to start OpMode");
//        telemetry.update();
//        waitForStart();
//
//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//
//                telemetryAprilTag();
//
//                telemetry.update();
//
//                if (gamepad1.dpad_down) {
//                    visionPortal.stopStreaming();
//                } else if (gamepad1.dpad_up) {
//                    visionPortal.resumeStreaming();
//                }
//
//                // Share the CPU.
//                sleep(20);
//            }
//        }
//
//
//        visionPortal.close();
//
//    }
//    private void initAprilTag() {
//
//        aprilTag = new AprilTagProcessor.Builder()
//
//
//                .build();
//
//        aprilTag.setDecimation(3);
//
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//
//        builder.addProcessor(aprilTag);
//        visionPortal = builder.build();
//
//    }
//
//
//    /**
//     * Add telemetry about AprilTag detections.
//     */
//    private void telemetryAprilTag() {
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//
//        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");
//
//    }   // end method telemetryAprilTag()
//
//}
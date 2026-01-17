package org.firstinspires.ftc.teamcode.Auto;//package org.firstinspires.ftc.teamcode.tuning;


import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;




import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "BlueAuto", group = "Autonomous")
public class BlueAuto extends LinearOpMode {


    public class Gate {
        private Servo Gate;


        public Gate(HardwareMap hardwareMap) {
            Gate = hardwareMap.get(Servo.class, "gate");


        }

        public class CloseGate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Gate.setPosition(0.15);
                return false;
            }
        }


        public Action closeGate() {
            return new CloseGate();
        }


        public class OpenGate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Gate.setPosition(0);
                return false;
            }
        }


        public Action openGate() {
            return new OpenGate();
        }


    }

    public class Hood {
        private Servo Hood;


        public Hood(HardwareMap hardwareMap) {
            Hood = hardwareMap.get(Servo.class, "hood");


        }


        public class HoodActivation implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Hood.setPosition(0.6);
                return false;
            }
        }


        public Action HoodActivation() {
            return new HoodActivation();
        }
    }


    public class TurretTurn {
        private Servo TurretTurn;


        public TurretTurn(HardwareMap hardwareMap) {
            TurretTurn = hardwareMap.get(Servo.class, "turretservo");


        }


        public class TurretTurnShootingPOS implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                TurretTurn.setPosition(0.25);
                return false;
            }
        }


        public Action TurretTurnShootingPOS() {
            return new TurretTurnShootingPOS();
        }


    }

    public class Intake {
        private DcMotorEx intake;


        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorEx.Direction.REVERSE);


        }

        public class IntakeInLong implements Action {
            private boolean initialized = false;
            private long startTime;
            private final long runTimeMs = 3000; // 1 second (change this)


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(0.7);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }


                long elapsed = System.currentTimeMillis() - startTime;
                packet.put("intakeTimeMs", elapsed);


                if (elapsed < runTimeMs) {
                    return true; // keep running
                } else {
                    intake.setPower(0);
                    return false; // action finished
                }
            }
        }


        public Action intakeIn() {
            return new IntakeInLong();
        }
    }


    public class Shooter {
        private DcMotorEx outtake;
        private DcMotorEx outtake2;


        public Shooter(HardwareMap hardwareMap) {
            outtake = hardwareMap.get(DcMotorEx.class, "outtake");
            outtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            outtake.setDirection(DcMotorEx.Direction.FORWARD);
            outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


            outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
            outtake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            outtake2.setDirection(DcMotorEx.Direction.FORWARD);
            outtake2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            outtake.setVelocityPIDFCoefficients(1.489409090909091, 0.1489409090909091, 0, 14.89409090909091);


        }

        public class ShooterOut implements Action {
            private boolean initialized = false;
            private long startTime;
            private final long runTimeMs = 3000; // 1 second (change this)


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtake.setPower(-0.5);
                    outtake2.setPower(0.5);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }


                long elapsed = System.currentTimeMillis() - startTime;
                packet.put("intakeTimeMs", elapsed);


                if (elapsed < runTimeMs) {
                    return true; // keep running
                } else {
                    outtake.setPower(0);
                    outtake2.setPower(0);
                    return false; // action finished
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
        Pose2d initialPose = new Pose2d(-60, -38, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        TurretTurn turretTurn = new TurretTurn(hardwareMap);
        Gate gate = new Gate(hardwareMap);
        Hood hood = new Hood(hardwareMap);
//        Feeder
//                feeder = new Feeder(hardwareMap);


        int visionOutputPosition = 1;


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-10, -30));


        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()



                .strafeToConstantHeading(new Vector2d(-10, -60))
                .waitSeconds(0.5)


                .build();
        Action trajectoryActionCloseout2 = tab1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-7,-23))










//                .strafeToConstantHeading(new Vector2d(12,40))
//                .strafeToConstantHeading(new Vector2d(-12,20))
//                .turn(Math.toRadians(-45))

                .build();
        Action trajectoryActionCloseout3 = tab1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(15, -12))


                .build();
        Action DriveBack = tab1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-60, -10))
                .waitSeconds(2)

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
                new ParallelAction(
                       DriveBack,
                        shooter.ShootOut()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        hood.HoodActivation(),
                      shooter.ShootOut()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        shooter.ShootOut(),
                        intake.intakeIn()

                )
        );


        Actions.runBlocking(
                new ParallelAction(
                        trajectoryActionChosen,
                        turretTurn.TurretTurnShootingPOS(),
                        gate.closeGate()

                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        trajectoryActionCloseOut,
                        intake.intakeIn(),
                        shooter.ShootOut()


                ));
        Actions.runBlocking(
                new ParallelAction(
                        trajectoryActionCloseout2,
                        gate.openGate(),
                        shooter.ShootOut()

                )
        );//6 ball end


        Actions.runBlocking(
                new ParallelAction(
                        shooter.ShootOut(),
                        intake.intakeIn()


                ));
        Actions.runBlocking(
                new ParallelAction(
                        shooter.ShootOut(),
                        gate.openGate()


                )
        );
        Actions.runBlocking(
                new SequentialAction(


                        trajectoryActionCloseout2,
                        shooter.ShootOut()


                ));


        Actions.runBlocking(
                new ParallelAction(
                        shooter.ShootOut(),
                        intake.intakeIn()


                ));
//                new SequentialAction(
//                        trajectoryActionCloseout3
//
//
//                )
//
//
//
//        );
//        Actions.runBlocking(
//                new ParallelAction(
//                        intake.intakeIn(),
//                        trajectoryActionCloseout3
//                )
//        );
//        Actions.runBlocking(
//                new ParallelAction(
//                        intake.intakeIn(),
//                        shooter.ShootOut()
//
//                )
//        );


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

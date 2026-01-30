package org.firstinspires.ftc.teamcode.TeleOp.Tests;//package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Auto.RedAuto;
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
@Autonomous(name = "TestingRR", group = "Autonomous")
public class TestingRR extends LinearOpMode{


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
                    intake.setPower(0.8);
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
            return new TestingRR.Hood.HoodActivation();
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
            return new Shooter.ShooterOut();
        }
    }
    public class Gate {
        private Servo Gate;

        public Gate(HardwareMap hardwareMap) {
            Gate = hardwareMap.get(Servo.class, "gate");


        }
        public class CloseGate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Gate.setPosition(0.55);
                return false;
            }
        }

        public Action closeGate() {
            return new CloseGate();
        }


        public class OpenGate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Gate.setPosition(1);
                return false;
            }
        }

        public Action openGate() {
            return new OpenGate();
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
                TurretTurn.setPosition(1);
                return false;
            }
        }

        public Action TurretTurnShootingPOS() {
            return new TurretTurnShootingPOS();
        }


    }




    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-48, 48, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap,initialPose);

        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Gate gate = new Gate(hardwareMap);
        TurretTurn turretTurn = new TurretTurn(hardwareMap);
        Hood hood = new Hood(hardwareMap);



        int visionOutputPosition = 1;



        TrajectoryActionBuilder GoToFirstRow = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-10, 10));

        Action ShootRow1 = GoToFirstRow.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-10, 10))
//                .strafeToConstantHeading(new Vector2d(-12,20))
//                .turn(Math.toRadians(-45))
                .build();

        Action IntakeRow1 = drive.actionBuilder(new Pose2d(-10, 10, Math.toRadians(135)))
                .turnTo(Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-10, 48))
                .build();

        Action IntakeRowTwo = drive.actionBuilder(new Pose2d(-10, 10, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(12, 48), Math.PI / 2)
                .build();

        Action OpenGate1 = drive.actionBuilder(new Pose2d(12, 48, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(0, 56), -Math.PI / 2)
                .build();

        Action ComebackToShootRowTwo = drive.actionBuilder(new Pose2d(12, 48, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-10, 10), -Math.PI / 2)
                .build();

        Action IntakeFromGate = drive.actionBuilder(new Pose2d(-10, 10, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(9,55),Math.PI/2)

                .build();

        Action IntakeRowThree = drive.actionBuilder(new Pose2d(-10, 10, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(36, 48), Math.PI / 2)
                .build();

        Action ComebackToShootRowThree = drive.actionBuilder(new Pose2d(36, 48, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-10, 10), -Math.PI / 2)
                .build();






        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = GoToFirstRow.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = GoToFirstRow.build();
        } else {
            trajectoryActionChosen = GoToFirstRow.build();



        }
        Actions.runBlocking(
                new ParallelAction(
                        trajectoryActionChosen,
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
                        IntakeRow1,
                        intake.intakeIn(),
                        turretTurn.TurretTurnShootingPOS(),
                        gate.closeGate()
                )
        );
        Actions.runBlocking(
                new ParallelAction (
                        ShootRow1,
                        shooter.ShootOut(),
                        gate.openGate()

                )
        );

        Actions.runBlocking(
                new ParallelAction (
                        shooter.ShootOut(),
                        intake.intakeIn()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        gate.closeGate()
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        IntakeRowTwo,
                        intake.intakeIn()


                )
        );
        //Might have to change pls be cautious
        Actions.runBlocking(
                new SequentialAction(
                        OpenGate1,
                        new ParallelAction(
                                ComebackToShootRowTwo,
                                shooter.ShootOut(),
                                gate.openGate()

                        )
                )
        );
        /*Actions.runBlocking(
                new SequentialAction(
                        gate.openGate()

                ));*/

        Actions.runBlocking(
                new ParallelAction(
                        shooter.ShootOut(),
                        intake.intakeIn()

                ));

        //Stooooop

        Actions.runBlocking(
                new SequentialAction(
                        gate.closeGate(),
                        new ParallelAction(
                                IntakeFromGate,
                                intake.intakeIn()
                        )
                ));


       /* Actions.runBlocking(
                new SequentialAction(
                        gate.openGate(),
                        ShootRow1
                ));
        Actions.runBlocking(
                new SequentialAction(
                        shooter.ShootOut()


                ));

        Actions.runBlocking(
                new ParallelAction(
                        shooter.ShootOut(),
                        intake.intakeIn()


                ));
        Actions.runBlocking(
                new ParallelAction(
                        gate.closeGate(),
                        intake.intakeIn(),
                        IntakeRowTwo
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        ComebackToShootRowTwo
                ));

        Actions.runBlocking(
                new SequentialAction(
                        gate.openGate(),
                        shooter.ShootOut()


                ));
        Actions.runBlocking(
                new ParallelAction(
                        intake.intakeIn(),
                        shooter.ShootOut()

                )
        );*/






    }



}
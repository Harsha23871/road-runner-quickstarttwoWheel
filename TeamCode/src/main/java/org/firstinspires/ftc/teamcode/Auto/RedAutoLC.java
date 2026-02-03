package org.firstinspires.ftc.teamcode.Auto;//package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;





import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "RedAutoLC", group = "Autonomous")
public class RedAutoLC extends LinearOpMode{

    double Speed;
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
                    intake.setPower(1);
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
                Hood.setPosition(0.175);
                return false;
            }
        }
        public Action HoodActivation() {
            return new Hood.HoodActivation();
        }
    }


    public class Shooter {
        private DcMotorEx outtake;
        private DcMotorEx outtake2;


        public Shooter(HardwareMap hardwareMap) {
            outtake = hardwareMap.get(DcMotorEx.class, "outtake");
            outtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            outtake.setDirection(DcMotorEx.Direction.REVERSE);
            outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            outtake.setVelocityPIDFCoefficients(8, 0, 0.01, 8.8);


            outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
            outtake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            outtake2.setDirection(DcMotorEx.Direction.FORWARD);
            outtake2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            outtake2.setVelocityPIDFCoefficients(8, 0, 0.01, 8.8);


        }







        public class ShooterOut implements Action {
            private boolean initialized = false;
            private long startTime;
            private final long runTimeMs = 3000; // 1 second (change this)


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtake.setVelocity(2900*28/60);
                    outtake2.setVelocity(2900*28/60);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }


                long elapsed = System.currentTimeMillis() - startTime;
                packet.put("intakeTimeMs", elapsed);


                if (elapsed < runTimeMs) {
                    return true; // keep running
                } else {
                    outtake.setVelocity(0);
                    outtake2.setVelocity(0);
                    return false; // action finished
                }
            }
        }
        public Action ShootOut() {
            return new Shooter.ShooterOut();

        }
        public class ShooterRev implements Action {
            private boolean initialized = false;
            private long startTime;
            private final long runTimeMs = 3000; // 1 second (change this)


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtake.setVelocity(3500*28/60);
                    outtake2.setVelocity(3500*28/60);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }


                long elapsed = System.currentTimeMillis() - startTime;
                packet.put("intakeTimeMs", elapsed);


                if (elapsed < runTimeMs) {
                    return true; // keep running
                } else {
                    outtake.setVelocity(0);
                    outtake2.setVelocity(0);
                    return false; // action finished
                }
            }
        }
        public Action ShootRev() {
            return new Shooter.ShooterRev();

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
        Pose2d initialPose = new Pose2d(-65, 38, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap,initialPose);

        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Gate gate = new Gate(hardwareMap);
        TurretTurn turretTurn = new TurretTurn(hardwareMap);
        Hood hood = new Hood(hardwareMap);



        int visionOutputPosition = 1;



        TrajectoryActionBuilder GoToFirstRow = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-15, 25));


        Action IntakeRow1 = drive.actionBuilder(new Pose2d(-15, 25, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-15, 55))
                .build();

        Action ShootRow1 = drive.actionBuilder(new Pose2d(-15,55,Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-30,15))
//                .strafeToConstantHeading(new Vector2d(-12,20))
//                .turn(Math.toRadians(-45))
                .build();



        Action IntakeRowTwo = drive.actionBuilder(new Pose2d(-30, 15, Math.toRadians(90)))
                .setTangent(Math.toRadians(330))
                .splineToConstantHeading(new Vector2d(12, 55), Math.PI / 2)
                .build();

        Action OpenGate1 = drive.actionBuilder(new Pose2d(12, 48, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(0, 56), -Math.PI / 2)
                .build();

        Action ComebackToShootRowTwo = drive.actionBuilder(new Pose2d(12, 55, Math.toRadians(90)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-30, 15), -Math.PI / 2)
                .build();

        Action leave = drive.actionBuilder(new Pose2d(-30, 15, Math.toRadians(90)))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0, 40), -Math.PI / 2)
                .build();

        Action IntakeFromGate = drive.actionBuilder(new Pose2d(-10, 10, Math.toRadians(90)))
                .setTangent(20)
                .splineTo(new Vector2d(10, 58), Math.toRadians(100))

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
                        gate.closeGate(),
                        hood.HoodActivation()

                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        hood.HoodActivation(),
                        IntakeRow1,
                        intake.intakeIn(),
                        turretTurn.TurretTurnShootingPOS()
                )
        );

        Actions.runBlocking(
                new ParallelAction (
                        ShootRow1,
                        intake.intakeIn(),
                        shooter.ShootRev()


                ));

        Actions.runBlocking(
                new ParallelAction(
                        gate.openGate()
                ));


        Actions.runBlocking(
                new ParallelAction (
                        shooter.ShootRev(),
                        intake.intakeIn()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        gate.closeGate(),
                        IntakeRowTwo,
                        intake.intakeIn()
                )
        );
        Actions.runBlocking(
                new ParallelAction (
                        gate.closeGate(),
                        shooter.ShootRev(),
                        intake.intakeIn(),
                        ComebackToShootRowTwo
                )
        );
        Actions.runBlocking(
                new SequentialAction (
                        gate.openGate()
                )
        );
        Actions.runBlocking(
                new ParallelAction (
                        shooter.ShootRev(),
                        intake.intakeIn()
                )
        );
        Actions.runBlocking(
                new ParallelAction (
                        leave
                )
        );




//        Actions.runBlocking(
//                new SequentialAction(
//                        gate.closeGate()
//                )
//        );
//
//        Actions.runBlocking(
//                new ParallelAction(
//                        IntakeRowTwo,
//                        intake.intakeIn()
//
//
//                )
//        );
//        //Might have to change pls be cautious
//        Actions.runBlocking(
//                        new ParallelAction(
//                                ComebackToShootRowTwo,
//                                shooter.ShootOut(),
//                                gate.openGate()
//                )
//        );
//        /*Actions.runBlocking(
//                new SequentialAction(
//                        gate.openGate()
//
//                ));*/
//
//        Actions.runBlocking(
//                new ParallelAction(
//                        shooter.ShootOut(),
//                        intake.intakeIn()
//
//                ));
//
//        //Stooooop
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        gate.closeGate(),
//                        new ParallelAction(
//                                IntakeFromGate,
//                                intake.intakeIn()
//                        )
//                ));


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
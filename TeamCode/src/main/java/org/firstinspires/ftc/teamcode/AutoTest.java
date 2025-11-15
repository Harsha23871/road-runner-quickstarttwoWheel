package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "TestPaths", group = "Autonomous")
public class AutoTest extends LinearOpMode {
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
        private DcMotor shooter;

        public Shooter(HardwareMap hardwareMap) {
            shooter = hardwareMap.get(DcMotor.class, "Shooter");
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        }
        public class ShooterOut implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    shooter.setPower(0.8);
                    initialized = true;
                    return true;
                }
                else {
                   shooter.setPower(0);
                    return false;
                }

            }
        }
        public Action ShootOut() {
            return new ShooterOut();
        }
    }


    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-144, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap,initialPose);

        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        waitForStart();


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-144,0, 90))
                        .turn(45)
                        .lineToYConstantHeading(25)
                        .waitSeconds(1)
                        .build());

        Actions.runBlocking(
                new SequentialAction(
                        intake.intakeIn(),
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
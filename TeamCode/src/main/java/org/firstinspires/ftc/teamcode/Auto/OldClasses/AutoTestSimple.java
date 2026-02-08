//package org.firstinspires.ftc.teamcode.TeleOp.Tests;//package org.firstinspires.ftc.teamcode.tuning;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.TeleOp.AprilTagsWebCam;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//import androidx.annotation.NonNull;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.robotcontroller.external.samples.externalhardware.AprilWebcam;
//
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Config
//@Autonomous(name = "AutoTestSimple", group = "Autonomous")
//public class AutoTestSimple {
//
//
//
//
//    public class Shooter {
//        private DcMotor shooter;
//
//        public Shooter(HardwareMap hardwareMap) {
//            shooter = hardwareMap.get(DcMotor.class, "Shooter");
//            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            shooter.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        }
//        public class ShooterOut implements Action{
//            private boolean initialized = false;
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    shooter.setPower(0.8);
//                    initialized = true;
//                    return true;
//                }
//                else {
//                    shooter.setPower(0);
//                    return false;
//                }
//
//            }
//        }
////        public Action Shoot() {
////            return new AutoTestSimple().Shooter.ShooterOut();
////        }
////    }
//
//
//
//
//
//
//
//
//}}
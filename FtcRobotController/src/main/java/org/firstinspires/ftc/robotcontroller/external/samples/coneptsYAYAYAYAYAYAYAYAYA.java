package org.firstinspires.ftc.robotcontroller.external.samples;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Concept: AprilTag Shooter", group = "Concept")
public class coneptsYAYAYAYAYAYAYAYAYA extends LinearOpMode {

    DcMotorEx motor;
    CRServo servo;

    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ================== SHOOTER CONSTANTS ==================
    final double g = 9.81;                // gravity in m/s^2
    final double shooterAngleDeg = 35;    // your hood angle
    final double shooterAngle = Math.toRadians(shooterAngleDeg);
    final double targetHeight = 1.0;      // target height relative to shooter in meters
    final double wheelRadius = 0.03;      // shooter wheel radius in meters
    final double TPR = 560;               // Yellow Jacket 1:1 ticks per revolution
    // ========================================================

    @Override
    public void runOpMode() {

        initAprilTag();

        telemetry.addData(">", "Touch START to begin");
        telemetry.update();

        motor = hardwareMap.get(DcMotorEx.class, "CoreHex");

        // PIDF values
        motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        1.137743055555556,
                        0.1137743055555556,
                        0,
                        11.37743055555556
                )
        );

        servo = hardwareMap.get(CRServo.class, "servo");

        waitForStart();

        while (opModeIsActive()) {

            // ================== TELEMETRY & SHOOTER ==================
            List<AprilTagDetection> tags = aprilTag.getDetections();

            telemetry.addData("# Tags", tags.size());

            for (AprilTagDetection detection : tags) {

                if (detection.metadata != null) {
                    telemetry.addLine(String.format("ID: %d (%s)",
                            detection.id, detection.metadata.name));

                    double distanceInches = detection.ftcPose.range;
                    double bearing = detection.ftcPose.bearing;

                    telemetry.addData("Range (inch)", distanceInches);
                    telemetry.addData("Bearing (deg)", bearing);

                    // ---- SHOOTER VELOCITY CALCULATION ----
                    if (detection.id == 20) {  // only for your target

                        double x = distanceInches * 0.0254;  // inches → meters

                        // geometry safety check
                        if (x <= 0.01) {
                            motor.setVelocity(0);
                            continue;
                        }

                        double denom = 2 * Math.pow(Math.cos(shooterAngle), 2) * (x * Math.tan(shooterAngle) - targetHeight);

                        if (denom <= 0) {
                            motor.setVelocity(0);
                            continue;
                        }

                        double v0 = Math.sqrt(g * x * x / denom);  // m/s

                        double radPerSec = v0 / wheelRadius;
                        double ticksPerSec = radPerSec * (TPR / (2 * Math.PI));

                        motor.setVelocity(ticksPerSec);  // PIDF handles speed

                         telemetry.addLine("--- Shooter ---");
                        telemetry.addData("v0 (m/s)", v0);
                        telemetry.addData("Motor ticks/sec", ticksPerSec);

                        // simple servo aiming
                        if (bearing > 2) servo.setPower(-0.5);
                        else if (bearing < -2) servo.setPower(0.5);
                        else servo.setPower(0);
                    }
                }
            }

            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }

    // ============================================================
    //                     APRILTAG INIT
    // ============================================================

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM)
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        else
            builder.setCamera(BuiltinCameraDirection.BACK);

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }
}

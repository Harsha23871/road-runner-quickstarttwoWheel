package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretTrackingTest {

    /* ==================== TUNING ==================== */
    private static final double SERVO_CENTER = 0.5;

    // Turret mechanical limits (radians)
    private static final double MAX_LEFT_RAD  = Math.PI / 2;
    private static final double MAX_RIGHT_RAD = -Math.PI / 2;

    /* ==================== HARDWARE ==================== */
    private final Servo turretServo;

    /* ==================== TARGET ==================== */
    private double targetX;
    private double targetY;

    public TurretTrackingTest(Servo turretServo) {
        this.turretServo = turretServo;
        turretServo.setPosition(SERVO_CENTER);
    }

    /* Set a fixed field target */
    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
    }

    /* Call every loop */
    public void update(Pose2d robotPose) {
        double dx = targetX - robotPose.position.x;
        double dy = targetY - robotPose.position.y;

        // Angle from robot to target (field frame)
        double fieldAngle = Math.atan2(dy, dx);

        // Turret-relative angle
        double robotHeading = robotPose.heading.toDouble();
        double turretAngle = fieldAngle - robotHeading;

        // Normalize to [-pi, pi]
        turretAngle = normalizeAngle(turretAngle);

        // Clamp to turret range
        turretAngle = clamp(turretAngle, MAX_RIGHT_RAD, MAX_LEFT_RAD);

        // Convert radians → servo position
        double servoPos = map(
                turretAngle,
                MAX_RIGHT_RAD, MAX_LEFT_RAD,
                0.0, 1.0
        );

        turretServo.setPosition(servoPos);
    }

    /* ==================== UTILS ==================== */

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private double map(double x,
                       double inMin, double inMax,
                       double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin)
                / (inMax - inMin) + outMin;
    }
}

package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem {

    private final DcMotor shooterMotor;   // flywheel
    private final DcMotor yawMotor;       // rotates shooter left/right
    private final Servo pitchServo;       // adjusts shooter pitch

    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double targetYawAngle = 0;  // target yaw in degrees
    private double lastError = 0;
    private double integral = 0;

    private final double pitchStraightUp = 0.0;
    private final double pitchMaxAngle = 1.0;

    // Max RPM of shooter flywheel (adjust for your motor)
    private static final double MAX_RPM = 6000;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        yawMotor = hardwareMap.get(DcMotor.class, "yawMotor");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");

        // Configure motors
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        yawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitchServo.setPosition(0.5); // neutral position
    }

    /* ---------- Shooter wheel control ---------- */
    public void setShooterPower(double power) {
        shooterMotor.setPower(power);
    }

    public void stopShooter() {
        shooterMotor.setPower(0);
    }

    public double getShooterPower() {
        return shooterMotor.getPower();
    }

    /* ---------- Pitch control ---------- */
    public void setPitchPosition(double position) {
        pitchServo.setPosition(Math.min(Math.max(position, pitchStraightUp), pitchMaxAngle));
    }

    public double getPitchPosition() {
        return pitchServo.getPosition();
    }

    /* ---------- Yaw control (PID) ---------- */
    public void setYawAngle(double angle) {
        targetYawAngle = angle;
    }

    public void updateYaw() {
        double currentYaw = yawMotor.getCurrentPosition(); // encoder ticks
        // Convert ticks to degrees (assuming 1 tick = 1 deg for example, adjust!)
        double currentAngle = currentYaw;

        double error = targetYawAngle - currentAngle;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double output = kP * error + kI * integral + kD * derivative;
        output = Math.max(Math.min(output, 1.0), -1.0); // clamp -1 to 1

        yawMotor.setPower(output);
    }

    public boolean isYawAtTarget(double toleranceDegrees) {
        double currentAngle = yawMotor.getCurrentPosition();
        return Math.abs(currentAngle - targetYawAngle) <= toleranceDegrees;
    }
}

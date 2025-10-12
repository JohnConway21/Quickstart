package org.firstinspires.ftc.teamcode.pedroPathing.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.ShooterSubsystem;

public class ShooterCommand {

    private final ShooterSubsystem shooter;

    // Constants (all in inches)
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;
    private static final double GOAL_Z = 36.0;

    private static final double SHOOTER_X = 0.0; // assume shooter at field origin x
    private static final double SHOOTER_Y = 0.0; // assume shooter y origin
    private static final double SHOOTER_Z = 12.0;

    private static final double VERTEX_Z = 60.0; // desired parabola peak

    private double targetPitch;   // radians
    private double targetYaw;     // radians
    private double shooterPower;  // 0..1

    private final ElapsedTime timer = new ElapsedTime();
    private final double shootDuration = 2.0; // seconds

    public ShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    public void initialize() {
        // Step 1: calculate horizontal yaw angle
        double deltaX = GOAL_X - SHOOTER_X;
        double deltaY = GOAL_Y - SHOOTER_Y;
        targetYaw = Math.atan2(deltaY, deltaX); // radians

        // Step 2: horizontal distance
        double horizontalDistance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);

        // Step 3: parabola math
        // Use x = horizontal distance, z = vertical height
        double x1 = 0; // shooter at origin
        double z0 = SHOOTER_Z;
        double zGoal = GOAL_Z;

        // Solve for other x-intercept
        double x2 = computeOtherXIntercept(x1, z0, horizontalDistance, zGoal, VERTEX_Z);

        // Vertex x-coordinate
        double xV = (x1 + x2)/2.0;

        // Parabola coefficient
        double a = (z0 - VERTEX_Z)/Math.pow(x1 - xV, 2);

        // Step 4: pitch angle from tangent at shooter
        double slope = 2 * a * (x1 - xV);
        targetPitch = Math.atan(slope);

        // Step 5: convert pitch to servo position
        shooter.setPitchPosition(targetPitchToServoPosition(targetPitch));

        // Step 6: set yaw target
        shooter.setYawAngle(Math.toDegrees(targetYaw));

        // Step 7: calculate flywheel power
        double launchSpeed = Math.sqrt(9.81 * horizontalDistance * horizontalDistance
                / (2 * Math.pow(Math.cos(targetPitch), 2) * (horizontalDistance * Math.tan(targetPitch) - (zGoal - z0))));
        double wheelRadius = 2.0; // inches (example)
        shooterPower = launchSpeedToMotorPower(launchSpeed, wheelRadius);
        shooter.setShooterPower(shooterPower);

        timer.reset();
    }

    public void execute() {
        shooter.updateYaw(); // PID loop
    }

    public boolean isFinished() {
        return timer.seconds() >= shootDuration && shooter.isYawAtTarget(2.0);
    }

    public void end() {
        shooter.stopShooter();
    }

    /* ---------- Helper Methods ---------- */

    private double computeOtherXIntercept(double x1, double z0, double xGoal, double zGoal, double vertexZ) {
        double xV = (x1 + xGoal)/2.0; // temporary vertex approximation
        double a = (z0 - vertexZ)/Math.pow(x1 - xV, 2);
        return xGoal - zGoal / (a*(xGoal - x1));
    }

    private double launchSpeedToMotorPower(double launchSpeed, double wheelRadius) {
        // Convert inches/sec to RPM and then to 0..1 power
        double wheelRPM = (launchSpeed / wheelRadius) * 60.0 / (2*Math.PI);
        double power = wheelRPM / 4000.0; // MAX_RPM
        return Math.min(Math.max(power, 0), 1.0);
    }

    private double targetPitchToServoPosition(double pitchRad) {
        double minAngle = 0.0;        // radians
        double maxAngle = Math.PI/2;   // radians
        return Math.min(Math.max((pitchRad - minAngle)/(maxAngle - minAngle), 0), 1);
    }
}

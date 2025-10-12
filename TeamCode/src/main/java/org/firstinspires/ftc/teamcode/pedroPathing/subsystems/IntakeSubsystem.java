package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {

    private final DcMotor intakeMotor;
    private final Telemetry telemetry;

    // Define constants for power levels
    private static final double INTAKE_POWER = 1.0;   // full power for intake
    private static final double OUTTAKE_POWER = -1.0; // reverse direction

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motor
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor"); // must match config name
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Pulls the game element (ball) inward */
    public void intake() {
        intakeMotor.setPower(INTAKE_POWER);
        telemetry.addData("Intake", "Running (in)");
    }

    /** Pushes the game element outward (reverse) */
    public void outtake() {
        intakeMotor.setPower(OUTTAKE_POWER);
        telemetry.addData("Intake", "Running (out)");
    }

    /** Stops the motor */
    public void stop() {
        intakeMotor.setPower(0);
        telemetry.addData("Intake", "Stopped");
    }

    /** Call periodically for telemetry updates */
    public void update() {
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.update();
    }
}

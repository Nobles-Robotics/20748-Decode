package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class Robot extends SubsystemGroup {
    public static final Robot INSTANCE = new Robot();

    // Intake timing constants (in seconds)
    private static final double INTAKE_STARTUP_TIME = 0.3;
    private static final double BALL_INTAKE_TIME = 0.8;
    private static final double STORAGE_ROTATE_TIME = 0.4;
    private static final double INTAKE_POWER = 1.0;

    public static Robot.Alliance getCurrentAlliance(){
        return currentAlliance;
    }

    public static Robot.Alliance currentAlliance = Robot.Alliance.BLUE;

    public enum Alliance {
        RED,
        BLUE
    }

    private Robot() {
        super(
                Intake.INSTANCE,
                Transitions.INSTANCE,
                Storage.INSTANCE,
                Outtake.INSTANCE
        );
    }

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
    }

    public static SequentialGroup outtakeAll = new SequentialGroup(
            new IfElseCommand(
                    Outtake::reachedTargetVelocity,
                    Outtake.on
    ));

    /**
     * Command to intake a single ball into storage.
     * Turns on intake and transition, waits for ball, then rotates storage.
     */
    public static Command intakeSingleBall() {
        return new SequentialGroup(
                // Wait for ball to enter storage
                new Delay(BALL_INTAKE_TIME),
                // Rotate storage to next position
                Storage.spinToNextIntakeIndex(),
                // Allow time for storage to rotate
                new Delay(STORAGE_ROTATE_TIME)
        );
    }

    /**
     * Command to start the intake mechanism (motor + transition servo).
     * Should be called before intake operations begin.
     */
    public static Command startIntake() {
        return new SequentialGroup(
                new ParallelGroup(
                        Intake.setIntakePowerCommand(INTAKE_POWER),
                        Transitions.on()
                ),
                new Delay(INTAKE_STARTUP_TIME)
        );
    }

    /**
     * Command to stop the intake mechanism.
     * Should be called after intake operations complete.
     */
    public static Command stopIntake() {
        return new ParallelGroup(
                Intake.setIntakePowerCommand(0),
                Transitions.off()
        );
    }

    /**
     * Command to intake all 3 balls during autonomous.
     * Starts intake, collects 3 balls sequentially, then stops intake.
     */
    public static SequentialGroup intakeAll = new SequentialGroup(
            // Start the intake mechanism
            new ParallelGroup(
                    Intake.setIntakePowerCommand(INTAKE_POWER),
                    Transitions.on()
            ),
            new Delay(INTAKE_STARTUP_TIME),
            // Intake first ball
            new Delay(BALL_INTAKE_TIME),
            Storage.spinToNextIntakeIndex(),
            new Delay(STORAGE_ROTATE_TIME),
            // Intake second ball
            new Delay(BALL_INTAKE_TIME),
            Storage.spinToNextIntakeIndex(),
            new Delay(STORAGE_ROTATE_TIME),
            // Intake third ball
            new Delay(BALL_INTAKE_TIME),
            Storage.spinToNextIntakeIndex(),
            new Delay(STORAGE_ROTATE_TIME),
            // Stop the intake mechanism
            new ParallelGroup(
                    Intake.setIntakePowerCommand(0),
                    Transitions.off()
            )
    );

}

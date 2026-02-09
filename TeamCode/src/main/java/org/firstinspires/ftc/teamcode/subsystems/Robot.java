package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.utils.SequentialGroupFixed;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;
public class Robot extends SubsystemGroup {
    public static final Robot INSTANCE = new Robot();
    private static final double OUTTAKE_DELAY = .15;
    private static final double INTAKE_DELAY = 0.5;

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

    public static SequentialGroupFixed outtakeAll = new SequentialGroupFixed(
            new InstantCommand(Outtake.on),
            new WaitUntil(Outtake::reachedTargetVelocity),
            new InstantCommand(Transitions.on()),
            new Delay(OUTTAKE_DELAY),
            new InstantCommand(Storage.setManualModeCommand(true)),
            new InstantCommand(Storage.setManualPowerCommand(0.35)),
            new Delay(OUTTAKE_DELAY*15),
            new InstantCommand(Transitions.off()),
            new InstantCommand(Outtake.off),
            new InstantCommand(Storage.setManualPowerCommand(0))
    );

    public static SequentialGroupFixed intakeAll = new SequentialGroupFixed(
            new InstantCommand(Intake.on()),
            new InstantCommand(Transitions.on()),
            new InstantCommand(Storage.requestAlign(1)),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.requestAlign(1)),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.requestAlign(1)),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Transitions.off()),
            new InstantCommand(Intake.off())
    );
}

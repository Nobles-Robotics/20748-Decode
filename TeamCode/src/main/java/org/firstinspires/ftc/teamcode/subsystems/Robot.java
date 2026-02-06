package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;
public class Robot extends SubsystemGroup {
    public static final Robot INSTANCE = new Robot();
    private static final double DELAY = 0.5;

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
            new InstantCommand(Outtake.on),
            new InstantCommand(Storage.spinToNextOuttakeIndex()),
            new WaitUntil(Outtake::reachedTargetVelocity),
            new InstantCommand(Transitions.on()),
            new Delay(DELAY),
            new WaitUntil(Outtake::reachedTargetVelocity),
            new InstantCommand(Storage.spinToNextOuttakeIndex()),
            new Delay(DELAY),
            new WaitUntil(Outtake::reachedTargetVelocity),
            new InstantCommand(Storage.spinToNextOuttakeIndex()),
            new Delay(DELAY),
            new InstantCommand(Transitions.off()),
            new InstantCommand(Outtake.off)
    );

    public static SequentialGroup outtakeAllStupidly = new SequentialGroup(
            new InstantCommand(Outtake.on),
            new WaitUntil(Outtake::reachedTargetVelocity),
            new InstantCommand(Transitions.on()),
            new InstantCommand(Storage.setManualModeCommand(true)),
            new InstantCommand(Storage.setManualPowerCommand(.4)),
            new InstantCommand(Transitions.off()),
            new InstantCommand(Outtake.off)
    );

    public static SequentialGroup outtakeOne = new SequentialGroup(
            Outtake.on,
            Storage.spinToNextOuttakeIndex(),
            new WaitUntil(Outtake::reachedTargetVelocity),
            Transitions.on(),
            new Delay(DELAY),
            Transitions.off(),
            Outtake.off
    );

//    public static SequentialGroup intakeAll = new SequentialGroup(
//
//    );
}

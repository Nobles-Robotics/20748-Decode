package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SequentialGroupFixed;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;
import kotlin.time.Instant;

public class Robot extends SubsystemGroup {
    public static final Robot INSTANCE = new Robot();
    private static final double INTAKE_DELAY = 0.1;
    private static final double OUTTAKE_DELAY = 0.25;
    public static final double CACHING_TOLERANCE = 0.03;

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

    public static SequentialGroupFixed outtakeAllStupidly = new SequentialGroupFixed(
            new InstantCommand(Outtake.on),
            new WaitUntil(Outtake::reachedTargetVelocity),
            new InstantCommand(Transitions.on()),
            new Delay(.15),
            new InstantCommand(Storage.setManualModeCommand(true)),
            new InstantCommand(Storage.setManualPowerCommand(0.35)),
            new Delay(2.5),
            new InstantCommand(Transitions.off()),
            new InstantCommand(Outtake.off),
            new InstantCommand(Storage.setManualPowerCommand(0))
    );
    public static SequentialGroupFixed intakeOne = new SequentialGroupFixed(
            new InstantCommand(Intake.on()),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Intake.off())
            );


    // Note: works
    public static SequentialGroupFixed intakeAll = new SequentialGroupFixed(
            new InstantCommand(Intake.on()),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new Delay(0.25),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new InstantCommand(Storage.checkIfStuck(INTAKE_DELAY, 4)),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new InstantCommand(Storage.checkIfStuck(INTAKE_DELAY, 4)),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new InstantCommand(Storage.checkIfStuck(INTAKE_DELAY, 4)),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new InstantCommand(Storage.checkIfStuck(INTAKE_DELAY, 4)),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new InstantCommand(Storage.checkIfStuck(INTAKE_DELAY, 4)),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.checkIfStuck(INTAKE_DELAY, 4)),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.checkIfStuck(INTAKE_DELAY, 4)),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.checkIfStuck(INTAKE_DELAY, 4)),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Intake.off())
    );
    public static SequentialGroupFixed intake3 = new SequentialGroupFixed(
            new InstantCommand(Intake.on()),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new Delay(INTAKE_DELAY),
            new InstantCommand(Intake.off())
    );

    public static SequentialGroupFixed outtakeAll = new SequentialGroupFixed(
            new InstantCommand(Outtake.on),
            new InstantCommand(() -> Outtake.setTargetVelocity(1850)),
            new Delay(OUTTAKE_DELAY),
            new InstantCommand(Storage.spinToNextOuttakeIndex()),
            new InstantCommand(Transitions.on()),
            new Delay(OUTTAKE_DELAY),
            new InstantCommand(Storage.spinToNextOuttakeIndex()),
            new Delay(OUTTAKE_DELAY),
            new InstantCommand(Storage.spinToNextOuttakeIndex()),
            new Delay(OUTTAKE_DELAY),
            new InstantCommand(Storage.spinToNextOuttakeIndex()),
            new Delay(OUTTAKE_DELAY),
            new InstantCommand(Transitions.off()),
            new InstantCommand(Outtake.off)
    );

    public static SequentialGroupFixed outtakeOn = new SequentialGroupFixed(
            new InstantCommand(Outtake.on),
            new ParallelDeadlineGroup(
                    new Delay(1),
                    new WaitUntil(Outtake::reachedTargetVelocity)
            ),
            new InstantCommand(Storage.spinToNextOuttakeIndex()),
            new InstantCommand(Transitions.on()),
            new Delay(OUTTAKE_DELAY),
            new InstantCommand(Transitions.off()),
            new InstantCommand(Outtake.off)
    );

    // !EXPERIMENTAL! disables color sensor, intense logging, and limit switch
    public static Command optimizeLoopTimes(boolean enable) {
        return new InstantCommand(() -> {
            if (enable) {
                Storage.setRequestReadColorSensor(false);
                Storage.setRequestReadLimitSwitch(false);
                Limelight.setRequestReadLimelight(false);
                Logger.disableInfoLogging();
            } else {
                Storage.setRequestReadColorSensor(true);
                Storage.setRequestReadLimitSwitch(true);
                Limelight.setRequestReadLimelight(true);
                Logger.enableInfoLogging();
            }
        });
    }
}

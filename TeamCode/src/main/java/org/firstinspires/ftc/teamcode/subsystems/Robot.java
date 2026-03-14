package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Storage.extraTimeToOuttakeAfterStuck;

import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SequentialGroupFixed;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class Robot extends SubsystemGroup {
    public static final Robot INSTANCE = new Robot();
    private static final double INTAKE_DELAY = 0.1;
    private static final double OUTTAKE_DELAY = 0.30;
    private static final double SMALL_DELAY = 0.15;
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
            Outtake.on,
            new WaitUntil(Outtake::reachedTargetVelocity),
            Transitions.on(),
            new Delay(.15),
            Storage.setManualModeCommand(true),
            Storage.setManualPowerCommand(0.35),
            new Delay(2.5),
            Transitions.off(),
            Outtake.off,
            Storage.setManualPowerCommand(0)
    );
    public static SequentialGroupFixed intakeOne = new SequentialGroupFixed(
            Intake.on(),
            Storage.spinToNextIntakeIndex(),
            new Delay(INTAKE_DELAY),
            Storage.spinToNextIntakeIndex(),
            new Delay(INTAKE_DELAY),
            Storage.spinToNextIntakeIndex(),
            new Delay(INTAKE_DELAY),
            Intake.off()
            );


    // Note: works
    public static SequentialGroupFixed intakeAll = new SequentialGroupFixed(
            Intake.on(),
            Storage.spinToNextIntakeIndex(),
            new Delay(0.1),
            Storage.spinToNextIntakeIndex(),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.assertManualPower(0)),
            new Delay(INTAKE_DELAY),
            Storage.spinToNextIntakeIndex(),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.assertManualPower(0)),
            new Delay(INTAKE_DELAY),
            Storage.spinToNextIntakeIndex(),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.assertManualPower(0)),
            new Delay(INTAKE_DELAY),
            Storage.spinToNextIntakeIndex(),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.assertManualPower(0)),
            new Delay(INTAKE_DELAY),
            Storage.spinToNextIntakeIndex(),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.assertManualPower(0)),
            new Delay(INTAKE_DELAY),
            Storage.spinToNextIntakeIndex(),
            new Delay(INTAKE_DELAY),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.assertManualPower(0)),
            new Delay(INTAKE_DELAY),
            Storage.spinToNextIntakeIndex(),
            new Delay(INTAKE_DELAY),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.assertManualPower(0)),
            new Delay(INTAKE_DELAY),
            Storage.spinToNextIntakeIndex(),
            new Delay(INTAKE_DELAY),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.assertManualPower(0)),
            new Delay(INTAKE_DELAY),
            Storage.spinToNextIntakeIndex(),
            new Delay(INTAKE_DELAY),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
//            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
//            new Delay(INTAKE_DELAY),
//            Storage.spinToNextIntakeIndex(),
//            new Delay(INTAKE_DELAY),
//            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new IfElseCommand(() -> !Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
            new Delay(INTAKE_DELAY),
            Storage.spinToNextIntakeIndex(),
            new Delay(INTAKE_DELAY),
            Intake.off()
    );

    public static SequentialGroupFixed intakeAllSmooth = new SequentialGroupFixed(
            Intake.on(),
            Storage.assertManualPower(0.6),
            new Delay(INTAKE_DELAY),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
            new Delay(INTAKE_DELAY),
            Storage.assertManualPower(0.6),
            new Delay(INTAKE_DELAY),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
            new Delay(INTAKE_DELAY),
            Storage.assertManualPower(0.6),
            new Delay(INTAKE_DELAY),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
            new Delay(INTAKE_DELAY),
            Storage.assertManualPower(0.6),
            new Delay(INTAKE_DELAY),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> Storage.isStorageMotorStuck(), Storage.spinToLastIntakeIndex()),
            new Delay(0.2),
            Storage.assertManualPower(0),
            Intake.off()
    );
    public static SequentialGroupFixed intake3 = new SequentialGroupFixed(
            Intake.on(),
            Storage.spinToNextIntakeIndex(),
            new Delay(INTAKE_DELAY),
            Storage.spinToNextIntakeIndex(),
            new Delay(INTAKE_DELAY),
            Storage.spinToNextIntakeIndex(),
            new Delay(INTAKE_DELAY),
            Intake.off()
    );

    public static SequentialGroupFixed outtakeAll = new SequentialGroupFixed(
            Outtake.on,
            new InstantCommand(() -> Outtake.setTargetVelocity(1850)),
            new ParallelRaceGroup(
                    new WaitUntil(Outtake::reachedTargetVelocity),
                    new Delay(1)
            ),
            Storage.spinToNextOuttakeIndex(),
            Transitions.on(),
            new Delay(OUTTAKE_DELAY),
            Storage.spinToNextOuttakeIndex(),
            new Delay(OUTTAKE_DELAY),
            Storage.spinToNextOuttakeIndex(),
            new Delay(OUTTAKE_DELAY),
            Storage.spinToNextOuttakeIndex(),
            new Delay(OUTTAKE_DELAY),
            Transitions.off(),
            Outtake.off
    );

    public static SequentialGroupFixed outtakeAllAuto = new SequentialGroupFixed(
            Transitions.off(),
            Outtake.on,
            new InstantCommand(() -> Outtake.setTargetVelocity(1850)),
            new ParallelRaceGroup(
                    new WaitUntil(Outtake::reachedTargetVelocity),
                    new Delay(1)
            ),
            Transitions.on(),
            new Delay(1),

            Storage.spinToNextOuttakeIndex(),
            new Delay(OUTTAKE_DELAY),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> Storage.isStorageMotorStuck(), Intake.off(), Intake.on()),
            Storage.spinToNextOuttakeIndex(),
            new Delay(OUTTAKE_DELAY),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> Storage.isStorageMotorStuck(), Intake.off(), Intake.on()),
            Storage.spinToNextOuttakeIndex(),
            new Delay(OUTTAKE_DELAY),
            Storage.checkIfStuck(INTAKE_DELAY, 4),
            new Delay(INTAKE_DELAY),
            new IfElseCommand(() -> Storage.isStorageMotorStuck(), Intake.off(), Intake.on()),
            Storage.spinToNextOuttakeIndex(),
            new Delay(OUTTAKE_DELAY),
            Transitions.off(),
            Outtake.off
    );

    public static double targetVelocityClose = 1715;
    public static double targetVelocityFar = 1925;

    public static InstantCommand setTargetVelocityAuto(boolean close){
        double targetVelocity;
        if (close){
            targetVelocity = targetVelocityClose;
        }
        else{
            targetVelocity = targetVelocityFar;
        }
        return new InstantCommand(() -> Outtake.setTargetVelocity(targetVelocity)); //1975 far, 1750 close
    }

    public static SequentialGroupFixed outtakeAllSmooth (boolean close){
        double targetVelocity;
        if (close){
            targetVelocity = targetVelocityClose;
        }
        else{
            targetVelocity = targetVelocityFar;
        }
        return new SequentialGroupFixed(
                Transitions.off(),
                Outtake.on,
                new InstantCommand(() -> Outtake.setTargetVelocity(targetVelocity)),
                new ParallelRaceGroup(
                        new WaitUntil(Outtake::reachedTargetVelocity),
                        new Delay(1)
                ),
                Intake.off(),
                Transitions.on(),

                Storage.assertManualPower(0.25),

                new Delay (INTAKE_DELAY),
                Storage.checkIfStuck(INTAKE_DELAY, 4),
                new Delay((INTAKE_DELAY)+0.02),
                new IfElseCommand(() -> Storage.isStorageMotorStuck(),
                        new ParallelGroup(
                                new SequentialGroupFixed(
                                        Transitions.off(),
                                        new Delay (INTAKE_DELAY),
                                        Storage.assertManualPower(1),
                                        Storage.outtakeStuckSignal()
                                ),
                                new Delay(0.6)
                        ),
                        new SequentialGroupFixed(
                                new Delay (0.5),
                                Storage.checkIfStuck(INTAKE_DELAY, 4),
                                new Delay((INTAKE_DELAY)+0.02),
                                new IfElseCommand(() -> Storage.isStorageMotorStuck(),
                                        new ParallelGroup(
                                                new SequentialGroupFixed(
                                                        Transitions.off(),
                                                        new Delay (INTAKE_DELAY),
                                                        Storage.assertManualPower(1),
                                                        Storage.outtakeStuckSignal()
                                                ),
                                                new Delay(0.5)
                                        )
                                )
                        )
                ),
                Storage.assertManualPower(0.25),
                Transitions.on(),
                new Delay(1),

//            new InstantCommand(Storage.spinToNextOuttakeIndex()),
//            new Delay(OUTTAKE_DELAY),
//            new InstantCommand(Storage.spinToNextOuttakeIndex()),
//            new Delay(OUTTAKE_DELAY),
//            new InstantCommand(Storage.spinToNextOuttakeIndex()),
//            new Delay(OUTTAKE_DELAY),
//            new InstantCommand(Storage.spinToNextOuttakeIndex()),
//            new Delay(OUTTAKE_DELAY),
                Storage.assertManualPower(0),
                Transitions.off(),
                Outtake.off,
                Storage.resetOuttakeStuckSignal()
        );
    }


    public static SequentialGroupFixed outtakeAllFar = new SequentialGroupFixed(
            Outtake.on,
            new InstantCommand(() -> Outtake.setTargetVelocity(2125)),
            new ParallelRaceGroup(
                    new WaitUntil(Outtake::reachedTargetVelocity),
                    new Delay(1)
            ),
            new Delay(SMALL_DELAY),
            new ParallelRaceGroup(
                    new WaitUntil(Outtake::reachedTargetVelocity),
                    new Delay(1)
            ),
            Storage.spinToNextOuttakeIndex(),
            Transitions.on(),
            new Delay(SMALL_DELAY),
            new ParallelRaceGroup(
                    new WaitUntil(Outtake::reachedTargetVelocity),
                    new Delay(1)
            ),
            Storage.spinToNextOuttakeIndex(),
            Storage.spinToNextOuttakeIndex(),
            new Delay(SMALL_DELAY),
            new ParallelRaceGroup(
                    new WaitUntil(Outtake::reachedTargetVelocity),
                    new Delay(1)
            ),
            Storage.spinToNextOuttakeIndex(),
            new Delay(SMALL_DELAY),
            new ParallelRaceGroup(
                    new WaitUntil(Outtake::reachedTargetVelocity),
                    new Delay(1)
            ),
            Transitions.off(),
            Outtake.off
    );

    public static SequentialGroupFixed outtakeOn = new SequentialGroupFixed(
            Outtake.on,
            new ParallelDeadlineGroup(
                    new Delay(1),
                    new WaitUntil(Outtake::reachedTargetVelocity)
            ),
            Storage.spinToNextOuttakeIndex(),
            Transitions.on(),
            new Delay(OUTTAKE_DELAY),
            Transitions.off(),
            Outtake.off
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

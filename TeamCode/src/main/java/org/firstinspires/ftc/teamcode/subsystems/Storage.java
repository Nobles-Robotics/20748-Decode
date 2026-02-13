package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.CACHING_TOLERANCE;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Storage implements Subsystem {
    public static final Storage INSTANCE = new Storage();
    private static boolean manualMode = true;
    private static boolean positionMode = false;
    private static double manualPower = 0;
    private final static MotorEx spin = new MotorEx("motorExp3", CACHING_TOLERANCE).brakeMode().reversed();
    private static DigitalChannel limitSwitch;
    private static RevColorSensorV3 colorSensor;
    private static double currentPosition;
    private static double targetPosition;
    private static final double DELTA_TICKS = 179.25;
    private static final double OUTTAKE_POSITION = DELTA_TICKS + DELTA_TICKS / 2;
    private static boolean lastState = false;
    public static boolean alignRequested = false;
    private static double newPower;
    private static boolean requestReadLimitSwitch = true;
    private static boolean requestReadColorSensor = true;
    private static boolean storageMotorStuck = false;

    public static void setRequestReadLimitSwitch(boolean toRequest){
        requestReadLimitSwitch = toRequest;
    }

    public static void setRequestReadColorSensor(boolean toRequest){
        requestReadColorSensor = toRequest;
    }

    public static ControlSystem controller = ControlSystem.builder()
            .posPid(0.015, 0, .04)
            .build();

    public static final State[] STATES = {
            State.NONE,
            State.NONE,
            State.NONE
    };

    public enum State {
        PURPLE,
        GREEN,
        NONE,
        BALL,
    }

    @Override
    public void initialize() {
        spin.setCurrentPosition(0);
        spin.zero();
        currentPosition = spin.getCurrentPosition();
        targetPosition = currentPosition;

        limitSwitch = ActiveOpMode.hardwareMap().get(DigitalChannel.class,
                "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = ActiveOpMode.hardwareMap().get(RevColorSensorV3.class,
                "colorSensor");
    }

    @Override
    public void periodic() {
        currentPosition = spin.getCurrentPosition();

        if (manualMode) {
            spin.setPower(manualPower);
            newPower = manualPower;
        } else if (positionMode) {
            controller.setGoal(new KineticState(targetPosition));
            newPower = controller.calculate(new KineticState(currentPosition));
            if (Math.abs(newPower) > 0.01) {
                spin.setPower(newPower);
            } else {
                spin.setPower(0);
            }
        } else {
            spin.setPower(0);
        }
        Logger.add("Storage", "target position: " + targetPosition + "current position:" + currentPosition + "current power:" + newPower);
        Logger.add("Storage", "color:" + getColor());

        boolean currentSwitchState = limitSwitch.getState();

        if (currentSwitchState && !lastState && alignRequested) {
            stop().schedule();
            alignRequested = false;
        }

        lastState = currentSwitchState;
    }

    public static Command spinToNextIntakeIndex() {
        return new LambdaCommand()
                .setStart(() -> {
                    manualMode = false;
                    positionMode = true;
                    double startPos = currentPosition + 10;

                    double remainder = startPos % DELTA_TICKS;
                    if (remainder < 0) remainder += DELTA_TICKS;

                    double ticksToMove = DELTA_TICKS - remainder;

                    targetPosition = startPos + ticksToMove;
                })
                .setIsDone(() -> true)
                .setStop(interrupted -> {
                })
                .requires(Storage.INSTANCE)
                .setInterruptible(true)
                .named("Spin to next index");
    }

    public static Command spinToNextOuttakeIndex() {
        return new LambdaCommand()
                .setStart(() -> {
                    manualMode = false;
                    positionMode = true;
                    double startPos = currentPosition - 10;

                    double remainder = startPos % DELTA_TICKS;
                    if (remainder < 0) remainder += DELTA_TICKS;

                    double ticksToMove = (DELTA_TICKS / 2.0) - remainder;
                    if (ticksToMove >= 0) ticksToMove -= DELTA_TICKS;

                    targetPosition = startPos + ticksToMove;
                })
                .setIsDone(() -> true)
                .setStop(interrupted -> {
                })
                .requires(Storage.INSTANCE)
                .setInterruptible(true)
                .named("Spin to next index");
    }

    public static Command requestAlign(double newPower) {
        return new InstantCommand(() -> {
            setManualMode(true);
            setManualPower(newPower);
            alignRequested = true;
        });
    }

    public static Command stop() {
        return new InstantCommand(() -> {
            manualMode = true;
            manualPower = 0;
        });
    }


    public static Command assertManualPower(double newPower) {
        return new InstantCommand(() -> {
            setManualMode(true);
            setManualPower(newPower);
        });
    }
    public static Command setManualPowerCommand(double newPower) {
        return new InstantCommand(() -> setManualPower(newPower));
    }

    public static Command setManualModeCommand(boolean newMode) {
        return new InstantCommand(() -> setManualMode(newMode));
    }

    public static Command setPositionModeCommand(boolean newMode) {
        return new InstantCommand(() -> setPositionMode(newMode));
    }

    public static Command resetEncoderCommand() {
        return new InstantCommand(() -> resetEncoder(0));
    }

    public static Command resetEncoderAtOuttakeCommand() {
        return new InstantCommand(() -> resetEncoder(OUTTAKE_POSITION));
    }

    public static Command checkIfStuck(double checkDurationSeconds, double minPositionDeltaTicks) {
        final double[] startPosition = {0};
        final long[] startTimeNanos = {0};

        return new LambdaCommand()
                .setStart(() -> {
                    startPosition[0] = currentPosition;
                    startTimeNanos[0] = System.nanoTime();
                    storageMotorStuck = false;
                })
                .setIsDone(() -> {
                    double elapsedSeconds = (System.nanoTime() - startTimeNanos[0]) / 1_000_000_000.0;

                    if (elapsedSeconds >= checkDurationSeconds) {
                        double movedTicks = Math.abs(currentPosition - startPosition[0]);
                        storageMotorStuck = movedTicks < minPositionDeltaTicks;
                        return true;
                    }

                    return false;
                })
                .setStop(interrupted -> {
                })
                .requires(Storage.INSTANCE)
                .setInterruptible(true)
                .named("Check storage stuck");
    }

    public static boolean isStorageMotorStuck() {
        return storageMotorStuck;
    }

    private static void setManualPower(double newPower) {
        manualPower = newPower;
    }

    private static void setPositionMode(boolean newBoolean) {
        positionMode = newBoolean;
    }

    private static void setManualMode(boolean newMode) {
        manualMode = newMode;
    }

    private static void resetEncoder(double newPosition) {
        spin.setCurrentPosition(newPosition);
        targetPosition = newPosition;
    }

    public State getColor() {
        NormalizedRGBA c = colorSensor.getNormalizedColors();
        double d = colorSensor.getDistance(DistanceUnit.MM);

        if (d > 30.0) {
            return State.NONE;
        }

        float divisor = Math.max(c.alpha, 1.0f);
        float r = c.red / divisor;
        float g = c.green / divisor;
        float b = c.blue / divisor;

        Logger.add("Storage", "r: " + r + "g:" + g + "b:" + b);

        if ((g / r) > 2.0 && g > b) {
            return State.GREEN;
        }
        else if ((b / g) > 1.3 && b > r) {
            return State.PURPLE;
        }

        return State.NONE;
    }

    public static boolean wasJustPressed() {
        boolean currentState = limitSwitch.getState();
        boolean justPressed = currentState && !lastState;
        lastState = currentState;
        return justPressed;
    }
}

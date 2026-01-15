package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Storage implements Subsystem {
    public static final Storage INSTANCE = new Storage();
    private static boolean manualMode = true;
    private static boolean runpowerMode = false;
    private static double manualPower = 0;
    private static double runPower = 0;
    private final static MotorEx spin = new MotorEx("motorExp0").brakeMode();



    private static DigitalChannel limitSwitch;
    private static NormalizedColorSensor colorSensor;
    private static double startPos = 0;
    private static final double TICKS = 185;

    private static boolean lastState = false;
    private static double index = 0;

    public static boolean getManualMode(){
        return manualMode;
    }
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
         spin.zero();
         limitSwitch = ActiveOpMode.hardwareMap().get(DigitalChannel.class,
         "limitSwitch");
         limitSwitch.setMode(DigitalChannel.Mode.INPUT);

         colorSensor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class,
         "colorSensor");

    }

    @Override
    public void periodic() {
        if (wasJustPressed()){
            index++;
            if (index >= STATES.length) {
                index = 0;
            }
        }
        if (manualMode) {
            spin.setPower(manualPower);
        } else if (runpowerMode){
            spin.setPower(runPower);
        }

        if (!limitSwitch.getState()){
            Robot.outtake1.schedule();
        }

        // Write Telemetry
        Logger.add("Storage", Logger.Level.DEBUG, "ticks: " + spin.getCurrentPosition());
        Logger.add("Storage", Logger.Level.DEBUG, "runPower?" + runpowerMode +  "runpower: " + runPower);
        Logger.add("Storage", Logger.Level.DEBUG, "manual?" + manualMode +  "power: " + manualPower);
        Logger.add("Storage", Logger.Level.DEBUG, "limit switch" + limitSwitch.getState());
        Logger.add("Storage", Logger.Level.DEBUG, "index: " + index);
        Logger.add("Storage", Logger.Level.DEBUG, "Color: " + getColor().red + ", " + getColor().green + ", " + getColor().blue);
    }

    public static Command setManualPowerCommand(double newPower) {
        return new InstantCommand(() -> setManualPower(newPower));
    }
    public static Command setManualModeCommand(boolean newMode) {
        return new InstantCommand(() -> setManualMode(newMode));
    }
    public static Command setRunPowerMode(boolean newMode) {
        return new InstantCommand(() -> setRunpowerMode(newMode));
    }

    public static Command resetEncoderCommand() {
        return new InstantCommand(Storage::resetEncoder);
    }

    public static Command spinToNextIntakeIndex() {
        return new LambdaCommand()
                .setStart(() -> {
                })
                .setIsDone(() -> true)
                .setStop(interrupted -> {})
                .requires(Storage.INSTANCE)
                .setInterruptible(true)
                .named("Spin to next index");
    }

    public static Command spinToNextOuttakeIndex() {
        return new LambdaCommand()
                .setStart(() -> {
                    manualMode = false;
                    runpowerMode = true;
                    runPower = 0.5;
                })
                .setIsDone(() -> !limitSwitch.getState())
                //.setIsDone(() -> false)
                .setStop(interrupted -> {
                    runPower = -.15;
                    manualPower = 0;
                })
                .requires(Storage.INSTANCE)
                .setInterruptible(true)
                .named("Spin to next index");
    }
    private static void setManualPower(double newPower) {
        manualPower = newPower;
    }
    private static void setRunpowerMode(boolean newBoolean) {
        runpowerMode = newBoolean;
    }
    private static void setManualMode(boolean newMode) {
        manualMode = newMode;
    }
    private static void resetEncoder() {
        spin.zero();
    }

    private static void resetEncoderAtOuttake() {
        spin.setCurrentPosition(270);
    }

    public static Command resetEncoderAtOuttakeCommand() {
        return new InstantCommand(Storage::resetEncoderAtOuttake);
    }

    public static NormalizedRGBA getColor() {
        return colorSensor.getNormalizedColors();
    }

    public static State readColor() {
        NormalizedRGBA colors = getColor();

        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;

        double sum = red + green + blue;

        double r = red / sum;
        double g = green / sum;
        double b = blue / sum;

        if (r > 0.35 && b > 0.35 && g < 0.25) {
            return State.PURPLE;
        }
        if (g > 0.45 && r < 0.3 && b < 0.3) {
            return State.GREEN;
        }
        return State.NONE;
    }

    public static boolean wasJustPressed() {
        boolean currentState = limitSwitch.getState();
        boolean justPressed = currentState && !lastState;
        lastState = currentState;
        return justPressed;
    }

//
//    public static Command prioritySpin(int targetIndex) {
//        return new LambdaCommand()
//                .setStart(() -> {
//                    manualMode = false;
//                    runPower = .6;
//                })
//                .setIsDone(() -> index == targetIndex)
//                .setStop(interrupted -> {
//                    runPower = 0;
//                    manualPower = 0;
//                    manualMode = true;
//                })
//                .requires(Storage.INSTANCE)
//                .setInterruptible(false)
//                .named("Priority Spin → " + targetIndex);
//    }
//
//    public static Command lazySpin(int targetIndex) {
//        return new LambdaCommand()
//                .setStart(() -> {
//                    manualMode = false;
//                    runPower = .6;
//                })
//                .setIsDone(() -> index == targetIndex)
//                .setStop(interrupted -> {
//                    if (!interrupted) {
//                        runPower = 0;
//                        manualPower = 0;
//                        manualMode = true;
//                    }
//                })
//                .requires(Storage.INSTANCE)
//                .setInterruptible(true)
//                .named("Lazy Spin → " + targetIndex);
//    }
//
//    public static Command spinToState(State targetState) {
//        int targetIndex = -1;
//        for (int i = 0; i < STATES.length; i++) {
//            if (STATES[i] == targetState) {
//                targetIndex = i;
//                break;
//            }
//        }
//        if (targetIndex == -1) {
//            return new NullCommand();
//        }
//        return prioritySpin(targetIndex);
//    }
//
//    public static Command reIndex() {
//
//        Command[] steps = new Command[STATES.length * 2];
//        int cmdIndex = 0;
//
//        for (int i = 0; i < STATES.length; i++) {
//            final int target = i;
//
//            steps[cmdIndex++] = prioritySpin(target);
//
//            steps[cmdIndex++] = new LambdaCommand()
//                    .setStart(() -> STATES[target] = readColor())
//                    .setIsDone(() -> true)
//                    .named("read color at " + target);
//        }
//
//        return new SequentialGroup(steps)
//                .named("reIndex");
//    }
//
//    public static Command clearCurrentIndex() {
//        return new LambdaCommand()
//                .setStart(() -> STATES[index] = State.NONE)
//                .setIsDone(() -> true)
//                .named("clear current index");
//    }
//
//    public static Command clearIndexState(int targetIndex) {
//        return new LambdaCommand()
//                .setStart(() -> STATES[targetIndex] = State.NONE)
//                .setIsDone(() -> true)
//                .named("clear state of " + targetIndex);
//    }
//
//    public static Command resetIndexStates() {
//
//        Command[] clears = new Command[STATES.length];
//
//        for (int i = 0; i < STATES.length; i++) {
//            clears[i] = clearIndexState(i);
//        }
//        return new SequentialGroup(clears)
//                .named("reset index states");
//    }


//    public static Command spinForwardTicks(int ticks) {
//        return new LambdaCommand()
//                .setStart(() -> {
//                    manualMode = false;
//                    pidControlMode = true;
//                    startPos = spin.getCurrentPosition();
//                    controller.setGoal(new KineticState(startPos + ticks));
//                })
//                .setUpdate(() -> {})
//                .setIsDone(() -> manualMode)
//                .setStop(interrupted -> {})
//                .requires(Storage.INSTANCE)
//                .setInterruptible(false)
//                .named("Spin Forward " + ticks + " Ticks");
//    }
}

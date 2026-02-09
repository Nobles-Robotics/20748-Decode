package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Storage implements Subsystem {
    public static final Storage INSTANCE = new Storage();

    // Hardware
    private final static MotorEx spin = new MotorEx("motorExp3").brakeMode();
    private static DigitalChannel limitSwitch;

    // Movement state
    private static boolean manualMode = true;
    private static double manualPower = 0;

    // Math / Averaging Variables
    private static double lastPressPosition = 0;
    private static double runningSum = 0;
    private static int validCount = 0;
    private static int dummyCount = 0;
    private static double delta;
    private static double mod = 0;
    private static double test1 = 0;
    private static double test2 = 0;
    private static double test3 = 0;

    // Switch state tracking
    private static boolean lastSwitchState = false;

    @Override
    public void initialize() {
        spin.setPower(0);
        // Initialize lastPress to current so the first delta isn't huge
        lastPressPosition = spin.getCurrentPosition();

        limitSwitch = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "limitSwitch");
    }

    @Override
    public void periodic() {
        // 1. Handle Motor Power
        if (manualMode) {
            spin.setPower(manualPower);
        }

        // 2. Handle Limit Switch Logic
        // Note: Check your specific wiring. Usually !getState() is pressed for Touch Sensors.
        // Adopting your previous logic style, assuming getState() returns true when pressed.
        boolean currentSwitchState = limitSwitch.getState();

        // Detect Rising Edge (Just Pressed)
        if (currentSwitchState && !lastSwitchState) {
            handleLimitSwitchPress();
            stopCommand().schedule();
        }

        lastSwitchState = currentSwitchState;
        double average = runningSum / validCount;
        double avg1 = test1 / mod * 3;
        double avg2 = test2 / mod * 3;
        double avg3 = test3 / mod * 3;

        ActiveOpMode.telemetry().addLine(limitSwitch.getState() + " limit switch");
        ActiveOpMode.telemetry().addLine(spin.getCurrentPosition() + " ");
        ActiveOpMode.telemetry().addLine("delta: " + delta + " avg: " + average);
        ActiveOpMode.telemetry().addData("Avg Ticks", average);
        ActiveOpMode.telemetry().addData("Valid Counts", validCount);
        ActiveOpMode.telemetry().addData("Bad Counts", dummyCount);
        ActiveOpMode.telemetry().addLine(avg1 + ", " + avg2 + ", " + avg3);
    }

    private void handleLimitSwitchPress() {
        double currentPosition = spin.getCurrentPosition();

        // Calculate absolute difference since last press
        delta = Math.abs(currentPosition - lastPressPosition);

        // Update the last position marker immediately
        lastPressPosition = currentPosition;
        // 3. Filter and Log
        if (delta >= 130 && delta <= 230) {
            runningSum += delta;
            validCount++;

            if (mod % 3 == 0) test1 += delta;
            if (mod % 3 == 1) test2 += delta;
            if (mod % 3 == 2) test3 += delta;
            mod++;
        } else {
            dummyCount++;
        }
    }

    public static Command setManualPowerCommand(double newPower) {
        return new InstantCommand(() -> {
            manualMode = true;
            manualPower = newPower;
        });
    }

    public static Command stopCommand() {
        return new InstantCommand(() -> {
            manualMode = true;
            manualPower = 0;
        });
    }

    public static Command resetAveragingCommand() {
        return new InstantCommand(() -> {
            runningSum = 0;
            validCount = 0;
            lastPressPosition = spin.getCurrentPosition();
        });
    }
}

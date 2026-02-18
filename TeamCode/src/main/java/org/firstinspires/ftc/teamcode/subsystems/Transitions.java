package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.CACHING_TOLERANCE;

import org.firstinspires.ftc.teamcode.utils.Logger;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Transitions implements Subsystem {

    public static final Transitions INSTANCE = new Transitions();
    private final static double FORWARD_POWER = 1;
    private final static double REVERSE_POWER = -1;
    public static double currentPower = 0;
    private final MotorEx transition = new MotorEx("motorExp0", CACHING_TOLERANCE);

    @Override
    public void initialize() {
        transition.setPower(0);
    }

    @Override
    public void periodic() {
        transition.setPower(currentPower);
        //Logger.add("Transition", "power: " + currentPower);
    }

    public static Command on() {
        return new InstantCommand(setTransitionPowerCommand(FORWARD_POWER));
    }
    public static Command reverse() {
        return new InstantCommand(setTransitionPowerCommand(REVERSE_POWER));
    }
    public static Command off() {
        return new InstantCommand(setTransitionPowerCommand(0));
    }
    public static Command setTransitionPowerCommand(double newPower) {
        return new InstantCommand(() -> setTransitionPower(newPower));
    }
    private static void setTransitionPower(double newPower) {
        currentPower = newPower;
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Drive.follower;
import static org.firstinspires.ftc.teamcode.utils.components.AllianceManager.currentAlliance;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.control.KineticState;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.Logger;

public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();
    private static final MotorEx outtake = new MotorEx("motorExp2").reversed().floatMode();
    private static Servo hoodServo;
    private static Servo traverseServo;
    private static final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private static double targetVelocity = 2700;
    private static double currentVelocity = 0;
    private static double manualPower = 0;
    private static boolean velocityMode = false;
    private static boolean manualMode = false;

    public static Pose shootTarget = new Pose(6, 144 - 6, 0);

    /*
    TODO: retune these values
     ks -> minimum for movement; kV -> 1/max velo;
     kP until reaches other velocites
     */
    private static final ControlSystem controller = ControlSystem.builder()
            .velPid(0.0045, 0, 0)
            .basicFF(0.0004, 0, 0.0345)
            .build();

    public static Command off = new InstantCommand(() -> {
        velocityMode = false;
        manualMode = false;
    });
    public static Command on = new InstantCommand(() -> {
        manualMode = false;
        velocityMode = true;
    });

    public static Command setOuttakeManualPowerCommand(double newPower) {
        return new InstantCommand(() -> setManualPower(newPower));
    }

    public static Command setOuttakeManualModeCommand(boolean newMode) {
        return new InstantCommand(() -> setManualMode(newMode));
    }

    @Override
    public void initialize() {
        manualMode = false;
        velocityMode = false;
        manualPower = 0;
        setShootTarget();
    }

    @Override
    public void periodic() {
        currentVelocity = outtake.getVelocity();

        if (manualMode) {
            outtake.setPower(manualPower);
        } else if (velocityMode){
            controller.setGoal(new KineticState(0, targetVelocity));
            double velocityPower = controller.calculate(new KineticState(0, currentVelocity));
            Logger.add("Outtake", "current power:" + velocityPower);

            outtake.setPower(velocityPower);
        } else {
            outtake.setPower(0);
        }
        Logger.add("Outtake", "current velo:" + currentVelocity);

        Logger.add("Outtake", "current distance:" + getDistance());
        Logger.panelsLog("velo", currentVelocity);
        Logger.panelsLog("power", outtake.getPower());
    }

    public static boolean reachedTargetVelocity(){
        return outtake.getVelocity() > (targetVelocity - 50);
    }

    public static void setTargetVelocity(double newTargetVelocity){
        Outtake.targetVelocity = newTargetVelocity;
    }

    public static void setManualMode(boolean newMode){
        manualMode = newMode;
    }

    public static void setManualPower(double newPower){
        manualPower = newPower;
    }

    public void setShootTarget() {
        if (currentAlliance == Alliance.BLUE&& shootTarget.getX() != 6)
            shootTarget = new Pose(6, 144 - 6, 0);
        else if (currentAlliance == Alliance.RED && shootTarget.getX() != (144 - 6))
            shootTarget = shootTarget.mirror();
    }

    public Pose getShootTarget() {
        return shootTarget;
    }

    public double getDistance() {
        double deltaX = - shootTarget.getX() + follower.getPose().getX();
        double deltaY = - shootTarget.getX() + follower.getPose().getY();
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }
}

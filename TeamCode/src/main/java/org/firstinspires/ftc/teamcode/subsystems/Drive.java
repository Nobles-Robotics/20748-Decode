package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.components.AllianceManager.currentAlliance;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.pedropathing.math.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();
    public static Follower follower;
    public static TelemetryManager telemetryM;
    private static boolean slowMode = false;
    private static final double slowModeMultiplier = 0.25;
    private static final boolean robotCentric = true;
    private static double targetHeading = Math.toRadians(180); // Radians
    public static Pose autoendPose;
    private static PIDFController controller;
    private static boolean headingLock = false;

    // 69 ->

    private static double aimAssistment = 0;
    @Override
    public void initialize() {
        follower = Constants.createFollower(ActiveOpMode.hardwareMap());
        follower.setStartingPose(new Pose(135, 9, Math.toRadians(0)));
        if (!(autoendPose == null)) {
            follower.setPose(autoendPose);
        }
        follower.update();
        controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //cornerReset();
    }

    public static Command cornerResetCommand() {
        return new InstantCommand(Drive::cornerReset);
    }
    public static void cornerReset() {
        if (currentAlliance == Alliance.BLUE) follower.setPose(new Pose(135, 9, Math.toRadians(0)));
        else follower.setPose(new Pose(9, 9, Math.toRadians(180)));
    }

    @Override
    public void periodic() {
        controller.setCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0.04));
        controller.updateError(getHeadingError());

        targetHeading = MathFunctions.normalizeAngle(Outtake.getAngle() + Math.PI);
        drive.schedule();
    }

    public static Command setSlowModeCommand(boolean newMode) {
        return new InstantCommand(() -> setSlowMode(newMode));
    }

    public static Command setHeadingLockCommand(boolean newMode) {
        return new InstantCommand(() -> setHeadingLock(newMode));
    }
    public static void setAimAssist(double num){
        aimAssistment = num;
    }
    public static Command drive = new LambdaCommand()
            .setStart(() -> follower.startTeleopDrive())
            .setUpdate(() -> {
                follower.update();
              //  telemetryM.update();

                double forward = slowMode ? -ActiveOpMode.gamepad1().left_stick_y * slowModeMultiplier: -ActiveOpMode.gamepad1().left_stick_y;
                forward = -forward;
                double strafe = slowMode ? -ActiveOpMode.gamepad1().left_stick_x * slowModeMultiplier: -ActiveOpMode.gamepad1().left_stick_x;
                strafe = -strafe;
                double turn = slowMode ? -(ActiveOpMode.gamepad1().right_stick_x + aimAssistment) * slowModeMultiplier: -(ActiveOpMode.gamepad1().right_stick_x + aimAssistment);

                if (headingLock)
                    follower.setTeleOpDrive(forward, strafe, controller.run(), robotCentric);
                else
                    follower.setTeleOpDrive(forward, strafe, turn, robotCentric);

//                Logger.add("Drive", "forward: " + forward + " strafe: " + strafe + " turn: " + turn);
//                Logger.add("Drive", "posx: " + Math.round(follower.getPose().getX()*100/100) + " posy: " + Math.round(follower.getPose().getY()*100)/100 + " heading: " + Math.round(follower.getPose().getHeading()*100)/100);
//                Logger.add("Drive", "heading lock?"+ headingLock + "target heading: " + Math.round(targetHeading*100)/100 + "error: " + Math.round(getHeadingError()*100)/100 + "target distance:" + Math.round(Outtake.getDistance()*100)/100);
//                Logger.add("Drive", "slowmode? " + slowMode + "multiplier: " + slowModeMultiplier);
            })
            .setStop(interrupted -> {})
            .setIsDone(() -> false)
            .requires(Drive.INSTANCE)
            .setInterruptible(false)
            .named("Drive");
    public static double getHeadingError() {
//        if (follower.getCurrentPath() == null) {
//            return 0;
//        }
        return MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);
    }

    private static void setSlowMode(boolean newMode) {
        slowMode = newMode;
    }

    private static void setHeadingLock(boolean newLock) {
        headingLock = newLock;
    }

    private static void setTargetHeading(double newHeading) {
        targetHeading = newHeading;
    }
}

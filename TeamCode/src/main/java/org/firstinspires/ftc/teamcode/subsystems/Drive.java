package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.math.MathFunctions.normalizeAngle;

import static org.firstinspires.ftc.teamcode.utils.components.AllianceManager.currentAlliance;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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


import org.firstinspires.ftc.teamcode.LimelightTest;

public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();
    public static Follower follower;
    public static TelemetryManager telemetryM;
    private static boolean slowMode = false;
    private static final double slowModeMultiplier = 0.25;
    private static final boolean robotCentric = true;
    private static double targetHeading = Math.toRadians(180); // Radians
    private static PIDFController controller;
    private static boolean headingLock = false;

    private static double aimAssistment = 0;
    @Override
    public void initialize() {
        follower = Constants.createFollower(ActiveOpMode.hardwareMap());
        follower.setStartingPose(new Pose(8, 6.25, Math.toRadians(0)).mirror());
        follower.update();
        controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        cornerReset();
    }

    public static Command cornerResetCommand() {
        return new InstantCommand(() -> cornerReset());
    }
    public static void cornerReset() {
        if (currentAlliance == Alliance.BLUE) follower.setPose(new Pose(135, 9, Math.toRadians(0)));
        else follower.setPose(new Pose(9, 9, Math.toRadians(180)));
    }

    @Override
    public void periodic() {
        controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
        controller.updateError(getHeadingError());

        targetHeading = MathFunctions.normalizeAngle(Outtake.INSTANCE.getAngle() + Math.PI);
        Logger.add("Drive", Logger.Level.DEBUG, "target heading? " + targetHeading);
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
                telemetryM.update();

                double forward = slowMode ? -ActiveOpMode.gamepad1().left_stick_y * slowModeMultiplier: -ActiveOpMode.gamepad1().left_stick_y;
                //forward = -forward;
                double strafe = slowMode ? -ActiveOpMode.gamepad1().left_stick_x * slowModeMultiplier: -ActiveOpMode.gamepad1().left_stick_x;
                //strafe = -strafe;
                double turn = slowMode ? -ActiveOpMode.gamepad1().right_stick_x * slowModeMultiplier: -ActiveOpMode.gamepad1().right_stick_x;

                if (headingLock)
                    follower.setTeleOpDrive(forward, strafe, controller.run(), robotCentric);
                else
                    follower.setTeleOpDrive(forward, strafe, turn, robotCentric);

                Logger.add("Drive", Logger.Level.DEBUG, "forward: " + forward + " strafe: " + strafe + " turn: " + turn);
                Logger.add("Drive", Logger.Level.INFO, "slowmode? " + slowMode + "multiplier? " + slowModeMultiplier);
                //Logger.add("Test", LimelightTest, "slowmode? " + slowMode + "multiplier? " + slowModeMultiplier);
            })
            .setStop(interrupted -> {})
            .setIsDone(() -> false)
            .requires(Drive.INSTANCE)
            .setInterruptible(false)
            .named("Drive");
    public double getHeadingError() {
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

package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.math.MathFunctions.normalizeAngle;

import static org.firstinspires.ftc.teamcode.utils.components.AllianceManager.currentAlliance;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

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

    @Override
    public void initialize() {
        follower = Constants.createFollower(ActiveOpMode.hardwareMap());
        follower.setStartingPose(new Pose(8, 6.25, Math.toRadians(0)).mirror());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        cornerReset();
    }

    public void cornerReset() {
        if (currentAlliance == Alliance.BLUE) follower.setPose(new Pose(8, 6.25, Math.toRadians(0)).mirror());
        else follower.setPose(new Pose(8, 6.25, Math.toRadians(0)));
    }

    @Override
    public void periodic() {
        drive.schedule();
    }

    private static void setSlowMode(boolean newMode) {
        slowMode = newMode;
    }
    public static Command setSlowModeCommand(boolean newMode) {
        return new InstantCommand(() -> setSlowMode(newMode));
    }
    public static Command drive = new LambdaCommand()
            .setStart(() -> follower.startTeleopDrive())
            .setUpdate(() -> {
                follower.update();
                telemetryM.update();

                // Calculate the correct values based on Gamepad 1
                double forward = slowMode ? ActiveOpMode.gamepad1().left_stick_y * slowModeMultiplier: ActiveOpMode.gamepad1().left_stick_y;
                double strafe = slowMode ? ActiveOpMode.gamepad1().left_stick_x * slowModeMultiplier: ActiveOpMode.gamepad1().left_stick_x;
                double turn = slowMode ? -ActiveOpMode.gamepad1().right_stick_x * slowModeMultiplier: -ActiveOpMode.gamepad1().right_stick_x;

                follower.setTeleOpDrive(forward, strafe, turn, robotCentric);


                Logger.add("Drive", Logger.Level.DEBUG, "forward: " + forward + " strafe: " + strafe + " turn: " + turn);
                Logger.add("Drive", Logger.Level.DEBUG, "slowmode? " + slowMode + "multiplier? " + slowModeMultiplier);
            })
            .setStop(interrupted -> {})
            .setIsDone(() -> false)
            .requires(Drive.INSTANCE)
            .setInterruptible(false)
            .named("Drive");
}

package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.subsystems.Drive.autoendPose;
import static org.firstinspires.ftc.teamcode.utils.components.AllianceManager.currentAlliance;
import static org.firstinspires.ftc.teamcode.utils.components.AllianceManager.currentLocation;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Transitions;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.Location;
import org.firstinspires.ftc.teamcode.utils.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.utils.components.AllianceManager;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class NoAuto extends NextFTCOpMode {
    {
        addComponents(
                BulkReadComponent.INSTANCE, // TODO: make actual MANUAL mode bulkreading (we don't need to also read the expansion hub every loop)
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                AllianceManager.INSTANCE,
                new SubsystemComponent(
                        Storage.INSTANCE,
                        Robot.INSTANCE,
                        Drive.INSTANCE,
                        Intake.INSTANCE,
                        Outtake.INSTANCE,
                        Transitions.INSTANCE
                ),
                new PedroComponent(Constants::createFollower)
        );
    }


    public static final Pose startPoseBlue = new Pose(56, 8, Math.toRadians(270)); // Start Pose of our robot.

    public static final Pose endPoseBlue = new Pose(30, 8, Math.toRadians(270)); // Start Pose of our robot.



    public static Pose startPose;
    public static Pose endPose;

    boolean blue = false;

    boolean close = false;

    Path lastPath;

    private Command autonomousRoutine() {
        double standardDelay = 0.025;

        return new SequentialGroupFixed(

                new FollowPath(lastPath)
                new  Delay(1.0);
        );
    }


    @Override
    public void onStartButtonPressed() {
        if (currentAlliance == Alliance.BLUE){
            blue = true;
        } else {
            blue = false;
        }

        if (currentLocation == Location.FAR){
            close = false;
        } else {
            close = true;
        }

        Pose startPose;
        Pose endPose;


        if (blue){
            startPose = startPoseBlue;
            endPose = endPoseBlue;

        }

        else{
            startPose = startPoseBlue.mirror();
            endPose = endPoseBlue.mirror();
        }

        System.out.println(startPose.toString());
        System.out.println(endPose.toString());

        lastPath = new Path(new BezierLine(startPose, endPose));

        lastPath.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

        follower().setStartingPose(startPose);
        autonomousRoutine().schedule();
        follower().breakFollowing();

        autoendPose = follower().getPose();
    }

    public void onUpdate(){
        PanelsTelemetry.INSTANCE.getTelemetry().update();
        follower().update();

        autoendPose = follower().getPose();
    }

    @Override
    public void onInit(){
        Storage.zeroEncoderCommand().schedule();
        autoendPose = follower().getPose();
        ActiveOpMode.telemetry().update();
    }

    @Override public void onWaitForStart() {
        ActiveOpMode.telemetry().update();
    }



    public void onStop(){
        endPose = follower().getPose();
        autoendPose = follower().getPose();
    }
}

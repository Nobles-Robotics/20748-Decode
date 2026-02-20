package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.subsystems.Drive.autoendPose;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

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
import org.firstinspires.ftc.teamcode.utils.SequentialGroupFixed;

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
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class AANewMainAuto extends NextFTCOpMode {
    {
        addComponents(
                BulkReadComponent.INSTANCE, // TODO: make actual MANUAL mode bulkreading (we don't need to also read the expansion hub every loop)
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
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

    public static Pose endPose;

    public static final Pose startPoseFarBlue = new Pose(56, 8, Math.toRadians(270)); // Start Pose of our robot.
    public static final Pose startPoseCloseBlue = new Pose(20, 123, Math.toRadians(323)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static final Pose scorePoseBlue = new Pose(56, 81, Math.toRadians(315)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    public static final Pose intakeAlign1Blue = new Pose(45, 84, Math.toRadians(180));
    public static final Pose intake1Blue = new Pose(12, 84, Math.toRadians(180));

    public static final Pose intakeAlign3Blue = new Pose(45, 38, Math.toRadians(180));
    public static final Pose intake3Blue = new Pose(6, 38, Math.toRadians(180));

    private static Pose startPose;
    public static Pose scorePose;

    boolean blue = true;
    boolean close = true;


    Path scorePreloadPath;
    Path intakeAlign1Path;
    Path intake1Path;
    Path score1Path;

//    Path intakeAlign2 = new Path(new BezierLine(scorePose, intakeAlign2Blue));
//    Path intake2 = new Path(new BezierLine(intakeAlign2Blue, intake2Blue));
//    Path score2 = new Path(new BezierLine(intake2Blue, scorePose));

    Path intakeAlign3Path;
    Path intake3Path;
    Path score3Path;


    private Command autonomousRoutine() {
        scorePreloadPath.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        intakeAlign1Path.setLinearHeadingInterpolation(scorePose.getHeading(), intakeAlign1Blue.getHeading());
        intake1Path.setLinearHeadingInterpolation(intakeAlign1Blue.getHeading(), intake1Blue.getHeading());
        score1Path.setLinearHeadingInterpolation(intake1Blue.getHeading(), scorePose.getHeading());

//        intakeAlign2.setLinearHeadingInterpolation(scorePose.getHeading(), intakeAlign2Blue.getHeading());
//        intake2.setLinearHeadingInterpolation(intakeAlign2Blue.getHeading(), intake2Blue.getHeading());
//        score2.setLinearHeadingInterpolation(intake2Blue.getHeading(), scorePose.getHeading());

        intakeAlign3Path.setLinearHeadingInterpolation(scorePose.getHeading(), intakeAlign3Blue.getHeading());
        intake3Path.setLinearHeadingInterpolation(intakeAlign3Blue.getHeading(), intake3Blue.getHeading());
        score3Path.setLinearHeadingInterpolation(intake3Blue.getHeading(), scorePose.getHeading());

        double standardDelay = 0.025;

        return new SequentialGroupFixed(
                new InstantCommand(Outtake.on),
                new FollowPath(scorePreloadPath),
                new Delay(standardDelay),
                Robot.outtakeAll,
                //Robot.intakeOne,
                //Robot.outtakeAll,
                new Delay(standardDelay),
                new InstantCommand(Intake.on()),
                new FollowPath(intakeAlign1Path),
                new Delay(standardDelay),
                new ParallelGroup(
                        new SequentialGroupFixed(
                                new FollowPath(intake1Path, true, 0.5),
                                new Delay (0.05)
                        ),
                        new SequentialGroupFixed(
                                Robot.intakeAll
                        )

                ),
                new Delay(standardDelay),
                new InstantCommand(Outtake.on),
                new FollowPath(score1Path),
                new InstantCommand(Intake.on()),
                new WaitUntil(() -> !follower().isBusy()),
                Robot.outtakeAll,
                new Delay(standardDelay),
                //Robot.intakeOne,
                //Robot.outtakeAll,

                new FollowPath(intakeAlign3Path),
                new Delay(standardDelay),
                new ParallelGroup(
                        new SequentialGroupFixed(
                                new FollowPath(intake3Path, true, 0.6),
                                new Delay (0.05)
                        ),
                        new SequentialGroupFixed(
                                Robot.intakeAll
                        )

                ),
                new Delay(standardDelay),
                new InstantCommand(Outtake.on),
                new InstantCommand(Intake.on()),
                new FollowPath(score3Path),
                new WaitUntil(() -> !follower().isBusy()),
                new Delay(standardDelay),
                Robot.outtakeAll
                //Robot.intakeOne,
                //Robot.outtakeAll

        );
    }

    @Override
    public void onStartButtonPressed() {
        Pose intakeAlign1;
        Pose intake1;
        Pose intakeAlign3;
        Pose intake3;

        if (blue){
            scorePose = scorePoseBlue;
            intakeAlign1=intakeAlign1Blue;
            intake1 = intake1Blue;
            intakeAlign3 = intakeAlign3Blue;
            intake3 = intake3Blue;

            if(close){
                startPose = startPoseCloseBlue;
            }
            else{
                startPose = startPoseFarBlue;
            }
        }
        else{
            scorePose = scorePoseBlue.mirror();
            intakeAlign1=intakeAlign1Blue.mirror();
            intake1 = intake1Blue.mirror();
            intakeAlign3 = intakeAlign3Blue.mirror();
            intake3 = intake3Blue.mirror();

            if(close){
                startPose = startPoseCloseBlue.mirror();
            }
            else{
                startPose = startPoseFarBlue.mirror();
            }
        }

        scorePreloadPath = new Path(new BezierLine(startPose, scorePose));
        intakeAlign1Path = new Path(new BezierLine(scorePose, intakeAlign1));
        intake1Path = new Path(new BezierLine(intakeAlign1, intake1));
        score1Path = new Path(new BezierLine(intake1, scorePose));

        intakeAlign3Path = new Path(new BezierLine(scorePose, intakeAlign3));
        intake3Path = new Path(new BezierLine(intakeAlign3, intake3));
        score3Path = new Path(new BezierLine(intake3, scorePose));

        follower().setStartingPose(startPose);
        autonomousRoutine().schedule();
        follower().breakFollowing();

        autoendPose = follower().getPose();
    }

    public void onUpdate(){
        Drive.telemetryM.update();
        follower().update();

        autoendPose = follower().getPose();
    }

    public void onInit(){
        autoendPose = follower().getPose();
    }



    public void onStop(){
        endPose = follower().getPose();
        autoendPose = follower().getPose();
    }
}

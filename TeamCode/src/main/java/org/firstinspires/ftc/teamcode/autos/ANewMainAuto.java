package org.firstinspires.ftc.teamcode.autos;

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
public class ANewMainAuto extends NextFTCOpMode {
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

    public static final Pose intakeAlign1Blue = new Pose(68, 84, Math.toRadians(180));
    public static final Pose intake1Blue = new Pose(12, 84, Math.toRadians(180));

//    public static final Pose intakeAlign2Blue = new Pose(68, 60, Math.toRadians(180));
//    public static final Pose intake2Blue = new Pose(12, 60, Math.toRadians(180));

    public static final Pose intakeAlign3Blue = new Pose(68, 36, Math.toRadians(180));
    public static final Pose intake3Blue = new Pose(6, 36, Math.toRadians(180));

//
//    public static final Pose startPoseFarRed = new Pose(87, 8, Math.toRadians(270)); // Start Pose of our robot.
//    public static final Pose startPosseCloseRed = new Pose(124, 123, Math.toRadians(37)); // Start Pose of our robot.
//    public static final Pose scorePoseRed = new Pose(76, 76, Math.toRadians(225)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    private final Pose startPose = startPoseCloseBlue;
    public static final Pose scorePose = scorePoseBlue;
    //public static final Pose scorePosebutActually = new Pose(73, 70, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    Path scorePreload = new Path(new BezierLine(startPose, scorePose));
    Path intakeAlign1 = new Path(new BezierLine(scorePose, intakeAlign1Blue));
    Path intake1 = new Path(new BezierLine(intakeAlign1Blue, intake1Blue));
    Path score1 = new Path(new BezierLine(intake1Blue, scorePose));

//    Path intakeAlign2 = new Path(new BezierLine(scorePose, intakeAlign2Blue));
//    Path intake2 = new Path(new BezierLine(intakeAlign2Blue, intake2Blue));
//    Path score2 = new Path(new BezierLine(intake2Blue, scorePose));

    Path intakeAlign3 = new Path(new BezierLine(scorePose, intakeAlign3Blue));
    Path intake3 = new Path(new BezierLine(intakeAlign3Blue, intake3Blue));
    Path score3 = new Path(new BezierLine(intake3Blue, scorePose));


//    public static SequentialGroupFixed intakeAll = new SequentialGroupFixed(
//            new InstantCommand(Intake.on()),
//            new InstantCommand(Storage.setManualModeCommand(true)),
//            new InstantCommand(Storage.setManualPowerCommand(0.75))
//
//    );
//    public static SequentialGroupFixed endIntake = new SequentialGroupFixed(
//            new InstantCommand(Storage.setManualModeCommand(true)),
//            new InstantCommand(Storage.setManualPowerCommand(0)),
//            new InstantCommand(Intake.off())
//
//    );


    double intakeMaxPower = 0.5;

    private Command autonomousRoutine() {
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        intakeAlign1.setLinearHeadingInterpolation(scorePose.getHeading(), intakeAlign1Blue.getHeading());
        intake1.setLinearHeadingInterpolation(intakeAlign1Blue.getHeading(), intake1Blue.getHeading());
        score1.setLinearHeadingInterpolation(intake1Blue.getHeading(), scorePose.getHeading());

//        intakeAlign2.setLinearHeadingInterpolation(scorePose.getHeading(), intakeAlign2Blue.getHeading());
//        intake2.setLinearHeadingInterpolation(intakeAlign2Blue.getHeading(), intake2Blue.getHeading());
//        score2.setLinearHeadingInterpolation(intake2Blue.getHeading(), scorePose.getHeading());

        intakeAlign3.setLinearHeadingInterpolation(scorePose.getHeading(), intakeAlign3Blue.getHeading());
        intake3.setLinearHeadingInterpolation(intakeAlign3Blue.getHeading(), intake3Blue.getHeading());
        score3.setLinearHeadingInterpolation(intake3Blue.getHeading(), scorePose.getHeading());

        double standardDelay = 0.25;

        return new SequentialGroupFixed(
                new FollowPath(scorePreload),
                new Delay(standardDelay),
                Robot.outtakeAll,
                Robot.intakeOne,
                Robot.outtakeAll,
                new Delay(standardDelay),
                new FollowPath(intakeAlign1),
                new Delay(standardDelay),
                new ParallelGroup(
                        new SequentialGroupFixed(
                                new FollowPath(intake1, true, intakeMaxPower),
                                new Delay (1)
                        ),
                        new SequentialGroupFixed(
                                Robot.intakeAll
                        )

                ),
                new Delay(2),
                new FollowPath(score1),
                new WaitUntil(() -> !follower().isBusy()),
                new Delay(standardDelay),
                Robot.outtakeAll,
                Robot.intakeOne,
                Robot.outtakeAll,
                new Delay(standardDelay),

                new FollowPath(intakeAlign3),
                new Delay(standardDelay),
                new ParallelGroup(
                        new SequentialGroupFixed(
                                new FollowPath(intake3, true, intakeMaxPower),
                                new Delay (1)
                        ),
                        new SequentialGroupFixed(
                                Robot.intakeAll
                        )

                ),
                new Delay(2),
                new FollowPath(score3),
                new WaitUntil(() -> !follower().isBusy()),
                new Delay(standardDelay),
                Robot.outtakeAll,
                Robot.intakeOne,
                Robot.outtakeAll
        );
    }

    @Override
    public void onStartButtonPressed() {
        follower().setStartingPose(startPose);
        autonomousRoutine().schedule();
        follower().breakFollowing();
    }

    public void onUpdate(){
        Drive.telemetryM.update();
        follower().update();
    }

    public void onStop(){
        endPose = follower().getPose();
    }
}

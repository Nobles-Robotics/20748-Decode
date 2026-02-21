package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.subsystems.Drive.autoendPose;
import static org.firstinspires.ftc.teamcode.utils.components.AllianceManager.currentAlliance;
import static org.firstinspires.ftc.teamcode.utils.components.AllianceManager.currentLocation;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.telemetry.PanelsTelemetry;
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
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
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
public class AAANewMainAuto extends NextFTCOpMode {
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

    public static Pose endPose;

    public static final Pose startPoseFarBlue = new Pose(56, 8, Math.toRadians(270)); // Start Pose of our robot.
    public static final Pose startPoseCloseBlue = new Pose(20, 123, Math.toRadians(323)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static final Pose scorePoseBlue = new Pose(56, 81, Math.toRadians(315)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    public static final Pose intakeAlign1Blue = new Pose(45, 84, Math.toRadians(180));
    public static final Pose intake1Blue = new Pose(12, 84, Math.toRadians(180));

    public static final Pose intakeAlign3Blue = new Pose(45, 34, Math.toRadians(180));
    public static final Pose intake3Blue = new Pose(9, 34, Math.toRadians(180));

    public static final Pose targetExitPosBlue = new Pose(25, 50, Math.toRadians(315));


    private static Pose startPose;
    public static Pose scorePose;

    boolean blue = false;
    boolean close = false;


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
    Path finalExitPath;


    private Command autonomousRoutine() {
        double standardDelay = 0.025;

        return new SequentialGroupFixed(
                new InstantCommand(Intake.off()),
                new InstantCommand(Transitions.off()),
                new InstantCommand(Outtake.on),
                new FollowPath(scorePreloadPath),
                new Delay(standardDelay),
                Robot.outtakeAllSmooth,
                new Delay(standardDelay),
                new InstantCommand(Intake.on()),
                new InstantCommand(Storage.spinToNextIntakeIndex()),
                new FollowPath(intakeAlign1Path),
                new InstantCommand(Intake.on()),
                new Delay(standardDelay),
                new ParallelDeadlineGroup(
                        new Delay(2),
                        new ParallelGroup(
                            new SequentialGroupFixed(
                                    new InstantCommand(Intake.on()),
                                    new FollowPath(intake1Path, true, 0.5),
                                    new Delay (0.05)
                            ),
                            Robot.intakeAll
                        )
                ),
                new Delay(standardDelay),
                new InstantCommand(Outtake.on),
                new InstantCommand(Storage.spinToNextOuttakeIndex()),
                new InstantCommand(Intake.reverse()),
                new FollowPath(score1Path),
                new WaitUntil(() -> !follower().isBusy()),
                new InstantCommand(Intake.off()),
                Robot.outtakeAllSmooth,
                new Delay(standardDelay),
                new InstantCommand(Storage.spinToNextIntakeIndex()),
                new FollowPath(intakeAlign3Path),
                new InstantCommand(Intake.on()),
                new Delay(standardDelay),
                new ParallelDeadlineGroup(
                        new Delay(2),
                        new ParallelGroup(
                                new SequentialGroupFixed(
                                        new InstantCommand(Intake.on()),
                                        new FollowPath(intake3Path, true, 0.7),
                                        new Delay (0.05)
                                ),
                                Robot.intakeAll
                        )
                ),
                new Delay(standardDelay),
                new InstantCommand(Outtake.on),
                new InstantCommand(Storage.spinToNextOuttakeIndex()),
                new InstantCommand(Intake.reverse()),
                new FollowPath(score3Path),
                new WaitUntil(() -> !follower().isBusy()),
                new InstantCommand(Intake.off()),
                new Delay(standardDelay),
                Robot.outtakeAllSmooth,
                new Delay(standardDelay),
                new FollowPath(finalExitPath)

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

        Pose intakeAlign1;
        Pose intake1;
        Pose intakeAlign3;
        Pose intake3;
        Pose targetExitPos;

        if (blue){
            scorePose = scorePoseBlue;
            intakeAlign1=intakeAlign1Blue;
            intake1 = intake1Blue;
            intakeAlign3 = intakeAlign3Blue;
            intake3 = intake3Blue;
            targetExitPos = targetExitPosBlue;

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
            targetExitPos = targetExitPosBlue.mirror();

            if(close){
                startPose = startPoseCloseBlue.mirror();
            }
            else{
                startPose = startPoseFarBlue.mirror();
            }
        }

        System.out.println(scorePose.toString());
        System.out.println(intakeAlign1.toString());
        System.out.println(intake1.toString());
        System.out.println(intakeAlign3.toString());
        System.out.println(intake3.toString());
        System.out.println(targetExitPos.toString());
        System.out.println(startPose.toString());



        scorePreloadPath = new Path(new BezierLine(startPose, scorePose));
        intakeAlign1Path = new Path(new BezierLine(scorePose, intakeAlign1));
        intake1Path = new Path(new BezierLine(intakeAlign1, intake1));
        score1Path = new Path(new BezierLine(intake1, scorePose));

        intakeAlign3Path = new Path(new BezierLine(scorePose, intakeAlign3));
        intake3Path = new Path(new BezierLine(intakeAlign3, intake3));
        score3Path = new Path(new BezierLine(intake3, scorePose));
        finalExitPath = new Path(new BezierLine(scorePose, targetExitPos));

        scorePreloadPath.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        intakeAlign1Path.setLinearHeadingInterpolation(scorePose.getHeading(), intakeAlign1.getHeading());
        intake1Path.setLinearHeadingInterpolation(intakeAlign1.getHeading(), intake1.getHeading());
        score1Path.setLinearHeadingInterpolation(intake1.getHeading(), scorePose.getHeading());

//        intakeAlign2.setLinearHeadingInterpolation(scorePose.getHeading(), intakeAlign2.getHeading());
//        intake2.setLinearHeadingInterpolation(intakeAlign2.getHeading(), intake2.getHeading());
//        score2.setLinearHeadingInterpolation(intake2.getHeading(), scorePose.getHeading());

        intakeAlign3Path.setLinearHeadingInterpolation(scorePose.getHeading(), intakeAlign3.getHeading());
        intake3Path.setLinearHeadingInterpolation(intakeAlign3.getHeading(), intake3.getHeading());
        score3Path.setLinearHeadingInterpolation(intake3.getHeading(), scorePose.getHeading());
        finalExitPath.setLinearHeadingInterpolation(scorePose.getHeading(), targetExitPos.getHeading());

        intake1Path.setTimeoutConstraint(500);
        intake3Path.setTimeoutConstraint(500);

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
        CommandManager.INSTANCE.cancelAll();
        endPose = follower().getPose();
        autoendPose = follower().getPose();
    }
}

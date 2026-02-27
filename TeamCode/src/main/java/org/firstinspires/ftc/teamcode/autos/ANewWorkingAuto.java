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
public class ANewWorkingAuto extends NextFTCOpMode {
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
    public static final Pose startPoseCloseBlue = new Pose(20, 123, Math.toRadians(323)); // Scoring Pose of our robot.
    public static final Pose scorePoseCloseBlue = new Pose(56, 81, Math.toRadians(315)); // Scoring Pose of our robot.
    static final Pose scorePoseFarBlue = new Pose(59, 18, Math.toRadians(294));
    public static final Pose intakeAlign1Blue = new Pose(45, 84, Math.toRadians(180));
    public static final Pose intake1Blue = new Pose(12, 84, Math.toRadians(180));

    public static final Pose intakeAlign3Blue = new Pose(45, 36, Math.toRadians(180));
    public static final Pose intake3Blue = new Pose(6, 36, Math.toRadians(180));

    public static final Pose IntakePlayerBlue = new Pose(12, 10, Math.toRadians(210));
    public static final Pose IntakeAlignPlayerBlue = new Pose(11,17, Math.toRadians(185));


    public static final Pose targetExitPosBlue = new Pose(50, 35, Math.toRadians(295));

    private static Pose startPose;
    public static Pose scorePose1;
    public static Pose scorePoseGeneral;

    boolean blue = false;
    boolean close = false;


    // CHANGE THIS TO MANIPULATE PATHING
    boolean forceCloseScore1 = true;


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

    Path intakeAlignPlayerPath;
    Path intakePlayerPath;
    Path scorePlayerPath;


    private Command autonomousRoutine() {
        double standardDelay = 0.025; // Slight delay to ensure that everything is settled before moving on

        return new SequentialGroupFixed(

                // Setup
                new InstantCommand(Intake.off()),
                new InstantCommand(Transitions.off()),

                // Scoring Preload
                new InstantCommand(Outtake.on),
                new FollowPath(scorePreloadPath),
                new Delay(standardDelay),
                Robot.outtakeAllSmooth(close),
                new Delay(standardDelay),

                // Intake1
                new InstantCommand(Intake.on()),
                new InstantCommand(Storage.spinToNextIntakeIndex()),
                new FollowPath(intakeAlign1Path),
                new InstantCommand(Intake.on()),
                new Delay(standardDelay),
                new ParallelGroup(
                        new SequentialGroupFixed(
                                new InstantCommand(Intake.on()),
                                new FollowPath(intake1Path, true, 0.5),
                                new Delay (0.05)
                        ),
                        new SequentialGroupFixed(
                                Robot.intakeAll
                        )
                ),
                new Delay(standardDelay),

                // Scoring after Intake1
                new InstantCommand(Outtake.on),
                new InstantCommand(Storage.spinToNextOuttakeIndex()),
                new FollowPath(score1Path),
                new InstantCommand(Intake.off()),
                new InstantCommand(Intake.reverse()),
                new WaitUntil(() -> !follower().isBusy()),
                new InstantCommand(Intake.off()),
                Robot.outtakeAllSmooth(forceCloseScore1),
                new Delay(standardDelay),

                // Intake3
                new InstantCommand(Storage.spinToNextIntakeIndex()),
                new FollowPath(intakeAlign3Path),
                new InstantCommand(Intake.on()),
                new Delay(standardDelay),
                new ParallelGroup(
                        new SequentialGroupFixed(
                                new InstantCommand(Intake.off()),
                                new FollowPath(intake3Path, true, 0.5),
                                new Delay (0.05)
                        ),
                        new SequentialGroupFixed(
                                Robot.intakeAll
                        )

                ),
                new Delay(standardDelay),

                // Scoring after Intake3
                new InstantCommand(Outtake.on),
                new InstantCommand(Storage.spinToNextOuttakeIndex()),
                new FollowPath(score3Path),
                new InstantCommand(Intake.off()),
                new InstantCommand(Intake.reverse()),
                new WaitUntil(() -> !follower().isBusy()),
                new InstantCommand(Intake.off()),
                new Delay(standardDelay),
                Robot.outtakeAllSmooth(close),
                new Delay(standardDelay),

                // IntakePlayer
                new InstantCommand(Intake.on()),
                new InstantCommand(Storage.spinToNextIntakeIndex()),
                new FollowPath(intakeAlignPlayerPath),
                new InstantCommand(Intake.on()),
                new Delay(standardDelay),
                new ParallelGroup(
                        new SequentialGroupFixed(
                                new InstantCommand(Intake.off()),
                                new FollowPath(intakePlayerPath, true, 0.5),
                                new Delay (0.5)
                        ),
                        new SequentialGroupFixed(
                                Robot.intakeAll
                        )

                ),
                new Delay(standardDelay),

                // Scoring after IntakePlayer
                new InstantCommand(Outtake.on),
                new InstantCommand(Storage.spinToNextOuttakeIndex()),
                new FollowPath(scorePlayerPath),
                new InstantCommand(Intake.off()),
                new InstantCommand(Intake.reverse()),
                new WaitUntil(() -> !follower().isBusy()),
                new InstantCommand(Intake.off()),
                new Delay(standardDelay),
                Robot.outtakeAllSmooth(close),
                new Delay(standardDelay),

                // Park for leave points
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
        Pose intakeAlignPlayer;
        Pose intakePlayer;

        if (blue){
            intakeAlign1=intakeAlign1Blue;
            intake1 = intake1Blue;
            intakeAlign3 = intakeAlign3Blue;
            intake3 = intake3Blue;
            targetExitPos = targetExitPosBlue;
            intakeAlignPlayer = IntakeAlignPlayerBlue;
            intakePlayer = IntakePlayerBlue;



            if(close){
                startPose = startPoseCloseBlue;
                scorePoseGeneral = scorePoseCloseBlue;
            }
            else{
                startPose = startPoseFarBlue;
                scorePoseGeneral = scorePoseFarBlue;
            }

            if (forceCloseScore1){
                scorePose1 = scorePoseCloseBlue;
            }
            else{
                scorePose1 = scorePoseGeneral;
            }
        }

        else{
            scorePoseGeneral = scorePoseFarBlue.mirror();
            intakeAlign1=intakeAlign1Blue.mirror();
            intake1 = intake1Blue.mirror();
            intakeAlign3 = intakeAlign3Blue.mirror();
            intake3 = intake3Blue.mirror();
            targetExitPos = targetExitPosBlue.mirror();
            intakeAlignPlayer = IntakeAlignPlayerBlue.mirror();
            intakePlayer = IntakePlayerBlue.mirror();

            if(close){
                startPose = startPoseCloseBlue.mirror();
                scorePoseGeneral = scorePoseCloseBlue.mirror();

            }
            else{
                startPose = startPoseFarBlue.mirror();
                scorePoseGeneral = scorePoseFarBlue.mirror();
            }

            if (forceCloseScore1){
                scorePose1 = scorePoseCloseBlue.mirror();
            }
            else{
                scorePose1 = scorePoseGeneral;
            }
        }

        System.out.println(scorePoseGeneral.toString());
        System.out.println(intakeAlign1.toString());
        System.out.println(intake1.toString());
        System.out.println(intakeAlign3.toString());
        System.out.println(intake3.toString());
        System.out.println(targetExitPos.toString());
        System.out.println(startPose.toString());



        scorePreloadPath = new Path(new BezierLine(startPose, scorePoseGeneral));
        intakeAlign1Path = new Path(new BezierLine(scorePoseGeneral, intakeAlign1));
        intake1Path = new Path(new BezierLine(intakeAlign1, intake1));
        score1Path = new Path(new BezierLine(intake1, scorePose1));

        intakeAlign3Path = new Path(new BezierLine(scorePose1, intakeAlign3));
        intake3Path = new Path(new BezierLine(intakeAlign3, intake3));
        score3Path = new Path(new BezierLine(intake3, scorePoseGeneral));
        finalExitPath = new Path(new BezierLine(scorePoseGeneral, targetExitPos));

        intakeAlignPlayerPath = new Path(new BezierLine(scorePoseGeneral, intakeAlignPlayer));
        intakePlayerPath = new Path(new BezierLine(intakeAlignPlayer, intakePlayer));
        scorePlayerPath = new Path(new BezierLine(intakePlayer, scorePoseGeneral));


        scorePreloadPath.setLinearHeadingInterpolation(startPose.getHeading(), scorePoseGeneral.getHeading());
        intakeAlign1Path.setLinearHeadingInterpolation(scorePoseGeneral.getHeading(), intakeAlign1.getHeading());
        intake1Path.setLinearHeadingInterpolation(intakeAlign1.getHeading(), intake1.getHeading());
        score1Path.setLinearHeadingInterpolation(intake1.getHeading(), scorePose1.getHeading());

//        intakeAlign2.setLinearHeadingInterpolation(scorePose.getHeading(), intakeAlign2.getHeading());
//        intake2.setLinearHeadingInterpolation(intakeAlign2.getHeading(), intake2.getHeading());
//        score2.setLinearHeadingInterpolation(intake2.getHeading(), scorePose.getHeading());

        intakeAlign3Path.setLinearHeadingInterpolation(scorePose1.getHeading(), intakeAlign3.getHeading());
        intake3Path.setLinearHeadingInterpolation(intakeAlign3.getHeading(), intake3.getHeading());
        score3Path.setLinearHeadingInterpolation(intake3.getHeading(), scorePoseGeneral.getHeading());
        finalExitPath.setLinearHeadingInterpolation(scorePoseGeneral.getHeading(), targetExitPos.getHeading());
        intakeAlignPlayerPath.setLinearHeadingInterpolation(scorePoseGeneral.getHeading(), intakeAlignPlayer.getHeading());
        intakePlayerPath.setLinearHeadingInterpolation(intakeAlignPlayer.getHeading(), intakePlayer.getHeading());
        scorePlayerPath.setLinearHeadingInterpolation(intakePlayer.getHeading(), scorePoseGeneral.getHeading());


        intake1Path.setTimeoutConstraint(1000);
        intake3Path.setTimeoutConstraint(1000);
        scorePlayerPath.setTimeoutConstraint(1000);

        follower().setStartingPose(startPose);
        autonomousRoutine().schedule();
        follower().breakFollowing();

        autoendPose = follower().getPose();
    }

    public void onUpdate(){
        PanelsTelemetry.INSTANCE.getTelemetry().update();
        follower().update();

        autoendPose = follower().getPose();
        Drawing.drawDebug(follower());
    }

    @Override
    public void onInit(){
        Storage.zeroEncoderCommand().schedule();
        autoendPose = follower().getPose();
        ActiveOpMode.telemetry().update();
        Drawing.init();
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

package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.subsystems.Drive.autoendPose;
import static org.firstinspires.ftc.teamcode.utils.components.AllianceManager.currentAlliance;
import static org.firstinspires.ftc.teamcode.utils.components.AllianceManager.currentLocation;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import android.os.SystemClock;
import android.util.Log;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
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

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.CommandGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
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

    // Sets start position (manual setup during init)
    boolean blue = false;
    boolean close = false;

    // TODO: CHANGE BEFORE EVERY MATCH
    // Sets intake paths (true = runs path, false = skips)
    public static boolean runIntake1 = true;
    public static boolean runIntake2 = true;
    public static boolean runIntake3 = true;

    // --- SETTINGS ---

    // CHANGE THIS TO MANIPULATE PATHING
    // Set true if have full field control
    // Set false if shoot far, or if skipping
    boolean forceCloseScore1 = false;

    // Sets exit pose to close
    boolean startCloseForCloseExit = true; // This should always be true

    // Max time for intake before cuts loss
    double intakeMaxTime = 1;

    ///  ---------------

    public static Pose endPose;
    public static final Pose startPoseFarBlue = new Pose(56, 8, Math.toRadians(270)); // Start Pose of our robot.
    public static final Pose startPoseCloseBlue = new Pose(20, 123, Math.toRadians(323)); // Scoring Pose of our robot.
    public static final Pose scorePoseCloseBlue = new Pose(56, 81, Math.toRadians(315)); // Scoring Pose of our robot.
    public static final Pose scorePoseFarBlue = new Pose(59, 18, Math.toRadians(293.83));

    public static final Pose intakeAlign1Blue = new Pose(45, 84, Math.toRadians(180));
    public static final Pose intake1Blue = new Pose(12, 84, Math.toRadians(180));

    public static final Pose intakeAlign2Blue = new Pose(45, 58, Math.toRadians(180));
    public static final Pose intake2Blue = new Pose(6, 58, Math.toRadians(180));

    public static final Pose intakeAlign3Blue = new Pose(45, 36, Math.toRadians(180));
    public static final Pose intake3Blue = new Pose(6, 36, Math.toRadians(180));

    public static final Pose IntakePlayerBlue = new Pose(12, 10, Math.toRadians(210));
    public static final Pose IntakeAlignPlayerBlue = new Pose(11,17, Math.toRadians(185));


    public static final Pose targetExitPosFarBlue = new Pose(50, 35, Math.toRadians(295));
    public static final Pose targetExitPosCloseBlue = new Pose(56, 112, Math.toRadians(315));


    private static Pose startPose;
    public static Pose scorePose1;
    public static Pose scorePoseGeneral;


    Pose intakeAlign1;
    Pose intake1;
    Pose intakeAlign2;
    Pose intake2;
    Pose intakeAlign3;
    Pose intake3;
    Pose targetExitPos;
    Pose intakeAlignPlayer;
    Pose intakePlayer;


    Path scorePreloadPath;
    Path intakeAlign1Path;
    Path intake1Path;
    Path score1Path;

    Path intakeAlign2Path;
    Path intake2Path;
    Path intakeAlign2OutPath;
    Path score2Path;
    PathChain intake2Score2PathChain;

    Path intakeAlign3Path;
    Path intake3Path;
    Path score3Path;
    Path finalExitPath;

    Path intakeAlignPlayerPath;
    Path intakePlayerPath;
    Path scorePlayerPath;


    private Command autonomousRoutine() {
        double standardDelay = 0.025; // Slight delay to ensure that everything is settled before moving on

        SequentialGroupFixed setupGroup = new SequentialGroupFixed(

                // Setup
                new ParallelGroup(
                        new InstantCommand(Intake.off()),
                        new InstantCommand(Transitions.off()),
                        new InstantCommand(Robot.setTargetVelocityAuto(close))
                )
        );
        SequentialGroupFixed scorePreloadGroup = new SequentialGroupFixed(

                // Scoring Preload
                new ParallelGroup(
                    new InstantCommand(Outtake.on),
                    new FollowPath(scorePreloadPath)
                ),
                new WaitUntil(() -> !follower().isBusy()),
                new Delay(standardDelay*3),
                new InstantCommand(Intake.on()),
                Robot.outtakeAllSmooth(close),
                new Delay(standardDelay)
        );

        SequentialGroupFixed intake2Group = new SequentialGroupFixed(

                // Intake2
                new ParallelGroup(
                        new InstantCommand(Intake.on()),
                        new InstantCommand(Storage.spinToNextIntakeIndex()),
                        new FollowPath(intakeAlign2Path)
                ),
                new Delay(standardDelay),
                new ParallelGroup(
                        new ParallelRaceGroup(
                                new SequentialGroupFixed(
                                        new InstantCommand(Intake.on()),
                                        new FollowPath(intake2Path, true, 0.5),
                                        new Delay (0.05)
                                ),
                                new Delay(intakeMaxTime)
                        ),
                        new SequentialGroupFixed(
                                Robot.intakeAll
                        )
                ),
                new Delay(standardDelay),
                new ParallelGroup(
                        new InstantCommand(Outtake.on),
                        new InstantCommand(Storage.spinToNextOuttakeIndex()),
                        new InstantCommand(Intake.on()),
                        new FollowPath(intake2Score2PathChain, false)
                ),
                new WaitUntil(() -> !follower().isBusy()),
                new Delay(standardDelay),
                new InstantCommand(Intake.on()),
                Robot.outtakeAllSmooth(close),
                new Delay(standardDelay)
        );

        SequentialGroupFixed intake1Group = new SequentialGroupFixed(

                // Intake1
                new ParallelGroup(
                        new InstantCommand(Intake.reverse()),
                        new InstantCommand(Storage.spinToNextIntakeIndex()),
                        new FollowPath(intakeAlign1Path)
                ),
                new Delay(standardDelay),
                new ParallelGroup(
                        Intake.on(),
                        new ParallelRaceGroup(
                                new SequentialGroupFixed(
                                        new InstantCommand(Intake.on()),
                                        new FollowPath(intake1Path, true, 0.5),
                                        new Delay (0.05)
                                ),
                                new Delay(intakeMaxTime)
                        ),
                        new SequentialGroupFixed(
                                Robot.intakeAll
                        )
                ),
                new InstantCommand(Intake.on()),
                new Delay(standardDelay),
                // Scoring after Intake1
                new ParallelGroup(
                    new InstantCommand(Outtake.on),
                    new InstantCommand(Storage.spinToNextOuttakeIndex()),
//                    new SequentialGroupFixed(
//                        new Delay(0.1),
//                        new InstantCommand(Intake.reverse())
//                    ),
                    new InstantCommand(Intake.on()),
                    new FollowPath(score1Path)
                ),
                new WaitUntil(() -> !follower().isBusy()),
                new Delay(standardDelay),
                new InstantCommand(Intake.on()),
                Robot.outtakeAllSmooth(close), //forceCloseScore1
                new Delay(standardDelay)
        );

        SequentialGroupFixed intake3Group = new SequentialGroupFixed(

                // Intake3
                new ParallelGroup(
                        new InstantCommand(Intake.reverse()),
                        new InstantCommand(Storage.spinToNextIntakeIndex()),
                        new FollowPath(intakeAlign3Path)
                ),
                new Delay(standardDelay),
                new ParallelGroup(
                        new InstantCommand(Intake.on()),
                        new ParallelRaceGroup(
                                new SequentialGroupFixed(
                                        new InstantCommand(Intake.on()),
                                        new FollowPath(intake3Path, true, 0.5),
                                        new Delay (0.05)
                                ),
                                new Delay(intakeMaxTime)
                        ),
                        new SequentialGroupFixed(
                                Robot.intakeAll
                        )

                ),
                new InstantCommand(Intake.on()),
                new Delay(standardDelay),

                // Scoring after Intake3
                new ParallelGroup(
                        new InstantCommand(Outtake.on),
                        new InstantCommand(Storage.spinToNextOuttakeIndex()),

//                    new SequentialGroupFixed(
//                        new Delay(0.1),
//                        new InstantCommand(Intake.reverse())
//                    ),
                        new InstantCommand(Intake.on()),
                        new FollowPath(score3Path)
                ),
                new WaitUntil(() -> !follower().isBusy()),
                new Delay(standardDelay),
                new InstantCommand(Intake.on()),
                new Delay(standardDelay),
                Robot.outtakeAllSmooth(close),
                new Delay(standardDelay)
        );

        SequentialGroupFixed exitGroup = new SequentialGroupFixed(

                // Park for leave points
                new FollowPath(finalExitPath),

                // Turn off everything
                new InstantCommand(Intake.off()),
                new InstantCommand(Transitions.off())
        );

        // Close start = disable intake3Group
        // Far start = disable intake1Group
        List<Object> groups = new ArrayList<>();

        groups.add(setupGroup);
        groups.add(scorePreloadGroup);
        if (runIntake2){
            groups.add(intake2Group);
        }
        if (runIntake1){
            groups.add(intake1Group);
        }
        if (runIntake3){
            groups.add(intake3Group);
        }
        groups.add(exitGroup);

        return new SequentialGroupFixed(groups.toArray(new SequentialGroupFixed[0]));


//        return new SequentialGroupFixed(
//                setupGroup,
//                scorePreloadGroup,
//                intake2Group,
//                intake1Group,
//                intake3Group,
//                exitGroup
//        );
    }


    @Override
    public void onStartButtonPressed() {
        final long startButtonPressedNs = SystemClock.elapsedRealtimeNanos();

        // Start position management set during init
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

        // Build auto commands based on settings
        if (blue){
            intakeAlign1=intakeAlign1Blue;
            intake1 = intake1Blue;
            intakeAlign2 = intakeAlign2Blue;
            intake2 = intake2Blue;
            intakeAlign3 = intakeAlign3Blue;
            intake3 = intake3Blue;
            intakeAlignPlayer = IntakeAlignPlayerBlue;
            intakePlayer = IntakePlayerBlue;

            if(close){
                startPose = startPoseCloseBlue;
                scorePoseGeneral = scorePoseCloseBlue;
                if(startCloseForCloseExit){
                    targetExitPos = targetExitPosCloseBlue;
                }
                else{
                    targetExitPos = targetExitPosFarBlue;
                }
            }
            //FAR
            else{
                startPose = startPoseFarBlue;
                scorePoseGeneral = scorePoseFarBlue;
                targetExitPos = targetExitPosFarBlue;
            }

            if (forceCloseScore1){
                scorePose1 = scorePoseCloseBlue;
            }
            else{
                scorePose1 = scorePoseGeneral;
            }
        }

        // RED
        else{
            intakeAlign1=intakeAlign1Blue.mirror();
            intake1 = intake1Blue.mirror();
            intakeAlign2 = intakeAlign2Blue.mirror();
            intake2 = intake2Blue.mirror();
            intakeAlign3 = intakeAlign3Blue.mirror();
            intake3 = intake3Blue.mirror();
            intakeAlignPlayer = IntakeAlignPlayerBlue.mirror();
            intakePlayer = IntakePlayerBlue.mirror();


            if(close){
                startPose = startPoseCloseBlue.mirror();
                scorePoseGeneral = scorePoseCloseBlue.mirror();

                if(startCloseForCloseExit){
                    targetExitPos = targetExitPosCloseBlue.mirror();
                }
                else{
                    targetExitPos = targetExitPosFarBlue.mirror();
                }
            }
            //FAR
            else{
                startPose = startPoseFarBlue.mirror();
                scorePoseGeneral = scorePoseFarBlue.mirror();
                targetExitPos = targetExitPosFarBlue.mirror();
            }

            if (forceCloseScore1){
                scorePose1 = scorePoseCloseBlue.mirror();
            }
            else{
                scorePose1 = scorePoseGeneral;
            }
        }

        scorePreloadPath = buildPath(startPose, scorePoseGeneral, false);

        intakeAlign2Path = buildPath(scorePoseGeneral, intakeAlign2, false);
        intake2Path = buildPath(intakeAlign2, intake2, false);
        intakeAlign2OutPath = buildPath(intake2, intakeAlign2, false);
        score2Path = buildPath(intakeAlign2, scorePoseGeneral, false);
        intake2Score2PathChain = follower().pathBuilder()
                .addPaths(intakeAlign2OutPath, score2Path)
                .build();

        intakeAlign1Path = buildPath(scorePoseGeneral, intakeAlign1, false);
        intake1Path = buildPath(intakeAlign1, intake1, false);
        score1Path = buildPath(intake1, scorePose1, false);

        intakeAlign3Path = buildPath(scorePose1, intakeAlign3, false);
        intake3Path = buildPath(intakeAlign3, intake3, false);
        score3Path = buildPath(intake3, scorePoseGeneral, false);
        finalExitPath = buildPath(scorePoseGeneral, targetExitPos, false);

        intakeAlignPlayerPath = buildPath(scorePoseGeneral, intakeAlignPlayer, false);
        intakePlayerPath = buildPath(intakeAlignPlayer, intakePlayer, false);
        scorePlayerPath = buildPath(intakePlayer, scorePoseGeneral, false);

        // Ensures the robot doesn't get stuck on an intake cycle if the balls are jammed in front of it
        intake1Path.setTimeoutConstraint(intakeMaxTime*1000);
        intake2Path.setTimeoutConstraint(intakeMaxTime*1000);
        intake3Path.setTimeoutConstraint(intakeMaxTime*1000);
        scorePlayerPath.setTimeoutConstraint(intakeMaxTime*1000);

        // Starts auto
        follower().setStartingPose(startPose);

        final long scheduleStartNs = SystemClock.elapsedRealtimeNanos();
        final double msToScheduleStart = (scheduleStartNs - startButtonPressedNs) / 1_000_000.0;
        Log.i("ANewWorkingAuto", String.format("Auto start latency: %.3f ms from start press to autonomousRoutine.schedule()", msToScheduleStart));

        autonomousRoutine().schedule();
        follower().breakFollowing();

        // Sets autoendPose to correctly initiate the teleop
        autoendPose = follower().getPose();

//        System.out.println(scorePoseGeneral.toString());
//        System.out.println(intakeAlign1.toString());
//        System.out.println(intake1.toString());
//        System.out.println(intakeAlign3.toString());
//        System.out.println(intake3.toString());
//        System.out.println(targetExitPos.toString());
//        System.out.println(startPose.toString());
    }

    public void onUpdate(){
        PanelsTelemetry.INSTANCE.getTelemetry().update();
        //follower().update();

        autoendPose = follower().getPose();
        Drawing.drawDebug(follower());
    }

    @Override
    public void onInit(){
        Storage.zeroEncoderCommand().schedule(); // Storage spindexer starts in intake position
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


    /**
     * Helper to build a Path with linear heading interpolation between two poses.
     */
    private Path buildPath(Pose from, Pose to, boolean constantHeadingInterpolation) {
        Path path = new Path(new BezierLine(from, to));
        if(constantHeadingInterpolation){
            path.setConstantHeadingInterpolation(to.getHeading());
        }
        else{
            path.setLinearHeadingInterpolation(from.getHeading(), to.getHeading());

        }
        return path;
    }

}


//                // IntakePlayer
//                new InstantCommand(Intake.on()),
//                new InstantCommand(Storage.spinToNextIntakeIndex()),
//                new FollowPath(intakeAlignPlayerPath),
//                new InstantCommand(Intake.on()),
//                new Delay(standardDelay),
//                new ParallelGroup(
//                        new SequentialGroupFixed(
//                                new InstantCommand(Intake.on()),
//                                new FollowPath(intakePlayerPath, true, 0.5),
//                                new Delay (0.5)
//                        ),
//                        new SequentialGroupFixed(
//                                Robot.intakeAll
//                        )
//
//                ),
//                new InstantCommand(Intake.on()),
//                new Delay(standardDelay),
//
//                // Scoring after IntakePlayer
//                new InstantCommand(Outtake.on),
//                new InstantCommand(Storage.spinToNextOuttakeIndex()),
//                new FollowPath(scorePlayerPath),
//                new InstantCommand(Intake.on()),
//                new InstantCommand(Intake.reverse()),
//                new WaitUntil(() -> !follower().isBusy()),
//                new InstantCommand(Intake.on()),
//                new Delay(standardDelay),
//                Robot.outtakeAllSmooth(close),
//                new Delay(standardDelay),

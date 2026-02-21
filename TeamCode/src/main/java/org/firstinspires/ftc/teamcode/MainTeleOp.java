package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Transitions;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.components.AllianceManager;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.utils.components.LoopTimeComponent;
import org.firstinspires.ftc.teamcode.utils.photoncore.PhotonCore;

@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends NextFTCOpMode {
    {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                AllianceManager.INSTANCE,
                LoopTimeComponent.INSTANCE,
                new SubsystemComponent(
                        Storage.INSTANCE,
                        Robot.INSTANCE,
                        Drive.INSTANCE,
                        Intake.INSTANCE,
                        Outtake.INSTANCE,
                        Transitions.INSTANCE
                )
        );
    }

    private static final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    @Override public void onInit() {
//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(8); // Can be adjusted based on user preference - but raising this number further can cause issues
//        PhotonCore.enable();
    }

    @Override public void onWaitForStart() {
        ActiveOpMode.telemetry().update();
        panelsTelemetry.getTelemetry().update();
    }

    @Override public void onStartButtonPressed() {


        GamepadEx gp1 = Gamepads.gamepad1();
        GamepadEx gp2 = Gamepads.gamepad2();

         /*
         GAMEPAD SCHEMA
         sticks
         a | b | x | y
         up | down | left | right
         left bumper | right bumper
         left trigger | right trigger
         start | back
         */

        gp1.start()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Robot.optimizeLoopTimes(true))
                .whenBecomesFalse(() -> Robot.optimizeLoopTimes(false));

        gp1.a()
                .whenBecomesTrue(() -> Storage.assertManualPower(-0.25).schedule())
                .whenBecomesFalse(() -> Storage.assertManualPower(0).schedule());

        gp1.b()
                .whenBecomesTrue(() -> Storage.assertManualPower(-0.1).schedule())
                .whenBecomesFalse(() -> Storage.assertManualPower(0).schedule());

        gp1.x()
                .whenBecomesTrue(() -> {
                    Storage.spinToNextIntakeIndex().schedule();
                    Intake.on().schedule();
                })
                .whenBecomesFalse(() -> Intake.off().schedule());

        gp1.y()
            .whenBecomesTrue(() -> Robot.outtakeAll.schedule());

        gp1.dpadUp()
                .whenBecomesTrue(() -> Storage.resetEncoderCommand().schedule());

        gp1.dpadDown()
                .whenBecomesTrue(() -> Storage.spinToNextIntakeIndex().schedule());

        gp1.dpadLeft()
                .whenBecomesTrue(() -> Intake.on().schedule())
                .whenBecomesFalse(() -> Intake.off().schedule());

        gp1.dpadRight()
                .whenBecomesTrue(() -> Transitions.on().schedule())
                .whenBecomesFalse(() -> Transitions.off().schedule());

        gp1.back()
                .whenBecomesTrue(() -> Drive.cornerResetCommand().schedule());

        // ============================================================================================================================

        gp1.leftBumper()
                .whenBecomesTrue(() -> Intake.reverse().schedule())
                .whenBecomesFalse(() -> Intake.off().schedule());

//        gamepad2.leftBumper()
//                .whenBecomesTrue(() -> {
//                    Outtake.on.schedule();
//                    Outtake.setTargetVelocity(2650);
//                })
//                .whenBecomesFalse(() -> Outtake.off.schedule());

        gp1.rightTrigger().atLeast(.3)
                .whenBecomesTrue(() -> Drive.setSlowModeCommand(true).schedule())
                .whenBecomesFalse(() -> Drive.setSlowModeCommand(false).schedule());

        gp2.leftTrigger().atLeast(.9)
                .whenBecomesTrue(() -> Outtake.assertManualPower(-1).schedule())
                .whenBecomesFalse(() -> Outtake.assertManualPower(0).schedule());

        gp1.leftTrigger().atLeast(.5)
                .whenBecomesTrue(() -> Drive.setHeadingLockCommand(true).schedule())
                .whenBecomesFalse(() -> Drive.setHeadingLockCommand(false).schedule());

        gp2.start()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Robot.optimizeLoopTimes(true))
                .whenBecomesFalse(() -> Robot.optimizeLoopTimes(false));

        gp2.a()
                .whenBecomesTrue(() -> Storage.assertManualPower(0.75).schedule())
                .whenBecomesFalse(() -> Storage.assertManualPower(0).schedule());
        gp2.y()
                .whenBecomesTrue(() -> Storage.assertManualPower(0.3).schedule())
                .whenBecomesFalse(() -> Storage.assertManualPower(0).schedule());

        gp2.x()
                .whenBecomesTrue(() -> Storage.assertManualPower(1).schedule())
                .whenBecomesFalse(() -> Storage.assertManualPower(0).schedule());

        gp2.b()
                .whenBecomesTrue(() -> Storage.assertManualPower(0.5).schedule())
                .whenBecomesFalse(() -> Storage.assertManualPower(0).schedule());

        gp2.dpadLeft()
                .whenBecomesTrue(() -> {
                    Storage.spinToNextIntakeIndex().schedule();
                    Intake.on().schedule();
                })
                .whenBecomesFalse(() -> Intake.off().schedule());

        gp1.rightBumper()
                .whenBecomesTrue(() -> {
                    Storage.spinToNextIntakeIndex().schedule();
                    Intake.on().schedule();
                })
                .whenBecomesFalse(() -> Intake.off().schedule());

        gp2.dpadRight()
                .whenBecomesTrue(() -> Robot.outtakeAllClose.schedule());
        gp2.dpadLeft()
                .whenBecomesTrue(() -> Robot.outtakeAllFar.schedule());

        gp2.dpadUp()
                .whenBecomesTrue(() -> Intake.on().schedule())
                .whenBecomesFalse(() -> Intake.off().schedule());

        gp2.leftBumper()
                .whenBecomesTrue(() -> Transitions.on().schedule())
                .whenBecomesFalse(() -> Transitions.off().schedule());

        gp2.back()
                .whenBecomesTrue(() -> Drive.cornerResetCommand().schedule());



        // ============================================================================================================================

        gp2.dpadDown()
                .whenBecomesTrue(() -> Intake.reverse().schedule())
                .whenBecomesFalse(() -> Intake.off().schedule());

        gp2.rightBumper()
                .whenBecomesTrue(() -> {
                    Outtake.on.schedule();
                    Outtake.setTargetVelocity(1850);
                })
                .whenBecomesFalse(() -> Outtake.off.schedule());
        gp2.rightTrigger().atLeast(.5)
                .whenBecomesTrue(() -> {
                    Outtake.on.schedule();
                    Outtake.setTargetVelocity(2125);
                })
                .whenBecomesFalse(() -> Outtake.off.schedule());

    }


    @Override public void onUpdate() {
        //PhotonCore.CONTROL_HUB.clearBulkCache();
        //PhotonCore.EXPANSION_HUB.clearBulkCache();

//        for (String cname : CommandManager.INSTANCE.snapshot()) {
//            Logger.add("Commands", cname);
//        }
//        Logger.update();

        ActiveOpMode.telemetry().update();
//        panelsTelemetry.getTelemetry().update();
    }

    @Override public void onStop() {
    }
}

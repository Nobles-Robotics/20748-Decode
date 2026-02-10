package org.firstinspires.ftc.teamcode;

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
                        Transitions.INSTANCE,
                        Limelight.INSTANCE
                )
        );
    }
    @Override public void onWaitForStart() {
        ActiveOpMode.telemetry().update();
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

         both
         a = storage .75 | b = storage .5 | x = storage .2| y = transition
         up = nextIntake | down = nextOuttake | left = requestAlign | right = outtakeAll
         start = nothing| back = cornerReset

         gp1
         sticks = drive
         left bumper = intake back | right bumper = intake forward
         left trigger = heading lock | right trigger = slowmode

         gp2
         sticks = nothing
         left bumper = outtake far | right bumper = outtake close
         left trigger = nothing| right trigger = nothing
         */

        gp1.a().or(gp2.a())
                .whenBecomesTrue(() -> Storage.assertManualPower(0.75).schedule())
                .whenBecomesFalse(() -> Storage.assertManualPower(0).schedule());
        gp1.b().or(gp2.b())
                .whenBecomesTrue(() -> Storage.assertManualPower(0.5).schedule())
                .whenBecomesFalse(() -> Storage.assertManualPower(0).schedule());

        gp1.x().or(gp2.x()).or(gp2.leftBumper())
                .whenBecomesTrue(() -> Transitions.on().schedule())
                .whenBecomesFalse(() -> Transitions.off().schedule());

        gp1.y().or(gp2.y())
                .whenBecomesTrue(() -> Storage.assertManualPower(0.2).schedule())
                .whenBecomesFalse(() -> Storage.assertManualPower(0).schedule());

        gp1.dpadUp().or(gp2.dpadUp())
                //.whenBecomesTrue(() -> Storage.spinToNextIntakeIndex().schedule());
                .whenBecomesTrue(() -> Intake.on().schedule())
                .whenBecomesFalse(() -> Intake.off().schedule());

        gp1.dpadDown().or(gp2.dpadDown())
                //.whenBecomesTrue(() -> Storage.spinToNextOuttakeIndex().schedule());
                .whenBecomesTrue(() -> Intake.reverse().schedule())
                .whenBecomesFalse(() -> Intake.off().schedule());

        gp1.dpadLeft().or(gp2.dpadLeft())
                .whenBecomesTrue(() -> Storage.requestAlign(1).schedule());

        gp1.dpadRight().or(gp2.dpadRight())
                .whenBecomesTrue(() -> Robot.outtakeAll.schedule());

        gp1.back().or(gp2.back())
                .whenBecomesTrue(() -> Drive.cornerResetCommand().schedule());

        // ===========================================================================================================================

        gp1.leftBumper()
                .whenBecomesTrue(() -> Intake.reverse().schedule())
                .whenBecomesFalse(() -> Intake.off().schedule());

        gp2.rightBumper()
                .whenBecomesTrue(() -> {
                    Outtake.on.schedule();
                    Outtake.setTargetVelocity(2400);
                })
                .whenBecomesFalse(() -> Outtake.off.schedule());

//        gamepad2.leftBumper()
//                .whenBecomesTrue(() -> {
//                    Outtake.on.schedule();
//                    Outtake.setTargetVelocity(2650);
//                })
//                .whenBecomesFalse(() -> Outtake.off.schedule());

        gp1.rightTrigger().atLeast(.5)
                .whenBecomesTrue(() -> Drive.setSlowModeCommand(true).schedule())
                .whenBecomesFalse(() -> Drive.setSlowModeCommand(false).schedule());

        gp1.leftTrigger().atLeast(.5)
                .whenBecomesTrue(() -> Drive.setHeadingLockCommand(true).schedule())
                .whenBecomesFalse(() -> Drive.setHeadingLockCommand(false).schedule());
    }
    @Override public void onUpdate() {
        for (String cname : CommandManager.INSTANCE.snapshot()) {
            Logger.add("Commands", cname);
        }
        Logger.update();
    }

    @Override public void onStop() {
    }
}

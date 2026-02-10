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



@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends NextFTCOpMode {



    {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                AllianceManager.INSTANCE,
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

        GamepadEx gamepad1 = Gamepads.gamepad1();
        GamepadEx gamepad2 = Gamepads.gamepad2();

        gamepad2.rightBumper()
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

        gamepad1.leftBumper()
                .whenBecomesTrue(() -> Intake.reverse().schedule())
                .whenBecomesFalse(() -> Intake.off().schedule());

        // TODO: make toggle
        gamepad1.y()
                .whenBecomesTrue(() -> {

                    Drive.toggleSlowModeCommand();

                });

        gamepad2.x()
                .whenBecomesTrue(() -> Transitions.on().schedule())
                .whenBecomesFalse(() -> Transitions.off().schedule());

        gamepad1.rightBumper()
                .whenBecomesTrue(() -> Intake.on().schedule())
                .whenBecomesFalse(() -> Intake.off().schedule());

        gamepad1.dpadDown()
                .whenBecomesTrue(() -> Storage.spinToNextIntakeIndex().schedule());

        gamepad1.dpadUp()
                .whenBecomesTrue(() -> Storage.spinToNextOuttakeIndex().schedule());

        gamepad1.dpadRight()
                .whenBecomesTrue(() -> Robot.outtakeAll.schedule());

        gamepad1.dpadLeft()
                .whenBecomesTrue(() -> Robot.outtakeOne.schedule());

        gamepad1.a()
                .whenBecomesTrue(() -> Limelight.toggleAimAssist());

        gamepad2.dpadUp()
                .whenBecomesTrue(() -> Intake.on().schedule())
                .whenBecomesFalse(() -> Intake.off().schedule());

        gamepad2.dpadDown()
                .whenBecomesTrue(() -> Intake.reverse().schedule())
                .whenBecomesFalse(() -> Intake.off().schedule());


        gamepad2.leftBumper()
                .whenBecomesTrue(() -> Transitions.on().schedule())
                .whenBecomesFalse(() -> Transitions.off().schedule());

        gamepad2.a()
                .whenBecomesTrue(() -> {
                    Storage.setManualModeCommand(true).schedule();
                    Storage.setManualPowerCommand(0.75).schedule();
                })
                .whenBecomesFalse(() -> {
                    Storage.setManualModeCommand(true).schedule();
                    Storage.setManualPowerCommand(0).schedule();
                });
        gamepad2.b()
                .whenBecomesTrue(() -> {
                    Storage.setManualModeCommand(true).schedule();
                    Storage.setManualPowerCommand(0.5).schedule();
                })
                .whenBecomesFalse(() -> {
                    Storage.setManualModeCommand(true).schedule();
                    Storage.setManualPowerCommand(0).schedule();
                });
        gamepad2.y()
                .whenBecomesTrue(() -> {
                    Storage.setManualModeCommand(true).schedule();
                    Storage.setManualPowerCommand(0.3).schedule();
                })
                .whenBecomesFalse(() -> {
                    Storage.setManualModeCommand(true).schedule();
                    Storage.setManualPowerCommand(0).schedule();
                });

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

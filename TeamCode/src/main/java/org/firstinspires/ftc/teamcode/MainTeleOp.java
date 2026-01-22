package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Storage;
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
                        Storage.INSTANCE
                )
        );
    }
    @Override public void onWaitForStart() {
        ActiveOpMode.telemetry().update();
    }

    @Override public void onStartButtonPressed() {

        GamepadEx gamepad2 = Gamepads.gamepad2();

        gamepad2.a()
                .whenBecomesTrue(() -> Storage.setManualPowerCommand(0.075).schedule())
                .whenBecomesFalse(() -> Storage.setManualPowerCommand(0).schedule());
        gamepad2.b()
                .whenBecomesTrue(() -> Storage.setManualPowerCommand(0.5).schedule())
                .whenBecomesFalse(() -> Storage.setManualPowerCommand(0).schedule());
        gamepad2.y()
                .whenBecomesTrue(() -> Storage.setManualPowerCommand(0.2).schedule())
                .whenBecomesFalse(() -> Storage.setManualPowerCommand(0).schedule());
        gamepad2.back()
                .whenBecomesTrue(() -> Storage.resetAveragingCommand().schedule());
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

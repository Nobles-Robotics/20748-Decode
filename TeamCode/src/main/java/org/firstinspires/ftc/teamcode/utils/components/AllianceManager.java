package org.firstinspires.ftc.teamcode.utils.components;
import org.firstinspires.ftc.teamcode.autos.ANewWorkingAuto;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.Location;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class AllianceManager implements Component {

    public static final AllianceManager INSTANCE = new AllianceManager();
    public static Alliance currentAlliance = Alliance.BLUE;
    public static Location currentLocation = Location.FAR;
    public static boolean currentRunIntake1 = ANewWorkingAuto.runIntake1;
    public static boolean currentRunIntake2 = ANewWorkingAuto.runIntake2;

    public static boolean currentRunIntake3 = ANewWorkingAuto.runIntake3;

    private static boolean lastState;
    private static boolean lastStateY;

    private static boolean lastDPadLeft;
    private static boolean lastDPadUp;
    private static boolean lastDPadRight;


    @Override public void preWaitForStart() {
        ActiveOpMode.telemetry().addData("Alliance", currentAlliance);
        ActiveOpMode.telemetry().addLine("Press 'X' to toggle current alliance");
        if (ActiveOpMode.gamepad1().x && !lastState) {
            if (currentAlliance == Alliance.BLUE) {
                currentAlliance = Alliance.RED;
            } else {
                currentAlliance = Alliance.BLUE;
            }
        }
        lastState = ActiveOpMode.gamepad1().x;

        ActiveOpMode.telemetry().addData("Close/Far", currentLocation);
        ActiveOpMode.telemetry().addLine("Press 'Y' to toggle current location");
        if (ActiveOpMode.gamepad1().y && !lastStateY) {
            if (currentLocation == Location.FAR) {
                currentLocation = Location.CLOSE;
            } else {
                currentLocation = Location.FAR;
            }
        }
        lastStateY = ActiveOpMode.gamepad1().y;


        // 1 is closest to goal
        // 2 is middle
        // 3 is depot

        ActiveOpMode.telemetry().addLine("1 is closest to the goal; 2 is middle; 3 is closest to HP");
        ActiveOpMode.telemetry().addLine("runIntake1" + currentRunIntake1 + "press d-pad left to toggle");
        if (ActiveOpMode.gamepad1().dpad_left && !lastDPadLeft) {
            currentRunIntake1 = !currentRunIntake1;
        }
        lastDPadLeft = ActiveOpMode.gamepad1().dpad_left;

        ActiveOpMode.telemetry().addLine("runIntake2" + currentRunIntake2 + "press d-pad up to toggle");
        if (ActiveOpMode.gamepad1().dpad_up && !lastDPadUp) {
            currentRunIntake2 = !currentRunIntake2;
        }
        lastDPadUp = ActiveOpMode.gamepad1().dpad_up;

        ActiveOpMode.telemetry().addLine("runIntake3" + currentRunIntake3 + "press d-pad right to toggle");
        if (ActiveOpMode.gamepad1().dpad_right && !lastDPadRight) {
            currentRunIntake3 = !currentRunIntake3;
        }
        lastDPadRight = ActiveOpMode.gamepad1().dpad_right;

    }

    public static Alliance getCurrentAlliance(){
        return currentAlliance;
    }
    public static void setCurrentAlliance(Alliance newAlliance){ currentAlliance = newAlliance; }
}

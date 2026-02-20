package org.firstinspires.ftc.teamcode.utils.components;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.Location;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class AllianceManager implements Component {

    public static final AllianceManager INSTANCE = new AllianceManager();
    public static Alliance currentAlliance = Alliance.BLUE;
    public static Location currentLocation = Location.FAR;

    private static boolean lastState;
    private static boolean lastStateY;


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
    }

    public static Alliance getCurrentAlliance(){
        return currentAlliance;
    }
    public static void setCurrentAlliance(Alliance newAlliance){ currentAlliance = newAlliance; }
}

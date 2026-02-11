package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.ftc.ActiveOpMode;



import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import dev.nextftc.core.subsystems.SubsystemGroup;

import org.firstinspires.ftc.teamcode.utils.Logger;

import java.util.List;

public class Limelight extends SubsystemGroup {
    // put hardware, commands, etc here

    public static final Limelight INSTANCE = new Limelight();
    private Limelight(){
        super(Drive.INSTANCE);
    }
    private Limelight3A limelight;
    static boolean assistOn = false;
    private static boolean requestReadLimelight = true;

    public static void setRequestReadLimelight(boolean toRequest){
        requestReadLimelight = toRequest;
    }

    public static void setAimAssist(boolean bool){
        assistOn = bool;
    }

    public static void toggleAimAssist(){assistOn = !assistOn;}

    @Override
    public void initialize() {
        // initialization logic (runs on init)
        
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        //telemetry.setMsTransmissionInterval(110);

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();
        Drive.setAimAssist(0.0);
        if (result != null) {
            if (result.isValid()) {
//                telemetry.addData("tx", result.getTx()); // How far left or right the target is (degrees)
//                telemetry.addData("ty", result.getTy()); // How far up or down the target is (degrees)
//                telemetry.addData("ta", result.getTa()); // How big the target looks (0%-100% of the image)
//                telemetry.addData("         Separate", "I just los my daww");
                Logger.add("Limelight", Logger.Level.INFO, "tx: " + result.getTx());
                Logger.add("Limelight", Logger.Level.INFO, "ty: " + result.getTy());
                Logger.add("Limelight", Logger.Level.INFO, "ta: " + result.getTa());


                List<LLResultTypes.FiducialResult> fidResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fidResults) {
                    //telemetry.addData("APRIL ID ", fr.getFiducialId());
                    //telemetry.addData("Pattern ", getPattern(fr.getFiducialId()));
                    //telemetry.addData("----", "---");
                    //telemetry.addData("APRIL TARGET POSE (CAMERA SPACE) ", fr.getTargetPoseCameraSpace());

                    Logger.add("Limelight", Logger.Level.INFO, "APRIL ID: " + fr.getFiducialId() );
                    Logger.add("Limelight", Logger.Level.INFO, "Pattern " + getPattern(fr.getFiducialId())  );
                    Logger.add("Limelight", Logger.Level.INFO, "APRIL TARGET POSE (CAMERA SPACE): " + fr.getTargetPoseCameraSpace());

                    if (fr.getFiducialId() == 20 && assistOn){
                        Drive.setAimAssist(fr.getTargetPoseCameraSpace().getPosition().x);
                    }

                }
                //telemetry.update();
            }
        }

    }

    public String getPattern(int id) {
        String p1 = "P";
        String p2 = "P";
        String p3 = "P";

        if (id == 21){
            p1 = "G";
        }else if (id == 22){
            p2 = "G";
        }else if (id == 23){
            p3 = "G";
        }

        return p1 + " " + p2 + " " + p3;
    }
}

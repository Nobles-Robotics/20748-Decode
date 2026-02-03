package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name="LimeLightTest", group="TeleOp")
public class LimelightTest extends LinearOpMode {
    private Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException
    {


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(110);

        limelight.pipelineSwitch(0);
        /*
         * Starts polling for data.
         */
        limelight.start();
        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();

                    telemetry.addData("tx", result.getTx()); // How far left or right the target is (degrees)
                    telemetry.addData("ty", result.getTy()); // How far up or down the target is (degrees)
                    telemetry.addData("ta", result.getTa()); // How big the target looks (0%-100% of the image)

                    telemetry.addData("TagCount", result.getBotposeTagCount());
                    telemetry.addData("Barcode results", result.getBarcodeResults());

                    telemetry.addData("Detec results", result.getDetectorResults());
                    telemetry.addData("Separate","I just los my daww");
                    //telemetry.addData("Fiduc results", result.getFiducialResults());
                    List<LLResultTypes.FiducialResult> fidResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fidResults){
                        telemetry.addData("APRIL ID ", fr.getFiducialId());
                        telemetry.addData("APRIL SKEW ", fr.getSkew());
                        telemetry.addData("APRIL FAMILY ", fr.getFamily());
                        telemetry.addData("----", "---");
                    }


                    telemetry.update();
                }
            }
        }
    }
}

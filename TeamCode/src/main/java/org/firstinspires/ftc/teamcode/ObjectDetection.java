package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;

public class ObjectDetection extends  Autonomous_Base {

    WebcamName camera = hardwareMap.get(WebcamName.class, "Wecam 1");

    TfodProcessor tfodProcessor = TfodProcessor.easyCreateWithDefaults();

    VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(camera, tfodProcessor);

    @Override
    public void runOpMode() {
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                List<Recognition> recognitions = tfodProcessor.getRecognitions();

                for (Recognition recognition : recognitions) {
                    String label = recognition.getLabel();

                    float confidence = recognition.getConfidence();

                    telemetry.addLine("Recognized Object: " + label);
                    telemetry.addLine("Confidence: " + confidence);
                }
            }
        }
    }
}

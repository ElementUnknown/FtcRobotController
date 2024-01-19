/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;


@Autonomous(name="BlueRight", group="Robot")

public class BlueRight extends Autonomous_Base {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final String TFOD_MODEL_ASSET = "balls.tflite";
    private static final String[] LABELS = {
            "BlueBall",
            "RedBall",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public void runOpMode() {

        initTfod();
        super.robot.init(super.hardwareMap);
        super.robot.AprilInit(super.hardwareMap);
        if (super.robot.USE_WEBCAM)
            setManualExposure(6, 250);
        int Spike;
        waitForStart();
        Move(.8, -23, 0);
        //Check if found in center
        telemetryTfod();
        telemetry.update();
        if (checkDistance(10)) {
            PivotTick(4300, 1);
            Move(.8, 7, 0);
            PivotWaitFinish();
            Move(.3, -5, 0);
            PivotTick(200, 1);
            Move(.8, 11, 0);
            sleep(500);
            TurnByGyro(-90, -.8, 2);
            Move(.8,0,7.5);
            // Move(.8,0,3);
            PivotWaitFinish();
            //Located facing board
            Spike = 2;
        } else {
            //if not in the center check the left
            Move(.8, 2.5, -12);
            if (checkDistance(10)) {
                //if in left
                PivotTick(4300, 1);
                Move(.8, 10, -1.5);
                PivotWaitFinish();
                //Move(.3,0,0);
                //May bring this back
                PivotTick(200, 1);
                TurnByGyro(-90, -.8, 2);
                Move(.8,-12,0);
                horizontalMove(850, -.6);
                PivotWaitFinish();
                //Move(.8, 0, -13.5);
                Move(.8,0,-6);
                //finish facing the board
                Spike = 3;
            } else {
                //if not in center or left move to right

                PivotTick(3900, 1);
                TurnByGyro(-85, -.8, 3);
                PivotTick(4300, 1);
                PivotWaitFinish();
                Move(.8, -6.5, -6);
                //Move(.3,-,0);
                PivotTick(100, 1);

                sleep(300);
                //Move(.6, 0, -30);
                Move(.7,0,12);
                Move(.8,-7,0);
                horizontalMove(850, -.7);
                Move(.8,0,-5);
                Spike = 1;
                //finish facing board
            }
        }
        Move(.9, -75, 0);
        PivotTick(3200,1);
        LocateTag(.6,-90,1,-20);
        AprilTagNav(.6,-90,Spike,7,0,.5,-1,6000);
        PivotWaitFinish();
        sleep(100);
        Move(.7, 4, 0); // Move back to allow the pixel to fall
        PivotTick(10, 1);//close arm to final position
        Move(.3, -3, 24);
        PivotWaitFinish();//final move to park
        //in all auto codes we should consider changing final position out of the way of the board, maybe to the middle
        visionPortal.close();
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }
}

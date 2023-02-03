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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/**
 * This file illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="LeftSideAuto", group="Robot")
public class LeftSideAuto extends Autonomous_Base {

    HardwareMap robot = new HardwareMap();
    private int color = 0;
    private int checkNum = 0;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    double fx = 1419.6926;
    double fy = 1419.6926;
    double cx = 624.4230;
    double cy = 317.1988;
    double tagsize = 0.042;
    int numFramesWithoutDetection = 0;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    int apriltagNum;

    double lastDistance;

    @Override
    public void runOpMode() {

        super.robot.init(super.hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.setMsTransmissionInterval(50);

        /** initial tests for lift encoders **/
        /*robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmL.setTargetPosition(500);
        robot.liftArmR.setTargetPosition(500);
        robot.liftArmL.setPower(1);
        robot.liftArmR.setPower(1);
        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.liftArmL.isBusy() && robot.liftArmR.isBusy()){
        }*/
        /** More encoder tests and setup**/
        /*closeClaw();
        Highgoal();
        Move(.5,3,0);
        waitforarmfinish();
        Move(.5,3,0);
        Goalreadjust();
        waitforarmfinish();
        releaseClaw();*/
        super.robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        super.robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        super.robot.liftArmL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        super.robot.liftArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        closeClaw();

        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        if(detections != null)
        {
            // If we don't see any tags
            if(detections.size() == 0)
            {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else
            {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                }

                for(AprilTagDetection detection : detections)
                {
                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    apriltagNum = detection.id;
                }
            }
            telemetry.update();
        }

        Lowgoal(0,0);
        odsStop(.4);
        lastDistance = super.robot.ods.getDistance(DistanceUnit.INCH);

        /*Move(.4,0,18);
        sleep(500);
        while (color != 6 && color != 9 && color != 10 && checkNum < 5) {
            color = super.robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
            checkNum++;
            sleep(250);
        }
        telemetry.addData("Color Number", color);
        telemetry.update();
        Move(.5,0,21);*/

        waitforarmfinish();
        Move(.3,lastDistance,0);
        sleep(500);
        Goalreadjust();
        waitforarmfinish();
        releaseClaw();
        Move(.4,-2,0);
        Grabconeheight();
        Move(.3,0,14);
        waitforarmfinish();
        Move(.3,0,-2);
        Move(.3,23.5,0);
        closeClaw();
        sleep(50);
        MoveArm(400, 1);
        Move(.5,-48.5,0);
        Move(.3,0,-12.5);
        Medgoal(0,0);
        waitforarmfinish();
        Move(.3,2.5,0);
        Lowgoal(0,0);
        waitforarmfinish();
        releaseClaw();
        Move(.3,-4,0);

        /*telemetry.addData("Color Number", color);
        telemetry.update();
        if (color == 9 || color == 8) {
            Move(.3,0,12.5);
            Move(.3,25,0);
            ArmGround(0,0);
            waitforarmfinish();
        }
        else if (color == 6) {
            ArmGround(0,0);
            waitforarmfinish();
        }
        else {
            Move(.3,0,12);
            Move(.3, 48,0);
            ArmGround(0,0);
            waitforarmfinish();
        }*/

        if (apriltagNum == 1) {
            Move(.3,0,12.5);
            Move(.3,25,0);
            ArmGround(0,0);
            waitforarmfinish();
        }
        else if (apriltagNum == 2) {
            ArmGround(0,0);
            waitforarmfinish();
        }
        else if (apriltagNum == 3) {
            Move(.3,0,12);
            Move(.3, 48,0);
            ArmGround(0,0);
            waitforarmfinish();
        }
    }
}

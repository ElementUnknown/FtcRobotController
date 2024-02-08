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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@Autonomous(name="AutoTests", group="Robot")

public class AutoTests extends Autonomous_Base {

    public void runOpMode() {

        super.robot.init(super.hardwareMap);
        super.robot.AprilInit(super.hardwareMap);

        int Spike =1;
        boolean targetFound     = false;
        if (super.robot.USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        waitForStart();
        /*Move(.5,-23,0);
        //Check if found in center
        if(checkDistance(10)){
            PivotTick(4300, .75);
            Move(.5,7,0);
            PivotWaitFinish();
            Move(.3,-5,0);
            releaseClawL();
            PivotTick(200, .75);
            Move(.5,22.5,0);
            TurnByGyro(85,.5,2);
            PivotWaitFinish();
            //Located facing board
            Spike = 1;
        }
        else {
            //if not in the center check the left
            Move(.5,3.5,12);
            if(checkDistance(10)){
                //if in left
                PivotTick(4300, .75);
                Move(.5,10,-1.5);
                PivotWaitFinish();
                //Move(.3,0,0);
                //May bring this back
                releaseClawL();
                PivotTick(3500, 1);
                TurnByGyro(90,.5,2);
                PivotWaitFinish();
                Move(.5,0,-3);
                //finish facing the board
                Spike = 0;
            }
            else {
                //if not in center or left move to right

                PivotTick(3900, .75);
                TurnByGyro(90,.5,3);
                PivotTick(  4300, .75);
                PivotWaitFinish();
                Move(.6,-4.5,14);
                //Move(.3,-,0);
                releaseClawL();
                PivotTick(100,1);
                Move(.6,0,-30);

                Spike = 2;
                //finish facing board
            }
        }
        Move(.7,-87,0);
        sleep(5000);
        Move(.8,0,-10);
        Move(.8,0,28);
        closeClawL(); //Close left to conserve space on the board
        switch(Spike){
            case(0):
                Move(.5,0,4);
                //Move left and amount
                break;
            case(1):
                Move(.7,-16,0);
                break;
            case(2):
                Move(.5,0,-6);
                //Move right an amount
                break;
        }
        Move(.5,-2.5,0);
        sleep(500);//move to board
        releaseClawR();
        sleep(100);
        Move(.7,4,0); // Move back to allow the pixel to fall
        PivotTick(0,1);//close arm to final position
        Move(.3,-3,0);
        PivotWaitFinish();//final move to park
        //in all auto codes we should consider changing final position out of the way of the board, maybe to the middle */

        //April Tag Tests Navigation Tests
        /*super.robot.DESIRED_TAG_ID = 5;
        Nav: while (opModeIsActive()) {
            targetFound = false;
            super.robot.desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = super.robot.aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                        ((super.robot.DESIRED_TAG_ID < 0) || (detection.id == super.robot.DESIRED_TAG_ID))) {
                    targetFound = true;
                    super.robot.desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            Found: if (targetFound) {
                telemetry.addData("", "TargetFound");
                telemetry.addData("Target", "ID %d (%s)", super.robot.desiredTag.id, super.robot.desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", super.robot.desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", super.robot.desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", super.robot.desiredTag.ftcPose.yaw);
                double Range = super.robot.desiredTag.ftcPose.range;
                double Bearing =  super.robot.desiredTag.ftcPose.bearing;
                double Rx = super.robot.desiredTag.ftcPose.x;
                double Ry = super.robot.desiredTag.ftcPose.y;
                if(Ry<5) Ry = 0;
                super.robot.Motor1.setPower( .4*((Ry - Rx)/(Math.abs(Ry) + Math.abs(Rx))) - (Bearing/40));
                super.robot.Motor2.setPower(.4*((Ry + Rx)/(Math.abs(Ry) + Math.abs(Rx))) +(Bearing/40));
                super.robot.Motor3.setPower(.4*((Ry + Rx)/(Math.abs(Ry) + Math.abs(Rx))) - (Bearing/40));
                super.robot.Motor4.setPower(.4*((Ry - Rx)/(Math.abs(Ry) + Math.abs(Rx))) + (Bearing/40));
                if(Ry ==0 && Math.abs(Rx) < 1 && Math.abs(Bearing) < 2){

                    super.robot.Motor1.setPower(0);
                    super.robot.Motor2.setPower(0);
                    super.robot.Motor3.setPower(0);
                    super.robot.Motor4.setPower(0);
                    break Nav;
                }

            } else {
                telemetry.addData(">", "No Target Found\tplease move me");
                super.robot.Motor1.setPower(0);
                super.robot.Motor2.setPower(0);
                super.robot.Motor3.setPower(0);
                super.robot.Motor4.setPower(0);
            }
           telemetry.update();
        }
        telemetry.addData("Target Was Found", "");
        telemetry.update();
        sleep(5000);*/
        //TurnByGyro(90,.7,2);
        //AprilTagNav(.3,0,8,12,0,.5,-1, 7000);
        /*super.robot.liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        super.robot.liftArm.setTargetPosition(-500);
        super.robot.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        super.robot.liftArm.setPower(-.7);
        while(super.robot.liftArm.isBusy()){
            telemetry.addData("Ticks", super.robot.liftArm.getCurrentPosition());
            telemetry.update();
        }
        super.robot.liftArm.setPower(0);*/
        //Move(.6,50,0);
        //Move(.6,-50,0);
        //AprilTagNav(.8,0,5,13,0,.5,-1,10000);
        //Move(1,-12,0);
        //Move(1,0,-30);
        AprilTagNav(.6,getHeading(),5,12,0,1.5,1,20000);
        //PivotTick(500,1);

    }
}

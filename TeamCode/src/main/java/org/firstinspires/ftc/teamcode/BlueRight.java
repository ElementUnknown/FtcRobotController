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
import org.firstinspires.ftc.teamcode.opencv.Alliance;
import org.firstinspires.ftc.teamcode.opencv.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;


@Autonomous(name="BlueRight", group="Robot")

public class BlueRight extends Autonomous_Base {




    public void runOpMode() {

        super.robot.init(super.hardwareMap);
        //super.robot.initTfod(super.hardwareMap);
        super.robot.AprilInit(super.hardwareMap, Alliance.BLUE_RIGHT);
        setManualExposure(6, 250);
        int Spike;
        int Case = robot.getSpike();
        boolean found = false;

        waitForStart();

        //MAson do what ever detection Code for the thing here as we start
        //Move(1, -23, 0);
        //Check if found in center
       /* Move(1.25,-16,0);
        List<Recognition> currentRecognitions;
        for (int i = 0; i < 10; i++) {
            currentRecognitions = super.robot.tfod.getRecognitions();
            if(!currentRecognitions.isEmpty()){
                Case = 2;
                found= true;
                break;
            }
            else sleep(75);
        }
        if(!found) {
            Move(1, 0, 14);
            for (int i = 0; i < 10; i++) {
                currentRecognitions = super.robot.tfod.getRecognitions();
                if (!currentRecognitions.isEmpty()) {
                    Case = 1;
                    found = true;
                    break;
                } else sleep(75);
            }
            if (!found) {
                Case = 3;
            }
        }*/
        double Rxb;
        telemetry.addData("case", Case);
        telemetry.addData("was found", found);
        telemetry.update();
        //sleep(3000);

        if(Case == 2){
            PivotTick(750,.3);
            Move(.7, -14, 3);
            PivotWaitFinish();
            PivotTick(925,.2);
            Move(.5,-6,0);
            //Move(1,0,-12);
            openClawR();
            super.robot.elbow.setPosition(.8);
            sleep(100);
            PivotTick(300,.5);
            Move(.7,3.5,-14);
            Move(.7,-33,0);
            PivotTick(920,.2);
            TurnByGyro(-96, -.8, 2);
            Move(1,-96,0);
            //finish facing the board
            Spike = 2;
            Rxb = -.5;
        }
        else if(Case == 3){
            PivotTick(750,.3);
            Move(.8,-7,-6.5);
            PivotWaitFinish();
            PivotTick(925,.2);
            Move(.8,-5,0);
            openClawR();
            super.robot.elbow.setPosition(.8);
            Move(1,0,11.5);
            PivotTick(300,.5);
            Move(1,-36,0);
            PivotTick(920,.2);
            TurnByGyro(-95,-.8,2);
            Move(1,-78,0);
            Spike =1;
            Rxb = -1.5;
        }
        else {
            //if not in center or left move to right
            PivotTick(750,.3);
            Move(1,-26,-8);
            PivotWaitFinish();
            TurnByGyro(-93, -.8, 2);
            PivotTick(925,.2);
            Move(1, -4, 0);
            openClawR();
            super.robot.elbow.setPosition(.8);
            Move(1,7,0);
            Move(1, 0, -27);
            TurnByGyro(-6,-.8,3);
            //may need to correct with a turn
            Move(1,-98,0);
            Spike = 3;
            Rxb = -.5;
            //finish facing board
        }
        PivotTick(700,1);
        super.robot.elbow.setPosition(.7);
        if(Case == 3){
        Move(1,0,18);
        }
        else {
            Move(1,0,24);
        }
        int Nav = AprilTagNav(.6,getHeading(),Spike,13,Rxb,1,-1,5000);
        if(Nav >45){
            sleep(100);
            Move(.7, -5, 0);
            sleep(200);
            openClawL();
            Move(1,7,0);// Move back to allow the pixel to fall
            PivotTick(75, .2);//close arm to final position
            moveLift(0,1);
            Move(.8, -10, -18);

            PivotWaitFinish();//final move to park
        }
        else if (Nav > 0){
            openClawL();
            PivotTick(75,.2);
            Move(.3,-6,7);
            PivotWaitFinish();
        }
        else{
            openClawL();
            PivotTick(75,.2);
            Move(.4,-6,30);

        }
        //in all auto codes we should consider changing final position out of the way of the board, maybe to the middle
        super.robot.visionPortal.close();
    }
}

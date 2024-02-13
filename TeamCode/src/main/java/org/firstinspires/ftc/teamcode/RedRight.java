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


@Autonomous(name="RedRight", group="Robot")

public class RedRight extends Autonomous_Base {

    public void runOpMode() {

        super.robot.init(super.hardwareMap);
        //super.robot.initTfod(super.hardwareMap);
        super.robot.AprilInit(super.hardwareMap);
        setManualExposure(6, 250);
        int Spike;
        int Case = 0;
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

        telemetry.addData("case", Case);
        telemetry.addData("was found", found);
        telemetry.update();
        //sleep(3000);
        Case =3;
        if(Case == 2){
            PivotTick(750,.3);
            Move(.7, -14, 0);
            PivotWaitFinish();
            PivotTick(925,.2);
            Move(.5,-7,0);
            //Move(1,0,-12);
            openClawL();
            sleep(100);
            Move(1,4,-20);
            TurnByGyro(85, .8, 2);
            Move(1,-5,0);
            //finish facing the board
            Spike = 5;
        }
        else if(Case == 3){
            PivotTick(750,.3);
            Move(.8,-3,-10.5);
            PivotWaitFinish();
            PivotTick(925,.2);
            Move(.8,-9,0);
            openClawL();
            Move(1,0,-11.5);
            TurnByGyro(85,.7,1);
            Move(1,-4,0);
            Spike =4;
        }
        else {
            //if not in center or left move to right
            Move(1,-26,-12);
            PivotTick(750,.3);
            PivotWaitFinish();
            TurnByGyro(85, .8, 2);
            PivotTick(925,.2);
            Move(1, -4, 0);
            openClawL();
            Move(1,12,0);
            TurnByGyro(185, .8, 2);
            Move(1,-12,0);
            Spike = 6;
            //finish facing board
        }
        PivotTick(600,1);
        super.robot.elbow.setPosition(.3);
        if(AprilTagNav(.6,getHeading(),Spike,13,-.5,1,1,6000) > 45){
            sleep(100);
            //Move(.7, -2, 0);
            sleep(200);
            super.robot.elbow.setPosition(.3);
            Move(1,7,0);// Move back to allow the pixel to fall
            //PivotTick(10, 1);//close arm to final position
            moveLift(0,1);
            Move(.8, -10, 24);

            PivotWaitFinish();//final move to park
        }
        else{
            super.robot.elbow.setPosition(.3);
            PivotWaitFinish();
        }
        //in all auto codes we should consider changing final position out of the way of the board, maybe to the middle
        super.robot.visionPortal.close();
    }
}

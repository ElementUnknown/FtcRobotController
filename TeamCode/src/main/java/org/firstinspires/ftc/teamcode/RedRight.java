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

import org.firstinspires.ftc.teamcode.opencv.Alliance;


@Autonomous(name="RedRight", group="Robot")

public class RedRight extends Autonomous_Base {

    public void runOpMode() {

        super.robot.init(super.hardwareMap);
        //super.robot.initTfod(super.hardwareMap);
        super.robot.AprilInit(super.hardwareMap, Alliance.RED_RIGHT);
        setManualExposure(6, 250);
        int Spike;
        int Case;
        double Rxb;
        boolean found = false;
        waitForStart();
        Case = super.robot.getSpike();
        telemetry.addData("case", Case);
        telemetry.addData("was found", found);
        telemetry.update();
        sleep(1000);

        if(Case == 2){
            PivotTick(750,.3);
            Move(.7, -13, -3);
            PivotWaitFinish();
            PivotTick(925,.2);
            Move(.5,-7,0);
            //Move(1,0,-12);
            openClawL();
            PivotTick(500,.4);
            sleep(100);
            TurnByGyro(85,.7,2);
            Move(1,-20,0);
            //finish facing the board
            Spike = 5;
            Rxb = -.5;
        }
        else if(Case == 3){
            PivotTick(750,.3);
            Move(.8,-5,-6);
            PivotWaitFinish();
            PivotTick(915,.2);
            Move(.6,-6,0);
            openClawL();
            PivotTick(500,.4);
            //Move(1,0,-11.5);
            TurnByGyro(85,.7,1);
            Move(1,-20,0);
            Move(.7,0,7);
            Spike =6;
            Rxb = -1.5;
        }
        else {
            //if not in center or left move to right
            PivotTick(750,.3);
            Move(1,-26,-8);
            PivotWaitFinish();
            TurnByGyro(-85, -.8, 2);
            PivotTick(915,.2);
            Move(1, -4, 0);
            openClawL();
            PivotTick(825,.2);
            Move(1,12,0);
            TurnByGyro(180,.8,3);
            //may need to correct with a turn
            Move(1,-20,0);
            Spike = 4;
            Rxb = -.5;
            //finish facing board
        }
        PivotTick(730,1);
        super.robot.elbow.setPosition(.6);
        int Nav = AprilTagNav(.7,getHeading(),Spike,11,Rxb,1,1,4000);
        if(Nav >45){
            sleep(100);
            Move(.7, -5, 0);
            sleep(100);
            openClawR();
            Move(1,7,0);// Move back to allow the pixel to fall
            PivotTick(75, .2);//close arm to final position
            moveLift(0,1);
            Move(.8, -10, -24);

            PivotWaitFinish();//final move to park
        }
        else if (Nav > 0){
            openClawR();
            PivotTick(75,.2);
            Move(.3,-6,-7);
            PivotWaitFinish();
        }
        else{
            openClawL();
            PivotTick(72,.2);
            Move(.4,-6,-30);

        }
        super.robot.visionPortal.close();
    }
}

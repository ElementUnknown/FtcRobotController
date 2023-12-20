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
        super.robot.AprilInit(super.hardwareMap);
        int Spike;
        if (super.robot.USE_WEBCAM)
            setManualExposure(6, 250);
        waitForStart();
        Move(.7,-23,0);
        //Check if found in center
        if(checkDistance(10)){
            PivotTick(4300, 1);
            Move(.5,7,0);
            PivotWaitFinish();
            Move(.3,-5,0);
            //TurnByGyro(180,.7,3);
            //Move(.5,3,0);
            //dropPixel();
            releaseClawL();
            PivotTick(3200, 1);
            sleep(200);
           // Move(.5,15,0);
            TurnByGyro(85,.7,2);
            Move(.8,-12,0);
            PivotWaitFinish();
            //Located facing board
            Spike = 5;
        }
        else {
            //if not in the center check the left
            Move(.5,4,-12);
            if(checkDistance(10)){
                //if in left
                PivotTick(4300, 1);
                Move(.5,9,-1);
                PivotWaitFinish();
                //Move(.3,0,0);
                //May bring this back
                releaseClawL();
                PivotTick(3200, 1);
                sleep(500);
                TurnByGyro(90,.7,2);
                Move(.5,-9,0);
                PivotWaitFinish();
                //finish facing the board
                Spike = 6;
            }
            else {
                //if not in center or left move to right

                PivotTick(3900, 1);
                TurnByGyro(-90,-.7,3);
                PivotTick(  4300, 1);
                PivotWaitFinish();
                Move(.4,-6.5,-10);
                //PivotWaitFinish();
                //Move(.3,-,0);
                releaseClawL();
                Move(.7,17,0);
                PivotTick(3200,1);
                //Move(.7,0,20);

                TurnByGyro(180,.7,2);
                Spike = 4;
                //finish facing board
            }
        }
        //Move(.7,-16,0);
        //Move(.9,0,-10);
       // Move(.8,0,28);
        closeClawL(); //Close left to conserve space on the board
        /*switch(Spike){
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
        sleep(500);//move to board*/
        LocateTag(.6,90,-1,-12);
        AprilTagNav(.6,90,Spike,7,0,1,0,5000);
        if(Spike == 6){
            Move(.5,0,3);
           // Move(.5,-2,0);
        }
        PivotWaitFinish();
        releaseClawR();
        sleep(400);
        Move(.7,4,0); // Move back to allow the pixel to fall
        PivotTick(20,1);//close arm to final position
        Move(.6,-3,-29);
        Move(.7,-6,0);
        PivotWaitFinish();//final move to park
        //in all auto codes we should consider changing final position out of the way of the board, maybe to the middle
    }
}

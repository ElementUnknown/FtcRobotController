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


@Autonomous(name="RedLeft", group="Robot")

public class RedLeft extends Autonomous_Base {

    public void runOpMode() {

        super.robot.init(super.hardwareMap);
        super.robot.AprilInit(super.hardwareMap);
        if (super.robot.USE_WEBCAM)
            setManualExposure(6, 250);
        int Spike;
        waitForStart();
        Move(.8, -23, 0);
        //Check if found in center
        if (checkDistance(10)) {
            PivotTick(4300, 1);
            Move(.8, 7, 0);
            PivotWaitFinish();
            Move(.3, -5, 0);
            releaseClawL();
            PivotTick(200, 1);
            Move(.8, 11, 0);
            sleep(500);
            TurnByGyro(85, .8, 2);
            Move(.8,0,-7.5);
           // Move(.8,0,3);
            PivotWaitFinish();
            //Located facing board
            Spike = 5;
        } else {
            //if not in the center check the left
            Move(.8, 2.5, 12);
            if (checkDistance(10)) {
                //if in left
                PivotTick(4300, 1);
                Move(.8, 10, -1.5);
                PivotWaitFinish();
                //Move(.3,0,0);
                //May bring this back
                releaseClawL();
                PivotTick(200, 1);
                TurnByGyro(90, .8, 2);
                Move(.8,-12,0);
                horizontalMove(850, .6);
                PivotWaitFinish();
                //Move(.8, 0, -13.5);
                Move(.8,0,6);
                //finish facing the board
                Spike = 6;
            } else {
                //if not in center or left move to right

                PivotTick(3900, 1);
                TurnByGyro(85, .8, 3);
                PivotTick(4300, 1);
                PivotWaitFinish();
                Move(.8, -4.75, 6.5);
                //Move(.3,-,0);
                releaseClawL();
                PivotTick(100, 1);

                    
                //Move(.6, 0, -30);
                Move(.8,0,-12);
                Move(.8,-7,0);
                horizontalMove(850, .7);
                Move(.8,0,5);
                Spike = 4;
                //finish facing board
            }
        }
        Move(.9, -75, 0);
        PivotTick(3200,1);
        closeClawL(); //Close left to conserve space on the board
        LocateTag(.6,90,-1,-20);
        AprilTagNav(.6,90,Spike,7,0,.5,1,6000);
        releaseClawR();
        sleep(100);
        Move(.7, 4, 0); // Move back to allow the pixel to fall
        PivotTick(10, 1);//close arm to final position
        Move(.3, -3, -24);
        PivotWaitFinish();//final move to park
        //in all auto codes we should consider changing final position out of the way of the board, maybe to the middle
    }
}
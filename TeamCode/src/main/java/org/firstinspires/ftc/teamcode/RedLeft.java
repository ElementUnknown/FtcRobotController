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
        int Case =1;
        waitForStart();

        //MAson do what ever detection Code for the thing here as we start
        //Move(1, -23, 0);
        //Check if found in center
        if (Case == 1) {
            Move(1, -48, 0);
            dropPixel();
            Move(1.25, -7, 0);
            TurnByGyro(87,.8,1);
            Move(1,-75,0);
           // Move(.8,0,3);
            //Located facing board
            Spike = 5;
        }
        if(Case == 3){

            Move(1, -45, 6);
            dropPixel();
            Move(1,7,0);
            TurnByGyro(90, .8, 2);
            Move(1,-90,0);
            //finish facing the board
            Spike = 4;
        }
        else {
            //if not in center or left move to right
            Move(1,-27,0);
            TurnByGyro(85, .8, 1);
            Move(1, -29, 0);
            dropPixel();
            Move(1, 4, 0);
            Move(1,0,-30);
            Move(.8,-90,0);
            Spike = 6;
                //finish facing board
            }

        MovetoPlace();

        Move(1,0,-20);
        AprilTagNav(.6,getHeading(),Spike,7,0,.5,1,6000);
        sleep(100);
        Move(.7, 4, 0); // Move back to allow the pixel to fall
        PivotTick(10, 1);//close arm to final position
        Move(.3, -3, -24);
        PivotWaitFinish();//final move to park
        //in all auto codes we should consider changing final position out of the way of the board, maybe to the middle
    }
}
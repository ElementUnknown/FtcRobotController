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

        int Spike;
        waitForStart();
        Move(.5,-21,0);
        //Check if found in center
        if(checkDistance(10)){
            PivotTick(4350, .75);
            Move(.5,5,0);
            PivotWaitFinish();
            Move(.3,-5,0);
            releaseClawL();
            PivotTick(2500, .75);
            sleep(500);
            TurnByGyro(90,.5,4);
            PivotWaitFinish();
            //Located facing board
            Spike = 1;
        }
        else {
            //if not in the center check the left
            TurnByGyro(-45,.5,3);
            if(checkDistance(10)){
                //if in left
                PivotTick(4350, .75);
                Move(.5,5,0);
                PivotWaitFinish();
                Move(.3,-5,0);
                releaseClawL();
                PivotTick(2500, .75);
                sleep(200);
                TurnByGyro(135,.5,3);
                PivotWaitFinish();
                //finish facing the board
                Spike = 0;
            }
            else {
                //if not in center or left move to right
                PivotTick(3900, .75);
                TurnByGyro(90,.5,3);
                PivotTick(4500, .75);
                Move(.5,5,0);
                PivotWaitFinish();
                Move(.3,-5,0);
                releaseClawL();
                PivotTick(2500,.75);
                TurnByGyro(45,.5,3);
                Spike = 2;
                //finish facing board
            }
        }
        //assuming correct alignment to pass through gate
        Move(.7,-70,0); //based on 87 inch lateralmove, may vary with forward move
        //the goal is to be beyond the spike detection area of our partner yet not to close to the board
        //so that we may move horizontal adjustments accordingly
        closeClawL(); //Close left to conserve space on the board
        switch(Spike){
            case(0): //Spike == 0 should mean left
                Move(.5,0,-5);
                //Move left an amount
                break;
            case(1):
                //don't move?
                break;
            case(2):
                Move(.5,0,5);
                //Move right an amount
                break;
        }
        Move(.5,-17,0); //this move and move on l93 should add to 87
        releaseClawR();
        Move(.3,4,0); // Move back to allow the pixel to fall
        PivotTick(0,1);//close arm to final position
        Move(.3,-3,0); //final move to park
    }
}

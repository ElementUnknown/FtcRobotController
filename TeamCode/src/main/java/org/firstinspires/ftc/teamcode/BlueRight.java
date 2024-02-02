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




    public void runOpMode() {

        super.robot.init(super.hardwareMap);
        super.robot.AprilInit(super.hardwareMap);
        setManualExposure(6, 250);
        int Spike;
        int Case = 0;
        boolean found = false;

        waitForStart();
        Move(1.25, -16, 0);
       if(checkTFOD()){
           Case = 2;
           found = true;
       }
        if (!found) {
            Move(1, 0, -14);
            if(checkTFOD()){
                Case = 1;
                found = true;
            }
            if (!found) {
                Case = 3;
            }
        }

        telemetry.addData("case", Case);
        telemetry.addData("was found", found);
        telemetry.update();
        //sleep(3000);
        if (Case == 2) {
            Move(.7, 0, -12);
            Move(1, -33, 0);
            Move(1, 0, 12);
            dropPixel();
            Move(1, -5, 0);
            TurnByGyro(-93, -.8, 2);
            Move(1, -78, 0);
            //finish facing the board
            Spike = 2;
        } else if (Case == 1) {
            Move(1, 0, 15);
            Move(1, -32, 0);
            Move(1, 0, -12);
            dropPixel();
            Move(1, -5, 0);
            TurnByGyro(-97, -.8, 2);
            Move(1, -86, 0);
            Spike = 3;
        } else {
            //if not in center or left move to right
            Move(1, -10.5, 0);
            TurnByGyro(-100, -.8, 2);
            Move(1, -32.5, 0);
            dropPixel();
            Move(1.3,-2,0);
            Move(1, 0, -30);
            TurnByGyro(-15,-.8,3);
            //may need to correct with a turn
            Move(1, -60, 0);
            Spike = 1;
            //finish facing board
        }
        PivotTick(2000, 1);
        Move(1, 0, 25 + (3 * (2-Spike)));
        super.robot.elbow.setPosition(1);
        if (AprilTagNav(.6, getHeading(), Spike, 13, .5, 1, -1, 6000)) {
            sleep(100);
            //Move(.7, -2, 0);
            sleep(200);
            super.robot.elbow.setPosition(.3);
            Move(1, 7, 0);// Move back to allow the pixel to fall
            PivotTick(10, 1);//close arm to final position
            moveLift(0, 1);
            Move(.8, -10, -24);

            PivotWaitFinish();//final move to park
        } else {
            super.robot.elbow.setPosition(.3);
            PivotWaitFinish();
        }
        //in all auto codes we should consider changing final position out of the way of the board, maybe to the middle
        super.robot.visionPortal.close();
    }

}

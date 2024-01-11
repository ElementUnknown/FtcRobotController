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


@Autonomous(name="RedSideLeftODS", group="Robot")

public class RedSideLeftODS extends Autonomous_Base {

    boolean[] read = new boolean[4];

    public void runOpMode() {

        super.robot.init(super.hardwareMap);


        waitForStart();

        Move(.5, -15, 0);
        read[0] = checkDistance(15);
        sleep(200);
        read[1] = checkDistance(15);
        sleep(200);
        read[2] = checkDistance(15);
        sleep(200);
        read[3] = checkDistance(15);

        if (read[0] && read[1] && read[2] && read[3]) {
            MoveArm(500, -.5);
            Move(.5,11,0);
            MoveArm(500,.5);
            Move(.2, 0, 87);


        }
        else {
            Move(.5,0,11);
            read[0] = checkDistance(15);
            sleep(200);
            read[1] = checkDistance(15);
            sleep(200);
            read[2] = checkDistance(15);
            sleep(200);
            read[3] = checkDistance(15);

            if (read[0] && read[1] && read[2] && read[3]) {
                Move(.5,5,0);
                MoveArm(500,-.5);
                Move(.5,6,0);
                MoveArm(500,.5);
                Move(.5,0,97);
            }
            else {


            }
        }
    }
}

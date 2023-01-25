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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="RightSideAuto", group="Robot")
public class RightSideAuto extends Autonomous_Base {

    HardwareMap robot = new HardwareMap();
    private int color = 0;
    private int checkNum = 0;

    @Override
    public void runOpMode() {

        super.robot.init(super.hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        super.robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        super.robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        super.robot.liftArmL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        super.robot.liftArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        closeClaw();
        Lowgoal(0,0);
        Move(.3,0,-18);
        sleep(500);
        while (color != 6 && color != 9 && color != 10 && checkNum < 5) {
            color = super.robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
            checkNum++;
            sleep(250);
        }
        telemetry.addData("Color Number", color);
        telemetry.update();
        Move(.3,0,-21);
        sleep(1000);
        waitforarmfinish();
        Move(.3,0,0);
        sleep(500);
        Goalreadjust();
        waitforarmfinish();
        releaseClaw();
        Move(.3,-2,0);
        Move(.3,0,-12.5);
        //Move(.3,0,0);
        Grabconeheight();
        waitforarmfinish();
        Move(.3,22,0);
        closeClaw();
        MoveArm(400, 1);
        Move(.5,-47.5,0);
        Move(.3,0,13.5);
        Medgoal(0,0);
        waitforarmfinish();
        Move(.3,3,0);
        Lowgoal(0,0);
        waitforarmfinish();
        Goalreadjust();
        releaseClaw();
        waitforarmfinish();
        Move(.3,-4,0);
        telemetry.addData("Color Number", color);
        telemetry.update();
        if (color == 9 || color == 8) {
            ArmGround(0,0);
            Move(.3,0,-12);
            Move(.3,24,0);
            waitforarmfinish();
        }
        else if (color == 6) {
            ArmGround(0,0);
            Move(.3,0,-12);
            Move(.3, 48,0);
            waitforarmfinish();
        }
        else {
            ArmGround(0,0);
            waitforarmfinish();
        }
    }
}

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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop_22_23", group="Robot")
public class TeleopCode extends LinearOpMode {

    HardwareMap robot = new HardwareMap();

    @Override
    public void runOpMode() {

        double wheelspeed[] = new double[12];

        double lx;
        double lxRuntimeMod;
        double lxModifier;
        double ly;
        double lyRuntimeMod;
        double lyModifier;
        double rx;
        double ly2;
        boolean gamepadCheck;
        String lastButton = "None";
        ElapsedTime runtimely = new ElapsedTime();
        ElapsedTime runtimelx = new ElapsedTime();
        robot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.


            ly = gamepad1.left_stick_y*.7;
            lx = -gamepad1.left_stick_x*.7;
            rx = gamepad1.right_stick_x;
            ly2 = gamepad2.left_stick_y*.5;

            if (gamepad2.x) {
                lastButton = "X";
            }
            else if (gamepad2.y) {
                lastButton = "Y";
            }
            else if (gamepad2.a) {
                lastButton = "A";
            }
            else if (gamepad2.b) {
                lastButton = "B";
            }

            if (ly > -.2 && ly < .2) {
                runtimely.reset();
            }
            lyRuntimeMod = Double.max(runtimely.seconds()*1.3, .3);
            lyModifier = Double.min(lyRuntimeMod,1);

            if (lx > -.2 && lx < .2) {
                runtimelx.reset();
            }
            lxRuntimeMod = Double.max(runtimelx.seconds()*1.3, .3);
            lxModifier = Double.min(lxRuntimeMod,1);

            if (rx > -.2 && rx < .2) {
                rx = 0;
            }

            if (ly2 > -.2 && ly2 < .2) {
                ly2 = 0;
            }

            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                gamepadCheck = true;
            }
            else {
                gamepadCheck = false;
            }

            if (gamepad1.dpad_up) {
                wheelspeed[8] = -1;
                wheelspeed[9] = -1;
                wheelspeed[10] = -1;
                wheelspeed[11] = -1;
            }
            else if (gamepad1.dpad_down) {
                wheelspeed[8] = 1;
                wheelspeed[9] = 1;
                wheelspeed[10] = 1;
                wheelspeed[11] = 1;
            }
            else if (gamepad1.dpad_right) {
                wheelspeed[8] = 1;
                wheelspeed[9] = -1;
                wheelspeed[10] = -1;
                wheelspeed[11] = 1;
            }
            else if (gamepad1.dpad_left) {
                wheelspeed[8] = -1;
                wheelspeed[9] = 1;
                wheelspeed[10] = 1;
                wheelspeed[11] = -1;
            }

            wheelspeed[0] = (ly*lyModifier - lx*lxModifier);
            wheelspeed[1] = (ly*lyModifier + lx*lxModifier);
            wheelspeed[2] = (ly*lyModifier + lx*lxModifier);
            wheelspeed[3] = (ly*lyModifier - lx*lxModifier);
            wheelspeed[4] = rx*.5 + wheelspeed[0];
            wheelspeed[5] = -rx*.5 + wheelspeed[1];
            wheelspeed[6] = rx*.5 + wheelspeed[2];
            wheelspeed[7] = -rx*.5 + wheelspeed[3];

            if (rx == 0 && !gamepadCheck) {
                robot.Motor1.setPower(wheelspeed[0]);
                robot.Motor2.setPower(wheelspeed[1]);
                robot.Motor3.setPower(wheelspeed[2]);
                robot.Motor4.setPower(wheelspeed[3]);
            }
            else if (gamepadCheck) {
                robot.Motor1.setPower(wheelspeed[8]);
                robot.Motor2.setPower(wheelspeed[9]);
                robot.Motor3.setPower(wheelspeed[10]);
                robot.Motor4.setPower(wheelspeed[11]);
            }
            else {
                robot.Motor1.setPower(wheelspeed[4]);
                robot.Motor2.setPower(wheelspeed[5]);
                robot.Motor3.setPower(wheelspeed[6]);
                robot.Motor4.setPower(wheelspeed[7]);
            }

            robot.liftArmL.setPower(ly2);
            robot.liftArmR.setPower(ly2);

            if (gamepad2.a) {
                robot.Claw.setPosition(.8);
            }

            if (gamepad2.b) {
                robot.Claw.setPosition(.45);
            }

            if (gamepad2.x && lastButton.equals("X")) {
                robot.Claw.setPosition(.8);
            }
            else if (!gamepad2.x && lastButton.equals("X")) {
                robot.Claw.setPosition(.45);
            }

            if (gamepad2.y && lastButton.equals("Y")) {
                robot.Claw.setPosition(.45);
            }
            else if (!gamepad2.y && lastButton.equals("Y")) {
                robot.Claw.setPosition(.8);
            }
        }
    }
}

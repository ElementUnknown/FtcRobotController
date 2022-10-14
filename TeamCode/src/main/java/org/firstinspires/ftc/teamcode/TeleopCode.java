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

@TeleOp(name="Robot: Teleop POV", group="Robot")
public class TeleopCode extends LinearOpMode {

    HardwareMap robot = new HardwareMap();

    @Override
    public void runOpMode() {

        double horizontalSpeed;
        double verticalSpeed;
        double turnSpeed;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            verticalSpeed = gamepad1.left_stick_y;
            horizontalSpeed = gamepad1.left_stick_x;
            turnSpeed = gamepad1.right_stick_x;

            if (verticalSpeed > -.2 && verticalSpeed < .2) {
                verticalSpeed = 0;
            }

            if (horizontalSpeed < .2 && horizontalSpeed > -.2) {
                horizontalSpeed = 0;
            }

            if (verticalSpeed != 0 && horizontalSpeed == 0 && turnSpeed == 0) {
                robot.Motor1.setPower(verticalSpeed);
                robot.Motor2.setPower(verticalSpeed);
                robot.Motor3.setPower(verticalSpeed);
                robot.Motor4.setPower(verticalSpeed);
            }
            else if (verticalSpeed == 0 && horizontalSpeed != 0 && turnSpeed == 0) {
                robot.Motor1.setPower(-horizontalSpeed);
                robot.Motor2.setPower(-horizontalSpeed);
                robot.Motor3.setPower(horizontalSpeed);
                robot.Motor4.setPower(horizontalSpeed);
            }
            else if (verticalSpeed == 0 && horizontalSpeed == 0 && turnSpeed != 0) {
                robot.Motor1.setPower(turnSpeed);
                robot.Motor2.setPower(-turnSpeed);
                robot.Motor3.setPower(turnSpeed);
                robot.Motor4.setPower(-turnSpeed);
            }
            else {
                robot.Motor1.setPower(0);
                robot.Motor2.setPower(0);
                robot.Motor3.setPower(0);
                robot.Motor4.setPower(0);
            }
        }
    }
}

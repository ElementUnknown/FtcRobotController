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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

@TeleOp(name="Teleop _23_24", group="Robot")
public class TeleopCode extends Autonomous_Base {


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
        //double rx2;
        double angleDistance = 0;
        double initHeading = 0;
        boolean gamepadCheck;
        double ry2 = 0;
        boolean leftStickIsActive = false;
        boolean rightstickisactive = false;
        double SpeedMod;
        boolean lBumperSwap = false;
        boolean rBumperSwap = false;
        String lastButton = "None";
        ElapsedTime runtimely = new ElapsedTime();
        ElapsedTime runtimelx = new ElapsedTime();
        ElapsedTime TILT = new ElapsedTime();
        super.robot.init(super.hardwareMap);
        super.robot.Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        super.robot.Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        super.robot.Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        super.robot.Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.


            ly = gamepad1.left_stick_y*1;
            lx = -gamepad1.left_stick_x*.7;
            rx = gamepad1.right_stick_x;
            ly2 = gamepad2.left_stick_y*-1.0;
            ry2 = gamepad2.right_stick_y;

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
            lyRuntimeMod = Double.max(runtimely.seconds()*1.7, .3);
            lyModifier = Double.min(lyRuntimeMod,1);

            if (lx > -.2 && lx < .2) {
                runtimelx.reset();
            }
            lxRuntimeMod = Double.max(runtimelx.seconds()*1.7, .3);
            lxModifier = Double.min(lxRuntimeMod,1);

            if (rx > -.2 && rx < .2) {
                rx = 0;
            }

            if (ly2 > -.2 && ly2 < .2) {
                ly2 = 0;
            }
            if(gamepad1.b) SpeedMod = .5;
            else SpeedMod = 1;
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
            if (rx != 0) rightstickisactive = true;
            else rightstickisactive = false;
            if (lx != 0 || ly != 0) {
                leftStickIsActive = true;
            }
            else {
                leftStickIsActive = false;
            }

            if (!leftStickIsActive || rightstickisactive) {
                initHeading = getHeading();
            }

            angleDistance = getHeading() - initHeading;
            if (angleDistance > 180) {
                angleDistance = angleDistance - 360;
            }
            if (angleDistance < -180) {
                angleDistance = angleDistance + 360;
            }

            wheelspeed[0] = (ly*lyModifier - lx*lxModifier - (angleDistance / 90 ));
            wheelspeed[1] = (ly*lyModifier + lx*lxModifier + (angleDistance / 90 ));
            wheelspeed[2] = (ly*lyModifier + lx*lxModifier - (angleDistance / 90 ));
            wheelspeed[3] = (ly*lyModifier - lx*lxModifier + (angleDistance / 90 ));
            wheelspeed[4] =  rx*.3 + (ly*lyModifier - lx*lxModifier);
            wheelspeed[5] = -rx*.3 + (ly*lyModifier + lx*lxModifier);
            wheelspeed[6] =  rx*.3 + (ly*lyModifier - lx*lxModifier);
            wheelspeed[7] = -rx*.3 + (ly*lyModifier + lx*lxModifier);

            if (!gamepadCheck && (lx != 0 || ly != 0) && rx == 0) {
                super.robot.Motor1.setPower(SpeedMod * wheelspeed[0]);
                super.robot.Motor2.setPower(SpeedMod * wheelspeed[1]);
                super.robot.Motor3.setPower(SpeedMod * wheelspeed[2]);
                super.robot.Motor4.setPower(SpeedMod * wheelspeed[3]);
            }
            else if (gamepadCheck) {
                super.robot.Motor1.setPower(SpeedMod * wheelspeed[8]);
                super.robot.Motor2.setPower(SpeedMod * wheelspeed[9]);
                super.robot.Motor3.setPower(SpeedMod * wheelspeed[10]);
                super.robot.Motor4.setPower(SpeedMod * wheelspeed[11]);
            }
            else if (rx != 0){
                super.robot.Motor1.setPower(SpeedMod * wheelspeed[4]);
                super.robot.Motor2.setPower(SpeedMod * wheelspeed[5]);
                super.robot.Motor3.setPower(SpeedMod * wheelspeed[6]);
                super.robot.Motor4.setPower(SpeedMod * wheelspeed[7]);
            }
            else {
                super.robot.Motor1.setPower(0);
                super.robot.Motor2.setPower(0);
                super.robot.Motor3.setPower(0);
                super.robot.Motor4.setPower(0);
            }
            if (gamepad1.right_trigger > .5){
                super.robot.Motor1.setPower(.75);
                super.robot.Motor2.setPower(-.75);
                super.robot.Motor3.setPower(.75);
                super.robot.Motor4.setPower(-.75);
            }
            else if (gamepad1.left_trigger > .5){
                super.robot.Motor1.setPower(-.75);
                super.robot.Motor2.setPower(.75);
                super.robot.Motor3.setPower(-.75);
                super.robot.Motor4.setPower(.75);

            }

            if (gamepad2.y) {
                super.robot.intake.setPower(-1);
            }
            else {
                super.robot.intake.setPower(0);
            }

            if (gamepad2.a) {
                super.robot.intake.setPower(1);
            }
            else {
                super.robot.intake.setPower(0);
            }

            super.robot.PivotArm.setPower(ry2);

            if (gamepad2.left_bumper && lBumperSwap) {
                //super.robot.clawL.setPosition(.7);
                lBumperSwap = false;
            }
            else if (gamepad2.left_bumper) {
               // super.robot.clawL.setPosition(.3);
                lBumperSwap = true;
            }

            if (gamepad2.right_bumper && rBumperSwap) {
                //super.robot.clawR.setPosition(.7);
                rBumperSwap = false;
            }
            else if (gamepad2.right_bumper) {
                //super.robot.clawR.setPosition(.3);
                rBumperSwap = true;
            }

            if (gamepad2.dpad_up) {
                super.robot.liftArm.setPower(1);
            }
            else if (gamepad2.dpad_down) {
                super.robot.liftArm.setPower(-1);
            }
            else {
                super.robot.liftArm.setPower(0);
            }

            /*if (gamepad2.x && lastButton.equals("X")) {
                super.robot.Claw.setPosition(.35);
            }
            else if (!gamepad2.x && lastButton.equals("X")) {
                super.robot.Claw.setPosition(.0);
            }

            if (gamepad2.y && lastButton.equals("Y")) {
                super.robot.Claw.setPosition(.0);
            }
            else if (!gamepad2.y && lastButton.equals("Y")) {
                super.robot.Claw.setPosition(.35);
            }*/
            /** The Encoder System for arm adjustment may be used later as it may become helpful**/
    /*This is the arm encoder*/        /*if (gamepad2.dpad_left && ly2 == 0.0) {
                multiplier = (double)(760 - super.robot.liftArmL.getCurrentPosition()) / 200.0;
                liftpower = multiplier; //200 is the constant multipication variable, this determines the acceleration at start and stop
                if (liftpower > 1){
                    liftpower =1;
                }
                if (liftpower < -1){
                    liftpower = -1;
                }
                super.robot.liftArmR.setPower(liftpower);
                super.robot.liftArmL.setPower(liftpower);
            }
            else if (gamepad2.dpad_up && ly2 == 0.0) {
                multiplier = (double)(1250 - super.robot.liftArmL.getCurrentPosition()) / 200.0;
                liftpower = multiplier; //200 is the constant multipication variable, this determines the acceleration at start and stop
                if (liftpower > 1){
                    liftpower =1;
                }
                if (liftpower < -1){
                    liftpower = -1;
                }
                super.robot.liftArmR.setPower(liftpower);
                super.robot.liftArmL.setPower(liftpower);
            }
            else if (gamepad2.dpad_right && ly2 == 0.0) {
                multiplier = (double)(550 - super.robot.liftArmL.getCurrentPosition()) / 200.0;
                liftpower = multiplier; //200 is the constant multipication variable, this determines the acceleration at start and stop
                if (liftpower > 1){
                    liftpower =1;
                }
                if (liftpower < -1){
                    liftpower = -1;
                }
                super.robot.liftArmR.setPower(liftpower);
                super.robot.liftArmL.setPower(liftpower);
            }
            else if (gamepad2.dpad_down && ly2 == 0.0){
                multiplier = (double)(1 - super.robot.liftArmL.getCurrentPosition()) / 200.0;
                liftpower = multiplier; //200 is the constant multipication variable, this determines the acceleration at start and stop
                if (liftpower > 1){
                    liftpower =1;

                }
                if (liftpower < -1){
                    liftpower = -1;
                }
                super.robot.liftArmR.setPower(liftpower);
                super.robot.liftArmL.setPower(liftpower);
            }*/
            /** The anti-tip system can be modified for new hardware and still used, we should check the instability of the robot at various arm heights**/
    /*This is the Anti-tip*/        /*super.robot.angles = super.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (super.robot.angles.secondAngle > 10 && TILT.milliseconds() > 150 && super.robot.liftArmL.getCurrentPosition() > 1000){
                EmergencyCorrectionForward();
            }
            if (super.robot.angles.secondAngle < -10 && TILT.milliseconds() > 150 && super.robot.liftArmL.getCurrentPosition() > 1000){
                EmergencyCorrectionBackwards();
            }
            if (Math.abs(super.robot.angles.secondAngle) < 15 ) TILT.reset();
            telemetry.addData("", super.robot.angles.secondAngle);
            telemetry.addData("Distance", super.robot.ods.getDistance(DistanceUnit.INCH));
            if(super.robot.ods.getDistance(DistanceUnit.INCH) < 7){
                telemetry.addData("Goal Aligned", "");
            }*/
            telemetry.update();
        }
    }
}

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

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

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

@TeleOp(name="Field Centric", group="Robot")
public class TeleopCodeFieldCenteric extends Autonomous_Base {

    @Override
    public void runOpMode() {

        double wheelspeed[] = new double[12];

        double lx;
        double ly;
        double rx;
        double ly2;
        boolean LBumperOpen = true;
        boolean RBumperOpen = true;
        boolean LLastPressed = false;
        boolean RLastPressed = false;
        //double rx2;
        double angleDistance = 0;
        double initHeading = 0;
        boolean gamepadCheck;
        double multiplier = 0;
        double ry2 = 0;
        double liftpower = 0;
        boolean leftStickIsActive = false;
        boolean rightstickisactive = false;
        double speedMod =1;
        double totPower = 0;
        double AngleJ = 0;
        double Pheta = 0;
        double PowerX = 0;
        double PowerY = 0;
        double ARX =0;
        double ARY = 0;
        double yaw = 0;
        double Bearing;
        double KP;
        double rx2;
        int TargetID = -1;
        String lastButton = "None";
        ElapsedTime stickRuntime = new ElapsedTime();
        ElapsedTime DistanceTime = new ElapsedTime();
        double stickRuntimeMod;
        double stickMod;
        ElapsedTime TILT = new ElapsedTime();
        boolean Centric = false;
        boolean DoubleButton = false;

        super.robot.init(super.hardwareMap);
        super.robot.AprilInit(super.hardwareMap);


        super.robot.Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        super.robot.Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        super.robot.Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        super.robot.Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        super.robot.PivotArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //super.robot.liftArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //super.robot.liftArmL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //super.robot.liftArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //super.robot.liftArmL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // if (super.robot.USE_WEBCAM)
         //   setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean targetFound = false;
            robot.desiredTag = null;
            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            TargetID = -10;
            ly = -gamepad1.left_stick_y;
            lx = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            ly2 = gamepad2.right_stick_y;
            ry2 = gamepad2.left_stick_y;
            if (gamepad1.dpad_left) TargetID = 4;
            if (gamepad1.dpad_up) TargetID = 5;
            if(Math.abs(ly2) < .2) ly2 =0;
            if (gamepad1.dpad_right) TargetID = 6;
            gamepadCheck = (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right);
            if (gamepadCheck) {
                telemetry.addData("Checking for tags", "");
                List<AprilTagDetection> currentDetections = super.robot.aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if ((detection.metadata != null) &&
                            ((detection.id == TargetID - 3) || (detection.id == TargetID))) {
                        targetFound = true;
                        super.robot.desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                    }
                }
            }
            if (targetFound) {
                yaw = super.robot.desiredTag.ftcPose.yaw;
                ARX = super.robot.desiredTag.ftcPose.x;
                ARY = super.robot.desiredTag.ftcPose.y -7;
                Bearing = super.robot.desiredTag.ftcPose.bearing;
                KP = Math.abs(ARX) + Math.abs(ARY);
                KP = Math.max(12,KP);


            } else {
                ARX = 0;
                ARY = 0;
                yaw = 0;
                Bearing =0;
                KP = 10;
            }
            if (gamepad2.x) {
                lastButton = "X";
            } else if (gamepad2.y) {
                lastButton = "Y";
            } else if (gamepad2.a) {
                lastButton = "A";
            } else if (gamepad2.b) {
                lastButton = "B";
            }

            if (!leftStickIsActive && !rightstickisactive) {
                stickRuntime.reset();
            }
            stickRuntimeMod = Double.max(stickRuntime.seconds() * 1.7, .3);
            stickMod = Double.min(stickRuntimeMod, 1);


            //if (!checkDistance(6)) {
               // DistanceTime.reset();

            //}
            //if(DistanceTime.milliseconds() > 350){
               // speedMod = stickMod*(getDistance()/9);
            //}
            if(gamepad1.b) {
                speedMod = .5*stickMod;
            }
            else {
                speedMod = 1*stickMod;
            }
            if(ly2 < 0) super.robot.elbow.setPosition(1);
            else if(ly2 > .5)super.robot.elbow.setPosition(.6);
            else if(ly2 > 0) super.robot.elbow.setPosition(.35);
            //else super.robot.elbow.setPosition(super.robot.elbow.getPosition());
            if (rx > -.2 && rx < .2) {
                rx = 0;
            }




            rightstickisactive = (rx != 0);
            leftStickIsActive = (lx != 0 || ly != 0);

            if (!leftStickIsActive || rightstickisactive) {
                initHeading = getHeading();
            }
            if(gamepad1.x && gamepad1.b && !DoubleButton && Centric){
                DoubleButton = true;
                Centric = false;
            }
            if (gamepad1.x && gamepad1.b && !DoubleButton && !Centric){
                DoubleButton = true;
                Centric = true;
            }
            if(!gamepad1.x || !gamepad1.b){
                DoubleButton = false;
            }

            if (gamepad1.x || !Centric){
                super.robot.initAngle = getHeading();
            }
            angleDistance = getHeading() - initHeading;
            if (angleDistance > 180) {
                angleDistance = angleDistance - 360;
            }
            if (angleDistance < -180) {
                angleDistance = angleDistance + 360;
            }



            totPower = Math.sqrt(Math.pow(lx, 2) + Math.pow(ly, 2));
            AngleJ = Math.toDegrees((Math.atan2(-lx, -ly)));
            Pheta = AngleJ - (getHeading() - super.robot.initAngle);
            if(!opModeIsActive()) break;
            telemetry.addData("lift position", super.robot.liftArm.getCurrentPosition());
            /*telemetry.addData("Angle of joystick", AngleJ);
            telemetry.addData("Angle of Robot", getHeading());
            telemetry.addData("Angle of adjustment", Pheta);
            telemetry.addData("PowerX", PowerX);
            telemetry.addData("PowerY", PowerY);
            telemetry.addData("Total Power", totPower);
            telemetry.addData("Motor1", wheelspeed[0]);
            telemetry.addData("Motor2", wheelspeed[1]);
            telemetry.addData("Motor3", wheelspeed[2]);
            telemetry.addData("Motor4", wheelspeed[3]);
            telemetry.addData("InitHeading", initHeading);
            telemetry.addData("angledistance", angleDistance);
            telemetry.addData("Leftsitckactive", leftStickIsActive);
            telemetry.addData("RightStickisActive", rightstickisactive); */

            PowerX = totPower * Math.sin(Math.toRadians(Pheta));
            PowerY = totPower * Math.cos(Math.toRadians(Pheta));
            wheelspeed[0] = rx*.5 + (-PowerY + PowerX - (angleDistance / 90 ));
            wheelspeed[1] = -rx*.5 + (-PowerY - PowerX + (angleDistance / 90 ));
            wheelspeed[2] = rx*.5 + (-PowerY - PowerX - (angleDistance / 90 ));
            wheelspeed[3] = -rx*.5 + (-PowerY + PowerX + (angleDistance / 90 ));
            wheelspeed[8] = .4*((ARY - ARX)/KP) - ((Bearing) / 90 ) - (yaw / 150);
            wheelspeed[9] = .4*((ARY + ARX)/KP) + ((Bearing) / 90) + (yaw / 150);
            wheelspeed[10] = .4*((ARY + ARX)/KP) - ((Bearing) / 90) - (yaw / 150);
            wheelspeed[11] = .4*((ARY - ARX)/KP) + ((Bearing) / 90) + (yaw / 150);

            /*wheelspeed[4] = rx*.5 + (-PowerY*lyModifier - PowerX*lxModifier);
            wheelspeed[5] = -rx*.5 + (-PowerY*lyModifier + PowerX*lxModifier);
            wheelspeed[6] = rx*.5 + (-PowerY*lyModifier - PowerX*lxModifier);
            wheelspeed[7] = -rx*.5 + (-PowerY*lyModifier + PowerX*lxModifier);*/

            if(!gamepadCheck || !targetFound) {
                super.robot.Motor1.setPower(speedMod * wheelspeed[0]);
                super.robot.Motor2.setPower(speedMod * wheelspeed[1]);
                super.robot.Motor3.setPower(speedMod * wheelspeed[2]);
                super.robot.Motor4.setPower(speedMod * wheelspeed[3]);
            }
            else{
                super.robot.Motor1.setPower(wheelspeed[8]);
                super.robot.Motor2.setPower(wheelspeed[9]);
                super.robot.Motor3.setPower(wheelspeed[10]);
                super.robot.Motor4.setPower(wheelspeed[11]);
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
                telemetry.addData("Y Pressed: ",gamepad1.y);
            }
            else if (gamepad2.a) {
                super.robot.intake.setPower(1);
                telemetry.addData("A Pressed: ",gamepad1.a);
            }
            else {
                super.robot.intake.setPower(0);
            }

            if (gamepad2.dpad_left){
                super.robot.winch.setPower(1);
            }
            else if (gamepad2.dpad_right) {
                super.robot.winch.setPower(-1);
            }
            else {
                super.robot.winch.setPower(0);
            }

            if (Math.abs(ry2) < .1 ){
               ry2 = 0;
            }
            if(!gamepad2.right_bumper && gamepad2.right_trigger < .3) {
                super.robot.PivotArm.setPower(ry2 * .75);
            }

            if (gamepad2.left_bumper){
                closeClaw();
            }
            else if (gamepad2.left_trigger > .3){
                openClaw();
            }

            if (gamepad2.right_bumper){
                MovetoPlace();
            }
            else if (gamepad2.right_trigger > .3){
                MovetoGrab();
            }
            else {

                super.robot.PivotArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (gamepad1.right_bumper && gamepad1.left_bumper){
                super.robot.Launch.setPosition(0);
            }

            if (gamepad2.dpad_up) {
                super.robot.liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                super.robot.liftArm.setPower(1);
            }
            else if (gamepad2.dpad_down) {
                super.robot.liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                super.robot.liftArm.setPower(-1);
            }
            else if(!gamepad2.right_bumper && !(gamepad2.right_trigger > .2)){
                super.robot.liftArm.setPower(0);
            }
            if(getTilt() < -72 || super.robot.liftArm.getCurrentPosition() < 1000){
                TILT.reset();
            }
            if(TILT.milliseconds() > 150){
                ANTITIP();
            }
            telemetry.update();
        }
    }
}

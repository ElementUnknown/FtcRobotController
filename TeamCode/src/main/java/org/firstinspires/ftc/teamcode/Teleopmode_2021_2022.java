package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Teleop Mode", group="Robot")
//@Disabled

    public class Teleopmode_2021_2022 extends LinearOpMode {

    Hardware20212022 robot = new Hardware20212022();
    double wheelspeeds[] = new double[4];
    double ly;
    double lx;
    double rightturnpower;
    double leftturnpower;

    //BNO055IMU imu;
    //Orientation angles;

    //ColorSensor colorSensor;*/

    double wheelspinpower;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime Timerun = new ElapsedTime();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.Topleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Toprightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Bottomleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Bottomrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            float hsvValues[] = {0F,0F,0F};
            final float values[] = hsvValues;
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
            boolean bPrevState = false;
            boolean bCurrState = false;
            boolean bLedOn = true;
            colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");*/

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.update();

            lx = gamepad1.left_stick_x;
            ly = gamepad1.left_stick_y;

            if (gamepad2.right_trigger > .1 ){
                robot.wheelspin.setPower(.6);
            }
            else if (gamepad2.left_trigger > .1){
                robot.wheelspin.setPower(-.6);
            }
            else {
                robot.wheelspin.setPower(0);
            }

            if (lx < .05 && lx > -.05) {
                lx = 0;
            }
            if (ly < .05 && ly > -.05) {
                ly = 0;
            }
            if (gamepad1.right_trigger > .05) {
                rightturnpower = gamepad1.right_trigger;
            }
            else {
                rightturnpower = 0;
            }
            if (gamepad1.left_trigger > .05) {
                leftturnpower = gamepad1.left_trigger;
            }
            else {
                leftturnpower = 0;
            }

            wheelspeeds[0] = lx + ly;
            wheelspeeds[1] = -lx + ly;
            wheelspeeds[2] = -lx + ly;
            wheelspeeds[3] = lx + ly;

            if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0) {
                robot.Toprightmotor.setPower(wheelspeeds[0]);
                robot.Topleftmotor.setPower(wheelspeeds[1]);
                robot.Bottomrightmotor.setPower(wheelspeeds[2]);
                robot.Bottomleftmotor.setPower(wheelspeeds[3]);
            } else if (gamepad1.right_trigger > .05 && gamepad1.left_trigger == 0) {
                robot.Toprightmotor.setPower(rightturnpower);
                robot.Topleftmotor.setPower(-rightturnpower);
                robot.Bottomrightmotor.setPower(rightturnpower);
                robot.Bottomleftmotor.setPower(-rightturnpower);
            } else if (gamepad1.left_trigger > .05 && gamepad1.right_trigger == 0) {
                robot.Toprightmotor.setPower(-leftturnpower);
                robot.Topleftmotor.setPower(leftturnpower);
                robot.Bottomrightmotor.setPower(-leftturnpower);
                robot.Bottomleftmotor.setPower(leftturnpower);
            } else {
                robot.Topleftmotor.setPower(0);
                robot.Toprightmotor.setPower(0);
                robot.Bottomrightmotor.setPower(0);
                robot.Bottomleftmotor.setPower(0);
            }
            
            if (gamepad2.right_bumper) {
                robot.turntable.setPower(.5);
            }
            else if (gamepad2.left_bumper) {
                robot.turntable.setPower(-.5);
            }
            else {
                robot.turntable.setPower(0);
            }

            if (gamepad2.a) {
                robot.arm.setPower(-.5);
            }
            else if (gamepad2.b) {
                robot.arm.setPower(.5);
            }
            else {
                robot.arm.setPower(0);
            }

            /*if (gamepad2.b = true) {
                robot.wheelspin.setPower(1);
            } else {
                robot.wheelspin.setPower(0);
            }*/

                //Gyro Sensor
                /*angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                telemetry.addData("Heading", angles.firstAngle);
                telemetry.addData("Roll", angles.secondAngle);
                telemetry.addData("Pitch", angles.thirdAngle);
                telemetry.update();*/

            }
        }
    }
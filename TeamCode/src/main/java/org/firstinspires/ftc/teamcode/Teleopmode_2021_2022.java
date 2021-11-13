package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="TeleopMode", group="Robot")
//@Disabled

    public class Teleopmode_2021_2022 extends LinearOpMode {

    Hardware20212022 robot = new Hardware20212022();
    double wheelspeeds[] = new double[8];
    double ly;
    double lx;
    double turnpower;
    boolean rightBumper;
    boolean leftBumper;
    boolean rightwheelspinset;
    boolean lastrightwheelspinset;
    boolean lastleftwheelspinset;
    boolean leftwheelspinset;

    /*BNO055IMU imu;
    Orientation angles;

    ColorSensor colorSensor;*/

    double wheelspinpower;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime Timerun = new ElapsedTime();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

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
            rightBumper = gamepad2.right_bumper;
            leftBumper = gamepad2.left_bumper;


            if (lx < .05 && lx > -.05) {
                lx = 0;
            }
            if (ly < .05 && ly > -.05) {
                ly = 0;
            }
            if (turnpower < .05 && turnpower > -.05) {
                turnpower = 0;
            }

            if (gamepad1.right_trigger > .05) {
                turnpower = gamepad1.right_trigger;
            } else if (gamepad1.left_trigger > .075) {
                turnpower = -gamepad1.left_trigger;
            } else {
                turnpower = 0;
            }

            if (gamepad2.right_trigger > .05) {
                rightwheelspinset = true;
            } else if (gamepad2.left_trigger > .05) {
                leftwheelspinset = true;
            } else {
                rightwheelspinset = false;
                leftwheelspinset = false;
            }

            wheelspeeds[0] = -lx + ly;
            wheelspeeds[1] = lx + ly;
            wheelspeeds[2] = lx + ly;
            wheelspeeds[3] = -lx + ly;
            wheelspeeds[4] = (-turnpower + wheelspeeds[0]) / 2;
            wheelspeeds[5] = (turnpower + wheelspeeds[1]) / 2;
            wheelspeeds[6] = (turnpower + wheelspeeds[2]) / 2;
            wheelspeeds[7] = (-turnpower + wheelspeeds[3]) / 2;

            if (turnpower == 0) {
                robot.Toprightmotor.setPower(wheelspeeds[0]);
                robot.Topleftmotor.setPower(wheelspeeds[1]);
                robot.Bottomrightmotor.setPower(wheelspeeds[2]);
                robot.Bottomleftmotor.setPower(wheelspeeds[3]);
            } else if (turnpower != 0) {
                robot.Toprightmotor.setPower(-wheelspeeds[4]);
                robot.Topleftmotor.setPower(-wheelspeeds[5]);
                robot.Bottomleftmotor.setPower(-wheelspeeds[6]);
                robot.Bottomrightmotor.setPower(-wheelspeeds[7]);
            } else {
                robot.Topleftmotor.setPower(0);
                robot.Toprightmotor.setPower(0);
                robot.Bottomrightmotor.setPower(0);
                robot.Bottomleftmotor.setPower(0);
            }

            if (rightwheelspinset) {
                robot.wheelspin.setPower(.6);
                Timerun.reset();
               /* if (rightwheelspinset && lastrightwheelspinset == false) {
                    runtime.reset();
                }

                wheelspinpower = .1;
                while (Timerun.milliseconds() < 3500 && rightwheelspinset) {
                    while (Timerun.milliseconds() < 3000) {
                        if (runtime.milliseconds() < 300) {

                            wheelspinpower = runtime.milliseconds() / 300;
                        } else {
                            wheelspinpower = 1;
                        }
                        robot.wheelspin.setPower(wheelspinpower);
                    }
                    robot.wheelspin.setPower(0);
                }*/
            } else if (leftwheelspinset) {
                robot.wheelspin.setPower(-.6);
                Timerun.reset();
                /*if (leftwheelspinset && lastleftwheelspinset == false) {
                    runtime.reset();
                }

                wheelspinpower = .1;
                while (Timerun.milliseconds() < 3500 && leftwheelspinset) {
                    while (Timerun.milliseconds() < 3000) {
                        if (runtime.milliseconds() < 300) {

                            wheelspinpower = runtime.milliseconds() / 300;
                        } else {
                            wheelspinpower = 1;
                        }
                        robot.wheelspin.setPower(-wheelspinpower);
                    }
                    robot.wheelspin.setPower(0);
                }*/

            } else {
                robot.wheelspin.setPower(0);
            }

            lastrightwheelspinset = rightwheelspinset;
            lastleftwheelspinset = leftwheelspinset;


            /*if (gamepad2.a = true){
                robot.dustpan.setPosition(0);
            }
            else {
                robot.dustpan.setPosition(45);
            }*/

            /*if (gamepad2.b = true) {
                robot.wheelspin.setPower(1);
            } else {
                robot.wheelspin.setPower(0);
            }*/

            /*if (gamepad2.y = true) {
                robot.leftintake.setPower(.5);
                robot.rightintake.setPower(.5);
            }
            else {
                robot.leftintake.setPower(0);
                robot.rightintake.setPower(0);
            }*/








                /*Gyro and Color Sensor
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                bCurrState = gamepad1.x;

                if (bCurrState && (bCurrState != bPrevState))  {
                    bLedOn = !bLedOn;
                    colorSensor.enableLed(bLedOn);
                }

                bPrevState = bCurrState;
                Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

                telemetry.addData("Heading", angles.firstAngle);
                telemetry.addData("Roll", angles.secondAngle);
                telemetry.addData("Pitch", angles.thirdAngle);

                telemetry.addData("LED", bLedOn ? "On" : "Off");
                telemetry.addData("Clear", colorSensor.alpha());
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());
                telemetry.addData("Hue", hsvValues[0]);
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
                });


                telemetry.addData("Heading", angles.firstAngle);
                telemetry.addData("Roll", angles.secondAngle);
                telemetry.addData("Pitch", angles.thirdAngle);
                telemetry.update();

            }

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });*/
        }


    }
}
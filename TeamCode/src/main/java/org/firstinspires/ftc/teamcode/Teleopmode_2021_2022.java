package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

    @TeleOp(name="TeleopMode", group="Robot")
//@Disabled

    public class Teleopmode_2021_2022 extends LinearOpMode {

        Hardware20212022             robot = new Hardware20212022();
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

        double wheelspinpower;

        private ElapsedTime runtime = new ElapsedTime();
        private ElapsedTime Timerun = new ElapsedTime();


        @Override
        public void runOpMode() {

            robot.init(hardwareMap);

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
                }
                else if (gamepad1.left_trigger > .075) {
                    turnpower = -gamepad1.left_trigger;
                }
                else {
                    turnpower = 0;
                }
                if (gamepad2.right_trigger > .05) {
                    rightwheelspinset = true;
                }
                else if (gamepad2.left_trigger > .05) {
                    leftwheelspinset = true;
                }
                else {
                    rightwheelspinset = false;
                    leftwheelspinset = false;
                }

                wheelspeeds[0] = lx + ly;
                wheelspeeds[1] = -lx + ly;
                wheelspeeds[2] = -lx + ly;
                wheelspeeds[3] = lx + ly;
                wheelspeeds[4] = (-turnpower + wheelspeeds[0])/2;
                wheelspeeds[5] = (turnpower + wheelspeeds[1])/2;
                wheelspeeds[6] = (turnpower + wheelspeeds[2])/2;
                wheelspeeds[7] = (-turnpower + wheelspeeds[3])/2;

                if (turnpower == 0) {
                    robot.Toprightmotor.setPower(wheelspeeds[0]);
                    robot.Topleftmotor.setPower(wheelspeeds[1]);
                    robot.Bottomrightmotor.setPower(wheelspeeds[2]);
                    robot.Bottomleftmotor.setPower(wheelspeeds[3]);
                }
                else if (turnpower != 0) {
                    robot.Toprightmotor.setPower(-wheelspeeds[4]);
                    robot.Topleftmotor.setPower(-wheelspeeds[5]);
                    robot.Bottomleftmotor.setPower(-wheelspeeds[6]);
                    robot.Bottomrightmotor.setPower(-wheelspeeds[7]);
                }
                else {
                    robot.Topleftmotor.setPower(0);
                    robot.Toprightmotor.setPower(0);
                    robot.Bottomrightmotor.setPower(0);
                    robot.Bottomleftmotor.setPower(0);
                }
                if (rightwheelspinset ) {
                    robot.wheelspin.setPower(.6);
                    /*Timerun.reset();
                    if (rightwheelspinset && lastrightwheelspinset == false){
                    runtime.reset();
                    }

                    wheelspinpower = .1;
                    while (Timerun.milliseconds() < 3500 && rightwheelspinset){
                        while(Timerun.milliseconds() < 3000){
                    if (runtime.milliseconds() < 300){

                       wheelspinpower = runtime.milliseconds()/300;
                    }
                    else {
                        wheelspinpower = 1;
                    }
                    robot.wheelspin.setPower(wheelspinpower);
                    }
                        robot.wheelspin.setPower(0);
                    }*/
                }
                else if (leftwheelspinset ) {

                    robot.wheelspin.setPower(-.6);
                    /*Timerun.reset();
                    if (leftwheelspinset && lastleftwheelspinset == false) {
                        runtime.reset();
                    }

                    wheelspinpower = .1;
                    while (Timerun.milliseconds() < 3500 && leftwheelspinset){
                        while(Timerun.milliseconds() < 3000)
                        {
                             if (runtime.milliseconds() < 300) {

                            wheelspinpower = runtime.milliseconds() / 300;
                             } else {
                            wheelspinpower = 1;
                        }
                    robot.wheelspin.setPower(-wheelspinpower);
                        }
                        robot.wheelspin.setPower(0);
                }*/

                }

                else {
                    robot.wheelspin.setPower(0);
                }

                if (gamepad2.right_bumper) {
                    robot.Arm.setPower(-.6);
                }
                else if (gamepad2.left_bumper){
                    robot.Arm.setPower(.6);
                }
                else {
                    robot.Arm.setPower(0);
                }

                lastrightwheelspinset = rightwheelspinset;
                lastleftwheelspinset = leftwheelspinset;
                /*if (gamepad2.left_trigger > .5){
                    robot.lift.setPower(-1);
                }
                else if (gamepad2.left_trigger < .5 && gamepad2.right_trigger < .5){
                    robot.lift.setPower(0);
                }

                if (gamepad2.right_trigger > .5){
                    robot.lift.setPower(1);
                }
                else if (gamepad2.left_trigger < .5 && gamepad2.right_trigger < .5){
                    robot.lift.setPower(0);
                }*/

                //if (gamepad2.right_bumper) {
                //    robot.tray.setPosition(.45);
                //}
                //else if (gamepad2.y){
                //    robot.tray.setPosition(.25);
                // }
                // else if (!gamepad2.right_bumper) {
                //    robot.tray.setPosition(.85);





                //}

            }
        }
    }


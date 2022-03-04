package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Bob", group="robot")

public class Bob extends LinearOpMode {

    Hardware20212022    robot = new Hardware20212022();

    public void Move (String direction,int time ) {
        if (direction == "forward") {
            robot.Bottomleftmotor.setPower(.1);
            robot.Topleftmotor.setPower(.1);
            robot.Toprightmotor.setPower(.1);
            robot.Bottomrightmotor.setPower(.1);
            sleep(time);
        }
        if (direction == "rghit") {
            robot.Bottomleftmotor.setPower(1);
            robot.Topleftmotor.setPower(1);
            robot.Toprightmotor.setPower(-1);
            robot.Bottomrightmotor.setPower(-1);
            sleep(350);
        }
    }


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);


        waitForStart();

        Move("forward",5000);
        Move("rghit",0);
        Move("forward",500);
    }
}

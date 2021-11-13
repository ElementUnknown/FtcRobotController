package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous (name = "Redside_Far_withoutspin", group = "robot")
public class Redside_Far_withoutspin extends LinearOpMode {
    Autonomous_Base2021_22  Auto = new Autonomous_Base2021_22();


    @Override
    public void runOpMode() {

        Auto.robot.init(hardwareMap);

        telemetry.addData("Status","Ready to run");
        telemetry.update();

        waitForStart();
        telemetry.addData("starting", "starting");
        telemetry.update();

        //Autonomous code starts here.
        Auto.Move(.5, 1,0);
        //Auto.dropBlock();
        Auto.Move(.5,0,4);
    }
}

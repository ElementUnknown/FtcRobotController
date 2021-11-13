package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name = "Blueside_Far_withoutspin", group = "robot")
public class Blueside_Far_withoutspin extends LinearOpMode{

    Autonomous_Base2021_22      Auto =  new Autonomous_Base2021_22();
    private ElapsedTime     runtime = new ElapsedTime();

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
        Auto.Move(.5,0,-4);
    }
}

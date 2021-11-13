package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "AHHHHHHHHHHHHHHH", group = "robot")
public class Test_Program extends LinearOpMode{

    Hardware20212022             robot = new Hardware20212022();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status","Ready to run");
        telemetry.update();

        waitForStart();
        telemetry.addData("starting", "starting");
        telemetry.update();

        //Autonomous code starts here.
        robot.Bottomleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Bottomrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Topleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Toprightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Bottomleftmotor.setTargetPosition(1000);
        robot.Bottomrightmotor.setTargetPosition(-1000);
        robot.Topleftmotor.setTargetPosition(-1000);
        robot.Toprightmotor.setTargetPosition(1000);

        robot.Bottomleftmotor.setPower(.3);
        robot.Bottomrightmotor.setPower(.3);
        robot.Topleftmotor.setPower(.3);
        robot.Toprightmotor.setPower(.3);

        robot.Bottomleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Topleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Toprightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Bottomrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.Bottomleftmotor.isBusy() && robot.Bottomrightmotor.isBusy() && robot.Topleftmotor.isBusy() && robot.Toprightmotor.isBusy()){
        }

        robot.Bottomrightmotor.setPower(0);
        robot.Toprightmotor.setPower(0);
        robot.Bottomleftmotor.setPower(0);
        robot.Topleftmotor.setPower(0);
    }
}

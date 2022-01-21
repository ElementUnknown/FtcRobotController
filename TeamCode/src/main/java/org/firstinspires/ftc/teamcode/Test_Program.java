package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous (name = "AHHHHHHHHHHHHHHH", group = "robot")
public class Test_Program extends LinearOpMode{

    Autonomous_Base2021_22  Auto = new Autonomous_Base2021_22();
    //Hardware20212022             robot = new Hardware20212022();
    //BNO055IMU imu;
   // Orientation angles;

    @Override
    public void runOpMode() {

        Auto.robot.init(hardwareMap);
        //robot.init(hardwareMap);

        telemetry.addData("Status","Ready to run");
        telemetry.update();

        waitForStart();
        telemetry.addData("starting", "starting");
        telemetry.update();

        //Autonomous code starts here.

        Auto.pickUpBlock();
        sleep(1000);
        Auto.dropBlock();

        //Gyro Sensor
        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        /*Auto.robot.Bottomleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Auto.robot.Bottomrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Auto.robot.Topleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Auto.robot.Toprightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Auto.robot.Bottomleftmotor.setTargetPosition(1000);
        Auto.robot.Bottomrightmotor.setTargetPosition(1000);
        Auto.robot.Topleftmotor.setTargetPosition(1000);
        Auto.robot.Toprightmotor.setTargetPosition(1000);

        Auto.robot.Bottomleftmotor.setPower(.3);
        Auto.robot.Bottomrightmotor.setPower(.3);
        Auto.robot.Topleftmotor.setPower(.3);

        Auto.robot.Toprightmotor.setPower(.3);

        Auto.robot.Bottomleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Auto.robot.Topleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Auto.robot.Toprightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Auto.robot.Bottomrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (Auto.robot.Bottomleftmotor.isBusy() && Auto.robot.Bottomrightmotor.isBusy() && Auto.robot.Topleftmotor.isBusy() && Auto.robot.Toprightmotor.isBusy()){
        }

        Auto.robot.Bottomrightmotor.setPower(0);
        Auto.robot.Toprightmotor.setPower(0);
        Auto.robot.Bottomleftmotor.setPower(0);
        Auto.robot.Topleftmotor.setPower(0);*/
    }
}

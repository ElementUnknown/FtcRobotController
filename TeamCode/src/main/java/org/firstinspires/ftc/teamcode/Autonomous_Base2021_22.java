package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Autonomous_Base2021_22", group = "robot")
@Disabled
public class Autonomous_Base2021_22  extends LinearOpMode{
    Hardware20212022             robot = new Hardware20212022();
    private ElapsedTime     runtime = new ElapsedTime();
    public double vertical_ticks_perinch = 43.956043956;
    public double horizontal_ticks_perinch = 56.7375886525;
    //public Orientation angles;
    //public Orientation angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);*/
    int Trueangle;


    public void Move (double power, double distanceforward, double distancelateral){
        if ( distanceforward != 0 && distancelateral == 0) {
            int Target_ticks = (int) (vertical_ticks_perinch * distanceforward);
            robot.Bottomleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Bottomrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Topleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Toprightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.Bottomleftmotor.setTargetPosition(-Target_ticks);
            robot.Bottomrightmotor.setTargetPosition(-Target_ticks);
            robot.Topleftmotor.setTargetPosition(-Target_ticks);
            robot.Toprightmotor.setTargetPosition(-Target_ticks);

            robot.Bottomleftmotor.setPower(-power);
            robot.Bottomrightmotor.setPower(-power);
            robot.Topleftmotor.setPower(-power);
            robot.Toprightmotor.setPower(-power);

            robot.Bottomleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Topleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Toprightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Bottomrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            waitforfinish();
            robot.Bottomrightmotor.setPower(0);
            robot.Toprightmotor.setPower(0);
            robot.Bottomleftmotor.setPower(0);
            robot.Topleftmotor.setPower(0);
        }
        if(distanceforward == 0 && distancelateral != 0);{
            int Target_ticks = (int) (horizontal_ticks_perinch * distancelateral);
            robot.Bottomleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Bottomrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Topleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Toprightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.Bottomleftmotor.setTargetPosition((int)(Target_ticks));
            robot.Bottomrightmotor.setTargetPosition((int)(-Target_ticks));
            robot.Topleftmotor.setTargetPosition(-Target_ticks);
            robot.Toprightmotor.setTargetPosition(Target_ticks);

            robot.Bottomleftmotor.setPower(power);
            robot.Bottomrightmotor.setPower(-power);
            robot.Topleftmotor.setPower(-power);
            robot.Toprightmotor.setPower(power);

            robot.Bottomleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Topleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Toprightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Bottomrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            waitforfinish();
            robot.Bottomrightmotor.setPower(0);
            robot.Toprightmotor.setPower(0);
            robot.Bottomleftmotor.setPower(0);
            robot.Topleftmotor.setPower(0);
        }
    }

    public void Turning (int time, double speed){
        robot.Bottomleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Bottomrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Topleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Toprightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.Bottomleftmotor.setPower(-speed);
        robot.Bottomrightmotor.setPower(speed);
        robot.Topleftmotor.setPower(-speed);
        robot.Toprightmotor.setPower(speed);
        sleep(time);
        robot.Bottomrightmotor.setPower(0);
        robot.Toprightmotor.setPower(0);
        robot.Bottomleftmotor.setPower(0);
        robot.Topleftmotor.setPower(0);
    }

    public void spinWheel (int time, double speed){
        robot.wheelspin.setPower(speed);
        sleep(time);
        robot.wheelspin.setPower(0);
    }

    public void rotateTurntable (int time, double speed){
        robot.turntable.setPower(speed);
        sleep(time);
        robot.turntable.setPower(0);
    }

    public void pickUpBlock () {
        robot.intake.setPower(-.5);
        sleep(75);
        robot.intake.setPower(0);
    }

    public void dropBlock () {
        robot.intake.setPower(.5);
        sleep(100);
        robot.intake.setPower(0);
    }

    public void controlarm (int time, double speed) {
        robot.arm.setPower(speed);
        sleep(time);
        robot.arm.setPower(0);
    }

    public void waitforfinish(){
        while (robot.Bottomleftmotor.isBusy() && robot.Bottomrightmotor.isBusy() && robot.Topleftmotor.isBusy() && robot.Toprightmotor.isBusy()){

        }
    }



       /* robot.Bottomrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Bottomleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Toprightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Topleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Toprightmotor.setTargetPosition(1000);
        robot.Topleftmotor.setTargetPosition(-1000);
        robot.Bottomrightmotor.setTargetPosition(-1000);
        robot.Bottomleftmotor.setTargetPosition(1000);

        robot.Bottomrightmotor.setPower(-.3);
        robot.Toprightmotor.setPower(-.3);
        robot.Bottomleftmotor.setPower(-.3);
        robot.Topleftmotor.setPower(-.3);

        robot.Toprightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Topleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Bottomrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Bottomleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       waitforfinish();

        robot.Bottomrightmotor.setPower(0);
        robot.Toprightmotor.setPower(0);
        robot.Bottomleftmotor.setPower(0);
        robot.Topleftmotor.setPower(0);*/

    @Override
    public void runOpMode()  {

    }
}

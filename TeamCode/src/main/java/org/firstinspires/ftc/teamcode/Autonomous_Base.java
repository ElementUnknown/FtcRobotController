package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Autonomous_Base2021_22", group = "robot")
@Disabled
public class Autonomous_Base extends LinearOpMode{
    HardwareMap             robot = new HardwareMap();
    private ElapsedTime     runtime = new ElapsedTime();
    public double vertical_ticks_perinch = 43.956043956;
    public double horizontal_ticks_perinch = 56.7375886525;
    double currentHeading;
    //BNO055IMU imu;
    //Orientation angles;
    boolean turn[] = new boolean[3];

    public void Move (double power, double distanceforward, double distancelateral){
        if ( distanceforward != 0 && distancelateral == 0) {
            int Target_ticks = (int) (vertical_ticks_perinch * distanceforward);
            robot.Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.Motor1.setTargetPosition(-Target_ticks);
            robot.Motor2.setTargetPosition(-Target_ticks);
            robot.Motor3.setTargetPosition(-Target_ticks);
            robot.Motor4.setTargetPosition(-Target_ticks);

            robot.Motor1.setPower(-power);
            robot.Motor2.setPower(-power);
            robot.Motor3.setPower(-power);
            robot.Motor4.setPower(-power);

            robot.Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            waitforfinish();
            robot.Motor1.setPower(0);
            robot.Motor2.setPower(0);
            robot.Motor3.setPower(0);
            robot.Motor4.setPower(0);
        }
        if(distanceforward == 0 && distancelateral != 0);{
            int Target_ticks = (int) (horizontal_ticks_perinch * distancelateral);
            robot.Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.Motor1.setTargetPosition(Target_ticks);
            robot.Motor2.setTargetPosition(-Target_ticks);
            robot.Motor3.setTargetPosition((int)(Target_ticks));
            robot.Motor4.setTargetPosition((int)(-Target_ticks));

            robot.Motor1.setPower(power);
            robot.Motor2.setPower(-power);
            robot.Motor3.setPower(power);
            robot.Motor4.setPower(-power);

            robot.Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            waitforfinish();
            robot.Motor1.setPower(0);
            robot.Motor2.setPower(0);
            robot.Motor3.setPower(0);
            robot.Motor4.setPower(0);
        }
    }

    public void Turning (int time, double speed){
        robot.Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.Motor3.setPower(-speed);
        robot.Motor4.setPower(speed);
        robot.Motor2.setPower(-speed);
        robot.Motor1.setPower(speed);
        sleep(time);
        robot.Motor1.setPower(0);
        robot.Motor2.setPower(0);
        robot.Motor3.setPower(0);
        robot.Motor4.setPower(0);
    }

    public void waitforfinish(){
        while (robot.Motor1.isBusy() && robot.Motor2.isBusy() && robot.Motor3.isBusy() && robot.Motor4.isBusy()){

        }
    }

    /*public void gyroTurn(int turnNumber, in t turnWindow1, int turnWindow2) {
        turn[turnNumber] = false;
        while (turn[turnNumber] == false) {
            robot.Motor1.setPower(-.3);
            robot.Motor4.setPower(-.3);
            robot.Motor2.setPower(.3);
            robot.Motor3.setPower(.3);
            if (currentHeading > turnWindow1 && currentHeading < turnWindow2) {
                turn[turnNumber] = true;
            } else {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                this.imu.getPosition();
                currentHeading = angles.firstAngle;
                telemetry.addData("Heading", angles.firstAngle);
                telemetry.update();
            }
        }
    }*/

    public void raiseArm(int time, double speed) {
        robot.LiftarmL.setPower(speed);
        robot.LiftarmR.setPower(speed);
        sleep(time);
        robot.LiftarmR.setPower(0);
        robot.LiftarmR.setPower(0);
    }

    public void lowerArm(int time, double speed) {
        robot.LiftarmL.setPower(-speed);
        robot.LiftarmR.setPower(-speed);
        sleep(time);
        robot.LiftarmR.setPower(0);
        robot.LiftarmR.setPower(0);
    }

    @Override
    public void runOpMode()  {

    }
}

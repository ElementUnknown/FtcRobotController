package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Autonomous_Base extends LinearOpMode{
    HardwareMap             robot = new HardwareMap();
    private ElapsedTime     runtime = new ElapsedTime();
    public double vertical_ticks_perinch = 44.0771349;
    public double horizontal_ticks_perinch = 50.7936507;
    private int color = 0;
    private int checkNum = 0;
    double currentHeading;
    //BNO055IMU imu;
    //Orientation angles;
    boolean turn[] = new boolean[3];


    public void Move (double power, double distanceforward, double distancelateral){
        robot.Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

            robot.Motor1.setTargetPosition((int)(Target_ticks));
            robot.Motor2.setTargetPosition((int)(-Target_ticks));
            robot.Motor3.setTargetPosition((int)(-Target_ticks));
            robot.Motor4.setTargetPosition((int)(Target_ticks));

            robot.Motor1.setPower(power);
            robot.Motor2.setPower(-power);
            robot.Motor3.setPower(-power);
            robot.Motor4.setPower(power);

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

    public void verticalMove (int time, double speed) {
        robot.Motor1.setPower(speed);
        robot.Motor2.setPower(speed);
        robot.Motor3.setPower(speed);
        robot.Motor4.setPower(speed);
        sleep(time);
        robot.Motor1.setPower(0);
        robot.Motor2.setPower(0);
        robot.Motor3.setPower(0);
        robot.Motor4.setPower(0);
    }

    public void horizontalMove (int time, double speed) {
        robot.Motor1.setPower(-speed);
        robot.Motor2.setPower(speed);
        robot.Motor3.setPower(speed);
        robot.Motor4.setPower(-speed);
        sleep(time);
        robot.Motor1.setPower(0);
        robot.Motor2.setPower(0);
        robot.Motor3.setPower(0);
        robot.Motor4.setPower(0);
    }

    public void Turning (int time, double speed){
        robot.Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.Motor1.setPower(-speed);
        robot.Motor2.setPower(speed);
        robot.Motor3.setPower(-speed);
        robot.Motor4.setPower(speed);
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
    public void waitforarmfinish(){
        while (robot.liftArmL.isBusy() && robot.liftArmR.isBusy()){

        }
    }
    public void ArmGround (int TicksR, int TicksL){

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftArmL.setPower(1);
        robot.liftArmR.setPower(1);

        robot.liftArmL.setTargetPosition(0 - TicksL);
        robot.liftArmR.setTargetPosition(0 - TicksR);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void Lowgoal(int TicksR, int TicksL){

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftArmL.setPower(1);
        robot.liftArmR.setPower(1);

        robot.liftArmL.setTargetPosition(500 - TicksL);
        robot.liftArmR.setTargetPosition(500 - TicksR);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void Goalreadjust(){

        int LiftL = robot.liftArmL.getCurrentPosition();
        int LiftR = robot.liftArmR.getCurrentPosition();
        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftArmL.setPower(.75);
        robot.liftArmR.setPower(.75);

        robot.liftArmL.setTargetPosition(LiftL-200);
        robot.liftArmR.setTargetPosition(LiftR-200);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void Medgoal(int TicksR, int TicksL){

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftArmL.setPower(1);
        robot.liftArmR.setPower(1);

        robot.liftArmL.setTargetPosition(760 -TicksL);
        robot.liftArmR.setTargetPosition(760 - TicksR);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void Highgoal(int TicksR, int TicksL){

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftArmL.setPower(1);
        robot.liftArmR.setPower(1);

        robot.liftArmL.setTargetPosition(1100 - TicksL);
        robot.liftArmR.setTargetPosition(1100 - TicksR);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void Grabconeheight(){

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftArmL.setPower(.5);
        robot.liftArmR.setPower(.5);

        robot.liftArmL.setTargetPosition(185);
        robot.liftArmR.setTargetPosition(185);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

   public void MoveArm(int time, double speed) {
       robot.liftArmL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       robot.liftArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftArmL.setPower(speed);
        robot.liftArmR.setPower(speed);
        sleep(time);
        robot.liftArmL.setPower(0);
        robot.liftArmR.setPower(0);
   }

   public void checkColor() {
       while (color != 6 && color != 9 && color != 10 && checkNum < 5) {
           color = robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
           checkNum++;
           sleep(250);
       }
   }

   public void releaseClaw() {
       robot.Claw.setPosition(.83);
       sleep(500);
   }

   public void closeClaw() {
        robot.Claw.setPosition(.4);
        sleep(500);
   }

   @Override
   public void runOpMode()  {

   }
}

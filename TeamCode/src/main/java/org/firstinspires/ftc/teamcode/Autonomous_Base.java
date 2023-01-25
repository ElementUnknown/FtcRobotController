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
    public double intoffset = 0;
    private int color = 0;
    private int checkNum = 0;


    boolean turn[] = new boolean[3];


    public void Move (double power, double distanceforward, double distancelateral /*, FinalTurnSpeed*/){
        double InitHeading = getHeading();
        double AngleDistance = 0;
        robot.Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*if ( distanceforward != 0 && distancelateral == 0) {
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
        }*/
        /*else if(distanceforward == 0 && distancelateral != 0){
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
            robot.Motor2.setPower(power);
            robot.Motor3.setPower(power);
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
        }*/
        //diagnol movement statment (if it works it can replace the entire statment of movement
            robot.Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            int Target_ticks_Vertical = (int) -(vertical_ticks_perinch * distanceforward);
            int Target_tick_Horizontal = (int) -(horizontal_ticks_perinch * distancelateral);
            robot.Motor1.setTargetPosition((int)(Target_ticks_Vertical - Target_tick_Horizontal));
            robot.Motor2.setTargetPosition((int)(Target_ticks_Vertical + Target_tick_Horizontal));
            robot.Motor3.setTargetPosition((int)(Target_ticks_Vertical + Target_tick_Horizontal));
            robot.Motor4.setTargetPosition((int)(Target_ticks_Vertical - Target_tick_Horizontal));

            robot.Motor1.setPower(Math.abs(power * ((distanceforward - distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral)))));
            robot.Motor2.setPower(Math.abs(power * ((distanceforward + distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral)))));
            robot.Motor3.setPower(Math.abs(power * ((distanceforward + distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral)))));
            robot.Motor4.setPower(Math.abs(power * ((distanceforward - distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral)))));

            robot.Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            StraightWait(InitHeading);
            robot.Motor1.setPower(0);
            robot.Motor2.setPower(0);
            robot.Motor3.setPower(0);
            robot.Motor4.setPower(0);
        // Final Correction values for Gyro Turn
        /*AngleDistance = getHeading() - InitHeading;
        if (AngleDistance > 180)            AngleDistance = AngleDistance - 360;
        if (AngleDistance < -180)           AngleDistance = AngleDistance + 360;
        TurnByGyro(AngleDistance, .3,2,5);*/
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

        boolean continueloop = true;
        while (robot.Motor1.isBusy() && robot.Motor2.isBusy() && robot.Motor3.isBusy() && robot.Motor4.isBusy()){

        }
    }
    public void StraightWait(double InitHeading){ //To be Tested
        double Motor1PowerPrime = robot.Motor1.getPower();
        double Motor2PowerPrime = robot.Motor2.getPower();
        double Motor3PowerPrime = robot.Motor3.getPower();
        double Motor4PowerPrime = robot.Motor4.getPower();
        double Motor1Power = 0;
        double Motor2Power = 0;
        double Motor3Power = 0;
        double Motor4Power = 0;
        double AngleDistance = 0;
        boolean continueloop = true;
        double TickDistance;
        while ((robot.Motor1.isBusy() && robot.Motor2.isBusy() && robot.Motor3.isBusy() && robot.Motor4.isBusy()) && continueloop){
            AngleDistance = -getHeading() + InitHeading;

            if (AngleDistance > 180)            AngleDistance = AngleDistance - 360;
            if (AngleDistance <= -180)           AngleDistance = AngleDistance + 360;
            if (AngleDistance != 0){

                Motor1Power = Motor1PowerPrime - (Math.signum(robot.Motor1.getTargetPosition()) * (AngleDistance / 360 ));
                Motor2Power = Motor2PowerPrime + (Math.signum(robot.Motor2.getTargetPosition()) * (AngleDistance / 360 ));
                Motor3Power = Motor3PowerPrime - (Math.signum(robot.Motor3.getTargetPosition()) * (AngleDistance / 360 ));
                Motor4Power = Motor4PowerPrime + (Math.signum(robot.Motor4.getTargetPosition()) * (AngleDistance / 360 ));


                robot.Motor1.setPower(Motor1Power);
                robot.Motor2.setPower(Motor2Power);
                robot.Motor3.setPower(Motor3Power);
                robot.Motor4.setPower(Motor4Power);

                telemetry.addData("M1",String.valueOf(robot.Motor1.getPower()));
                telemetry.addData("M2",String.valueOf(robot.Motor2.getPower()));
                telemetry.addData("M3",String.valueOf(robot.Motor3.getPower()));
                telemetry.addData("M4",String.valueOf(robot.Motor4.getPower()));
                telemetry.update();
            }
            TickDistance = Math.abs((robot.Motor1.getTargetPosition() - robot.Motor1.getCurrentPosition())) + Math.abs((robot.Motor2.getTargetPosition() - robot.Motor2.getCurrentPosition())) + Math.abs((robot.Motor3.getTargetPosition() - robot.Motor3.getCurrentPosition())) + Math.abs((robot.Motor4.getTargetPosition() - robot.Motor4.getCurrentPosition()));
            if (Math.abs(TickDistance) > 200)      continueloop = true;

            else if (Math.abs(TickDistance) < 200) continueloop = false;

            telemetry.addData("",String.valueOf(InitHeading));
            telemetry.update();
        }
        robot.Motor1.setPower(Motor1Power);
        robot.Motor2.setPower(Motor2Power);
        robot.Motor3.setPower(Motor3Power);
        robot.Motor4.setPower(Motor4Power);
        waitforfinish();
    }
    /*robot.Motor1.setPower( Motor1Power - (AngleDistance / 360 ));
                robot.Motor2.setPower( Motor2Power + (AngleDistance / 360 ));
                robot.Motor3.setPower( Motor3Power - (AngleDistance / 360 ));
                robot.Motor4.setPower( Motor4Power + (AngleDistance / 360 ));*/
    public void waitforarmfinish(){
        while (robot.liftArmL.isBusy() || robot.liftArmR.isBusy()){

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

        robot.liftArmL.setTargetPosition(LiftL-150);
        robot.liftArmR.setTargetPosition(LiftR-150);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void raisearmoffgoal(){

        int LiftL = robot.liftArmL.getCurrentPosition();
        int LiftR = robot.liftArmR.getCurrentPosition();
        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftArmL.setPower(.75);
        robot.liftArmR.setPower(.75);

        robot.liftArmL.setTargetPosition(LiftL+150);
        robot.liftArmR.setTargetPosition(LiftR+150);

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

        robot.liftArmL.setPower(.6);
        robot.liftArmR.setPower(.6);

        robot.liftArmL.setTargetPosition(185);
        robot.liftArmR.setTargetPosition(185);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
   public void TurnByGyro(double target, double speed, double buffer, double multplierquotiant){//quotiant is the degree from the target with which you want to begin decelaeration
        double TurnSpeed = 0; // if quotiant is too small, the angle may not be met, so the loop may get stuck
        double currentHeading;
        boolean continueangleloop = false;
        double truetarget;
        double offset = 0;
        double angledistance = 0;
        robot.Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentHeading = getHeading();
        offset = getHeading();
        truetarget = (-target + offset);
        while(truetarget > 180)             truetarget    = truetarget    - 360;
        while(truetarget < -180)            truetarget    = truetarget    + 360;
        angledistance = currentHeading - truetarget;
        if (angledistance > 180)            angledistance = angledistance - 360;
        if (angledistance < -180)           angledistance = angledistance + 360;
        if (Math.abs(angledistance) >= buffer)    continueangleloop = true;
        while (opModeIsActive() && continueangleloop){
            currentHeading = getHeading();
            angledistance = currentHeading - truetarget;
            if (angledistance > 180)            angledistance = angledistance - 360;
            if (angledistance < -180)           angledistance = angledistance + 360;
            TurnSpeed= angledistance / multplierquotiant;
            if (TurnSpeed > 1)                  TurnSpeed     = 1;
            if (TurnSpeed < -1)                  TurnSpeed     = -1;

            robot.Motor1.setPower(speed*TurnSpeed);
            robot.Motor2.setPower(-speed*TurnSpeed);
            robot.Motor3.setPower(speed*TurnSpeed);
            robot.Motor4.setPower(-speed*TurnSpeed);
            if (Math.abs(angledistance) > buffer)      continueangleloop = true;
            else                                        continueangleloop = false;
            telemetry.addData("", String.valueOf(currentHeading));
            telemetry.update();
        }
   }
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
       robot.Claw.setPosition(.5);
       sleep(500);
   }

   public void closeClaw() {
        robot.Claw.setPosition(.0);
        sleep(500);
   }
   public double getHeading() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = robot.angles.firstAngle;
       while(heading > 180)             heading    = heading  - 360;
       while(heading < -180)            heading    = heading    + 360;
       return heading;
    }
    public void EmergencyCorrectionForward(){
        telemetry.addData("FORWARD TILT CORRECTION", "");
        telemetry.update();
        robot.liftArmL.setPower(-1);
        robot.liftArmR.setPower(-1);
        robot.Motor1.setPower(.75);
        robot.Motor2.setPower(.75);
        robot.Motor3.setPower(.75);
        robot.Motor4.setPower(.75);
        sleep(350);
        robot.Motor1.setPower(0);
        robot.Motor2.setPower(0);
        robot.Motor3.setPower(0);
        robot.Motor4.setPower(0);
        sleep(500);
        robot.liftArmL.setPower(0);
        robot.liftArmR.setPower(0);
        robot.liftArmL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void EmergencyCorrectionBackwards(){
        telemetry.addData("BACKWARD TILT CORRECTION","");
        telemetry.update();
        robot.liftArmL.setPower(-1);
        robot.liftArmR.setPower(-1);
        robot.Motor1.setPower(-.75);
        robot.Motor2.setPower(-.75);
        robot.Motor3.setPower(-.75);
        robot.Motor4.setPower(-.75);
        sleep(350);
        robot.Motor1.setPower(0);
        robot.Motor2.setPower(0);
        robot.Motor3.setPower(0);
        robot.Motor4.setPower(0);
        sleep(500);
        robot.liftArmL.setPower(0);
        robot.liftArmR.setPower(0);
        robot.liftArmL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

   @Override
   public void runOpMode()  {

   }
}

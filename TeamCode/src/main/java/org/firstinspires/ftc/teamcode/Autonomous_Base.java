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

        //diagnol movement statment (if it works it can replace the entire statment of movement
        //it worked
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
    public void PIDMove (double INCHForward, double INCHRight, double speed, double angle, double angleinc){
        double Offset = getHeading();
        double KP = 0;
        double KD = 0;
        double KI = 0;
        double KGP= 0;
        double KGD = 0;
        double KGI = 0 ;
        double TickIncPerSec = 100;

        int TimedTicksForward = 0;
        int TimedTicksRight = 0;
        double TimedAngle = 0;

        int TicksForward = (int) (INCHForward * vertical_ticks_perinch);
        int TicksRight = (int) (INCHRight * horizontal_ticks_perinch);

        robot.Motor1.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor2.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor3.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor4.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        int M1Error = 0;
        int M2Error = 0;
        int M3Error = 0;
        int M4Error = 0;
        double GyroError = 0;

        int PreviousM1error = 0;
        int PreviousM2error = 0;
        int PreviousM3error = 0;
        int PreviousM4error = 0;
        double PreviousAngleError = 0;

        double DM1;
        double DM2;
        double DM3;
        double DM4;
        double DG;

        double IM1 = 0;
        double IM2 = 0;
        double IM3 = 0;
        double IM4 = 0;
        double IG = 0;

        ElapsedTime TicksTime = new ElapsedTime();
        ElapsedTime DITime = new ElapsedTime();

        angle = -angle + Offset;

        double GyroCorrection;
        while (true) {
            TimedTicksForward = (int) (Math.signum(TicksForward) * (Math.min(TicksTime.seconds() * TickIncPerSec, Math.abs(TicksForward))));
            TimedTicksRight = (int) (Math.signum(TicksRight) * (Math.min(TicksTime.seconds() * TickIncPerSec, Math.abs(TicksRight))));
            TimedAngle = (Math.signum(angle) * (Math.min(TicksTime.seconds() * angleinc, Math.abs(angle))));

            M1Error = (TimedTicksForward - TimedTicksRight) - robot.Motor1.getCurrentPosition();
            M2Error = (TimedTicksForward + TimedTicksRight) - robot.Motor2.getCurrentPosition();
            M3Error = (TimedTicksForward + TimedTicksRight) - robot.Motor3.getCurrentPosition();
            M4Error = (TimedTicksForward - TimedTicksRight) - robot.Motor4.getCurrentPosition();
            GyroError = TimedAngle - getHeading();

            DM1 = (M1Error - PreviousM1error) / DITime.seconds();
            DM2 = (M2Error - PreviousM2error) / DITime.seconds();
            DM3 = (M3Error - PreviousM3error) / DITime.seconds();
            DM4 = (M4Error - PreviousM4error) / DITime.seconds();
            DG = (GyroError - PreviousAngleError) / DITime.seconds();

            IM1 = IM1 + M1Error * DITime.seconds();
            IM2 = IM2 + M2Error * DITime.seconds();
            IM3 = IM3 + M3Error * DITime.seconds();
            IM4 = IM4 + M4Error * DITime.seconds();
            IG = IG + GyroError * DITime.seconds();

            GyroCorrection = ((GyroError * KGP) + (DG * KGD) + (IG * KGI));

            robot.Motor1.setPower((speed * ((M1Error * KP) + (DM1 * KD) + (IM1 * KI))) - (GyroCorrection));
            robot.Motor1.setPower((speed * ((M2Error * KP) + (DM2 * KD) + (IM2 * KI))) + (GyroCorrection));
            robot.Motor1.setPower((speed * ((M3Error * KP) + (DM3 * KD) + (IM3 * KI))) - (GyroCorrection));
            robot.Motor1.setPower((speed * ((M4Error * KP) + (DM4 * KD) + (IM4 * KI))) + (GyroCorrection));

            DITime.reset();
            PreviousM1error = M1Error;
            PreviousM2error = M2Error;
            PreviousM3error = M3Error;
            PreviousM4error = M4Error;
            PreviousAngleError = GyroError;
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
        int KI = 0;
        int KP = 0;
        int KD = 0;
        double AngleInc = 90;
        double LastGyroError = 0;
        double P = 0;
        double I = 0;
        double D = 0;
        double TimeTarget = 0;
        ElapsedTime NotReset = new ElapsedTime();
        ElapsedTime Reset = new ElapsedTime();
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
            TimeTarget = Math.signum(truetarget) * Math.min(AngleInc * NotReset.seconds(), Math.abs(truetarget));
            angledistance = currentHeading - TimeTarget;

            if (angledistance > 180)            angledistance = angledistance - 360;
            if (angledistance < -180)           angledistance = angledistance + 360;

            P = angledistance * KP;
            D = (angledistance - LastGyroError) / Reset.seconds();
            I = I + (angledistance * Reset.seconds());

            LastGyroError = angledistance;

            robot.Motor1.setPower(speed * ((P) + (I * KI) + (D * KD)));
            robot.Motor2.setPower(-speed * ((P) + (I * KI) + (D * KD)));
            robot.Motor3.setPower(speed * ((P) + (I * KI) + (D * KD)));
            robot.Motor4.setPower(-speed* ((P) + (I * KI) + (D * KD)));
            if (Math.abs(angledistance) >= buffer)      continueangleloop = true;
            else                                        continueangleloop = false;
            telemetry.addData("", String.valueOf(currentHeading));
            telemetry.update();
            Reset.reset();
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

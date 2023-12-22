package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.*;
import java.util.concurrent.TimeUnit;


public class Autonomous_Base extends LinearOpMode {
    HardwareMap robot = new HardwareMap();
    private ElapsedTime runtime = new ElapsedTime();
    public final double vertical_ticks_perinch = 44.0771349;
    public final double horizontal_ticks_perinch = 50.7936507;
    public boolean spikeFound = false;


    boolean turn[] = new boolean[3];


    public void Move(double power, double distanceforward, double distancelateral) {
        double InitHeading = getHeading();
        ElapsedTime Time = new ElapsedTime();
        double M1Speed;
        double M2Speed;
        double M3Speed;
        double M4Speed;
        //Speed Variables are used to calculate the needed adjustments for straight line
        robot.Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Target_ticks_Vertical = (int) -(vertical_ticks_perinch * distanceforward);
        int Target_tick_Horizontal = (int) -(horizontal_ticks_perinch * distancelateral);
        //calculate the error
        robot.Motor1.setTargetPosition((Target_ticks_Vertical - Target_tick_Horizontal));
        robot.Motor2.setTargetPosition((Target_ticks_Vertical + Target_tick_Horizontal));
        robot.Motor3.setTargetPosition((Target_ticks_Vertical + Target_tick_Horizontal));
        robot.Motor4.setTargetPosition((Target_ticks_Vertical - Target_tick_Horizontal));
        //robot.Motor1.setPower(power);
        //robot.Motor2.setPower(power);
        //robot.Motor3.setPower(power);
        //robot.Motor4.setPower(power);
        M1Speed = (Math.abs(power * ((distanceforward - distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral)))));
        M2Speed = (Math.abs(power * ((distanceforward + distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral)))));
        M3Speed = (Math.abs(power * ((distanceforward + distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral)))));
        M4Speed = (Math.abs(power * ((distanceforward - distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral)))));
        //Calculating motor speeds based off of imputed power and target averages (basically a percent error calculation
        robot.Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set motors to run
        //waitforfinish();
        StraightWait(InitHeading, power);
        //go to straight wait with the heading set at the beginning of method, and motor speeds
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
    public void StraightWait(double InitHeading, double power) { //To be Tested
        ElapsedTime Time = new ElapsedTime();
        double Motor1Power = 0;
        double Motor2Power = 0;
        double Motor3Power = 0;
        double Motor4Power = 0;
        double AngleDistance = 0;
        double[] kp = new double[4];
        boolean continueloop = true;
        double TickDistance;
        //I don't even know how this works, I drank coffee and then the next day this was here
        while ((robot.Motor1.isBusy() && robot.Motor2.isBusy() && robot.Motor3.isBusy() && robot.Motor4.isBusy()) && continueloop) {
            AngleDistance = getHeading() - InitHeading;

            double accceleratioon_ctl = Math.min(Time.seconds() * 2, 1);
            if (AngleDistance > 180) AngleDistance = AngleDistance - 360;
            if (AngleDistance <= -180) AngleDistance = AngleDistance + 360;

            kp[0] = (robot.Motor1.getTargetPosition() - robot.Motor1.getCurrentPosition()) / 850.0;
            kp[1] = (robot.Motor2.getTargetPosition() - robot.Motor2.getCurrentPosition()) / 850.0;
            kp[2] = (robot.Motor3.getTargetPosition() - robot.Motor3.getCurrentPosition()) / 850.0;
            kp[3] = (robot.Motor4.getTargetPosition() - robot.Motor4.getCurrentPosition()) / 850.0;
            kp[0] = Math.max(Math.min(kp[0], 1) , -1);
            kp[1] = Math.max(Math.min(Math.abs(kp[1]), 1), -1);
            kp[2] = Math.max(Math.min(Math.abs(kp[2]), 1), -1);
            kp[3] = Math.max(Math.min(Math.abs(kp[3]), 1) , -1);

            Motor1Power = power * accceleratioon_ctl * (kp[0] - (Math.signum(robot.Motor1.getTargetPosition()) * (AngleDistance / 270)));
            Motor2Power = power * accceleratioon_ctl * (kp[1] + (Math.signum(robot.Motor2.getTargetPosition()) * (AngleDistance / 270)));
            Motor3Power = power * accceleratioon_ctl * (kp[2] - (Math.signum(robot.Motor3.getTargetPosition()) * (AngleDistance / 270)));
            Motor4Power = power * accceleratioon_ctl * (kp[3] + (Math.signum(robot.Motor4.getTargetPosition()) * (AngleDistance / 270)));


            robot.Motor1.setPower(Motor1Power);
            robot.Motor2.setPower(Motor2Power);
            robot.Motor3.setPower(Motor3Power);
            robot.Motor4.setPower(Motor4Power);

            telemetry.addData("M1", kp[0]);
            telemetry.addData("M2", kp[1]);
            telemetry.addData("M3", kp[2]);
            telemetry.addData("M4", kp[3]);
            telemetry.update();

            TickDistance = Math.abs((robot.Motor1.getTargetPosition() - robot.Motor1.getCurrentPosition())) + Math.abs((robot.Motor2.getTargetPosition() - robot.Motor2.getCurrentPosition())) + Math.abs((robot.Motor3.getTargetPosition() - robot.Motor3.getCurrentPosition())) + Math.abs((robot.Motor4.getTargetPosition() - robot.Motor4.getCurrentPosition()));
            //All I know is that this really long line of code just calculates the total error between the four motors
            continueloop = (Math.abs(TickDistance) >= 200);


        }

    }
    public void CentricMove(double Power, double Y,double X){
        robot.Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        X *= horizontal_ticks_perinch;
        Y *= vertical_ticks_perinch;
        double AngleM = Math.toDegrees(Math.atan2(X,Y));
        boolean ContLoop = true;
        double TotDis;
        double TrueAngle;
        double PX;
        double PY;
        int PM1 = robot.Motor1.getCurrentPosition(), PM2= robot.Motor2.getCurrentPosition(),
                PM3 = robot.Motor3.getCurrentPosition(), PM4 = robot.Motor4.getCurrentPosition();
        int dTY, dTX, DTDistance;
        while(ContLoop && opModeIsActive()){
            int DTM1 = robot.Motor1.getCurrentPosition() - PM1; //find change in ticks of each motor
            int DTM2 = robot.Motor2.getCurrentPosition() - PM2;
            int DTM3 = robot.Motor3.getCurrentPosition() - PM3;
            int DTM4 = robot.Motor4.getCurrentPosition() - PM4;
            TotDis = Math.sqrt(Math.pow(Y,2) + Math.pow(X,2)); //Gets updated every loop
            TrueAngle = AngleM - (getHeading() - robot.initAngle);
            PX = (Math.sin(TrueAngle) * TotDis) / TotDis;
            PY = (Math.cos(TrueAngle) * TotDis) / TotDis;

            robot.Motor1.setPower((PY - PX) * Power);
            robot.Motor2.setPower((PY + PX) * Power);
            robot.Motor3.setPower((PY + PX) * Power);
            robot.Motor4.setPower((PY - PX) * Power);

            dTY = DTM1 + DTM2 + DTM3 + DTM4; // This might work it might not work, might need to be a more complicated distance measure
            dTX = -DTM1 + DTM2 + DTM3 - DTM4; // Completly based off of the power calcs
            /** We may have to take in account frictions as a change in 1 motor without changes of any other
             * so we may just divide by 4
             * I dont know,this is a question to ask the internet / Lucas**/
            Y =- dTY;
            X =- dTX;
            ContLoop =(TotDis > 100);
            PM1 = robot.Motor1.getCurrentPosition(); // set previous ticks of motors to the current measure
            PM2 = robot.Motor2.getCurrentPosition();
            PM3 = robot.Motor3.getCurrentPosition();
            PM4 = robot.Motor4.getCurrentPosition();
        }
    }
    public void PIDMove(double INCHForward, double INCHRight, double speed, double angle, double angleinc) {
        double Offset = getHeading();
        boolean Runloop = true;
        double KP = .0015;
        double KD = .00045;
        double KI = .0004;
        double KGP = .2;
        double KGD = 0;
        double KGI = 0;

        float TimedTicksForward = 0;
        float TimedTicksRight = 0;
        double TimedAngle = 0;

        int TicksForward = (int) (-INCHForward * vertical_ticks_perinch);
        int TicksRight = (int) (-INCHRight * horizontal_ticks_perinch);

        robot.Motor1.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor2.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor3.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor4.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor1.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        robot.Motor2.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        robot.Motor3.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        robot.Motor4.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        float M1Error = 0;
        float M2Error = 0;
        float M3Error = 0;
        float M4Error = 0;
        double GyroError = 0;

        float PreviousM1error = 0;
        float PreviousM2error = 0;
        float PreviousM3error = 0;
        float PreviousM4error = 0;
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

        double TotalPowerM1 = 0;
        double TotalPowerM2 = 0;
        double TotalPowerM3 = 0;
        double TotalPowerM4 = 0;

        int loop = 0;
        ElapsedTime TicksTime = new ElapsedTime();
        ElapsedTime DITime = new ElapsedTime();

        angle = -angle + Offset;
        if (angle > 180) angle = angle - 360;
        if (angle < -180) angle = angle + 360;
        double GyroCorrection = 0;
        double TrueMax = 0;
        double Time = 0;
        while (Runloop && opModeIsActive()) {
            Time = DITime.milliseconds();
            TimedTicksForward = (float) (Math.signum(TicksForward) * (Math.min(TicksTime.seconds() * Math.abs(TicksForward) * 1, Math.abs(TicksForward))));
            TimedTicksRight = (float) (Math.signum(TicksRight) * (Math.min(TicksTime.seconds() * Math.abs(TicksRight) * 1, Math.abs(TicksRight))));
            TimedAngle = (Math.signum(angle) * (Math.min(TicksTime.seconds() * angleinc, Math.abs(angle))));

            if (Math.abs(TimedTicksForward) < 6)
                TimedTicksForward = 10 * Math.signum((TimedTicksForward));

            M1Error = (TimedTicksForward - TimedTicksRight) - robot.Motor1.getCurrentPosition();
            M2Error = (TimedTicksForward + TimedTicksRight) - robot.Motor2.getCurrentPosition();
            M3Error = (TimedTicksForward + TimedTicksRight) - robot.Motor3.getCurrentPosition();
            M4Error = (TimedTicksForward - TimedTicksRight) - robot.Motor4.getCurrentPosition();
            GyroError = TimedAngle + getHeading();
            GyroError = 0;

            if (GyroError > 180) GyroError = GyroError - 360;
            if (GyroError < -180) GyroError = GyroError + 360;

            DM1 = (M1Error - PreviousM1error) / Time;
            DM2 = (M2Error - PreviousM2error) / Time;
            DM3 = (M3Error - PreviousM3error) / Time;
            DM4 = (M4Error - PreviousM4error) / Time;
            DG = (GyroError - PreviousAngleError) / Time;

            IM1 = IM1 + (M1Error * Time);
            IM2 = IM2 + (M2Error * Time);
            IM3 = IM3 + (M3Error * Time);
            IM4 = IM4 + (M4Error * Time);
            IG = IG + (GyroError * Time);

            //GyroCorrection = ((GyroError * KGP) + (DG * KGD) + (IG * KGI));
            telemetry.addData("M1Error", String.valueOf(M1Error));
            telemetry.addData("M3Error", String.valueOf(M2Error));
            telemetry.addData("M3Error", String.valueOf(M3Error));
            telemetry.addData("M4Error", String.valueOf(M4Error));
            telemetry.addData("Target", String.valueOf(TimedTicksForward));
            telemetry.addData("D", String.valueOf(DM1));
            telemetry.addData("I", String.valueOf(IM1));
            telemetry.addData("KP", String.valueOf(KP));
            telemetry.addData("KD", String.valueOf(KD));
            telemetry.addData("KI", String.valueOf(KI));
            telemetry.addData("Total", String.valueOf(TotalPowerM1));
            telemetry.addData("Max", String.valueOf(TrueMax));
            telemetry.addData("M1", robot.Motor1.getPower());
            telemetry.addData("M2", robot.Motor2.getPower());
            telemetry.addData("M3", robot.Motor3.getPower());
            telemetry.addData("M4", robot.Motor4.getPower());

            telemetry.update();

            TotalPowerM1 = ((((M1Error * KP) + (DM1 * KD) + (IM1 * KI))) - (GyroCorrection));
            TotalPowerM2 = ((((M2Error * KP) + (DM2 * KD) + (IM2 * KI))) - (GyroCorrection));
            TotalPowerM3 = ((((M3Error * KP) + (DM3 * KD) + (IM3 * KI))) - (GyroCorrection));
            TotalPowerM4 = ((((M4Error * KP) + (DM4 * KD) + (IM4 * KI))) - (GyroCorrection));

            TrueMax = Math.max(Math.max(Math.abs(TotalPowerM3), Math.abs(TotalPowerM4)), Math.max(Math.abs(TotalPowerM1), Math.abs(TotalPowerM2)));

            if (Math.abs(TrueMax) > 1) {
                TotalPowerM1 = TotalPowerM1 / Math.abs(TrueMax);
                TotalPowerM2 = TotalPowerM2 / Math.abs(TrueMax);
                TotalPowerM3 = TotalPowerM3 / Math.abs(TrueMax);
                TotalPowerM4 = TotalPowerM4 / Math.abs(TrueMax);

            }
            robot.Motor4.setPower(speed * TotalPowerM4);
            robot.Motor3.setPower(speed * TotalPowerM3);
            robot.Motor2.setPower(speed * TotalPowerM2);
            robot.Motor1.setPower(speed * TotalPowerM1);

            DITime.reset();
            PreviousM1error = M1Error;
            PreviousM2error = M2Error;
            PreviousM3error = M3Error;
            PreviousM4error = M4Error;
            PreviousAngleError = GyroError;

            if ((Math.abs(M1Error) < 10) && (Math.abs(M2Error) < 10) && (Math.abs(M3Error) < 10) && (Math.abs(M4Error) < 10) && (Math.abs(GyroError) < 1))
                Runloop = false;
            loop += 1;
        }
        robot.Motor1.setPower(0);
        robot.Motor2.setPower(0);
        robot.Motor3.setPower(0);
        robot.Motor4.setPower(0);
        telemetry.addData("loop", String.valueOf(loop));
        telemetry.addData("Time", String.valueOf(TicksTime.seconds()));
        telemetry.update();
        sleep((5000));
    }

    public void verticalMove(int time, double speed) {
        robot.Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    public void horizontalMove(int time, double speed) {
        robot.Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void Turning(int time, double speed) {
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

    public void waitforfinish() {

        while (robot.Motor1.isBusy() || robot.Motor2.isBusy() || robot.Motor3.isBusy() || robot.Motor4.isBusy()) {
            telemetry.addData("Motor 1 Position", robot.Motor1.getCurrentPosition());
            telemetry.addData("Motor 2 Position", robot.Motor2.getCurrentPosition());
            telemetry.addData("Motor 3 Position", robot.Motor3.getCurrentPosition());
            telemetry.addData("Motor 4 Position", robot.Motor4.getCurrentPosition());
            telemetry.update();
        }
    }




    public void TurnByGyro(double target, double speed, double buffer) {//quotiant is the degree from the target with which you want to begin decelaeration
        double TurnSpeed = 0; // if quotiant is too small, the angle may not be met, so the loop may get stuck
        double currentHeading;
        boolean continueangleloop = false;
        double truetarget;
        double offset = 0;
        double angledistance = 0;
        //int KI = 0;
        double KP = 1.0/70.0;
        //int KD = 0;
        double AngleInc = 90;
        //double LastGyroError = 0;
        double P = 0;
        //double I = 0;
        //double D = 0;
        double TimeTarget = 0;
        ElapsedTime NotReset = new ElapsedTime();

        robot.Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentHeading = getHeading();
        offset = getHeading();
        truetarget = offset + target;
        while (truetarget > 180) truetarget = truetarget - 360;
        while (truetarget < -180) truetarget = truetarget + 360;
        angledistance = currentHeading - truetarget;
        if (angledistance > 180) angledistance = angledistance - 360;
        if (angledistance < -180) angledistance = angledistance + 360;
        continueangleloop = (Math.abs(angledistance) >= buffer);
        while (opModeIsActive() && continueangleloop) {
            currentHeading = getHeading();
            //TimeTarget = Math.signum(truetarget) * Math.min(AngleInc * NotReset.seconds(), Math.abs(truetarget));
            angledistance = -currentHeading + truetarget;

            if (angledistance > 180) angledistance = angledistance - 360;
            if (angledistance < -180) angledistance = angledistance + 360;

            P = Math.abs(angledistance) * KP;
            P = Math.min(P,1);
            P = Math.max(P,.3);
           // D = (angledistance - LastGyroError) / Reset.seconds();
            //I = I + (angledistance * Reset.seconds());

            //LastGyroError = angledistance;

            robot.Motor1.setPower(speed * (P));
            robot.Motor2.setPower(-speed * (P));
            robot.Motor3.setPower(speed * (P));
            robot.Motor4.setPower(-speed * (P));
             continueangleloop = (Math.abs(angledistance) >= buffer);
            telemetry.addData("Heading", String.valueOf(currentHeading));
            telemetry.addData("Error", String.valueOf(angledistance));
            telemetry.addData("TT", truetarget);
            telemetry.update();
        }
        robot.Motor1.setPower(0);
        robot.Motor2.setPower(0);
        robot.Motor3.setPower(0);
        robot.Motor4.setPower(0);
    }

    public void MoveArm(int time, double speed) {
        robot.liftArm.setPower(speed);
        sleep(time);
        robot.liftArm.setPower(0);
    }

    public void releaseClawL() {
        robot.clawL.setPosition(.2);
        sleep(500);
    }

    public void closeClawL() {
        robot.clawL.setPosition(0);
        sleep(500);
    }
    public void releaseClawR() {
        robot.clawR.setPosition(.6);
        sleep(500);
    }

    public void closeClawR() {
        robot.clawR.setPosition(.8);
        sleep(500);
    }

    public double getHeading() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = -robot.angles.firstAngle;
        while (heading > 180) heading = heading - 360;
        while (heading < -180) heading = heading + 360;
        return heading;
    }


    public void PivotTick(int target, double power){

        robot.PivotArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.PivotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.PivotArm.setTargetPosition(-target);
        robot.PivotArm.setPower(power);

    }
    public void PivotWaitFinish(){
        while(robot.PivotArm.isBusy()){

        }
        robot.PivotArm.setPower(0);

    }
    public void EmergencyCorrectionForward(){
        telemetry.addData("FORWARD TILT CORRECTION", "");
        telemetry.update();
        //robot.liftArmL.setPower(-1);
        //robot.liftArmR.setPower(-1);
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
        /*robot.liftArmL.setPower(0);
        robot.liftArmR.setPower(0);
        robot.liftArmL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
    }
    public void EmergencyCorrectionBackwards(){
        telemetry.addData("BACKWARD TILT CORRECTION","");
        telemetry.update();
        //robot.liftArmL.setPower(-1);
        //robot.liftArmR.setPower(-1);
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
        /*robot.liftArmL.setPower(0);
        robot.liftArmR.setPower(0);
        robot.liftArmL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
    }
    public int turnIntervalScan(double Radius, int Direction){
        ElapsedTime FoundTime = new ElapsedTime();

        TurnByGyro(60*-Direction, .7,6);
        double target = 10 * Direction;
        double KP = 1.0/10.0;
        double P;
        boolean ContLoop = true;
        int returned = 1;


        boolean Found = false;
        for (int i =0; i < 3; i++){
            double initAngle = getHeading();
            returned = i;
            double AngleDistance = target;
            while(AngleDistance > 5 && ContLoop) {


                AngleDistance = initAngle + target - getHeading();
                if (AngleDistance > 180) AngleDistance = AngleDistance - 360;
                if (AngleDistance < -180) AngleDistance = AngleDistance + 360;

                Found = (getDistance() > Radius);
                P = AngleDistance * KP;


                robot.Motor1.setPower(.3 * (P));
                robot.Motor2.setPower(-.3 * (P));
                robot.Motor3.setPower(.3 * (P));
                robot.Motor4.setPower(-.3 * (P));
                if (Found) {
                    if(FoundTime.milliseconds() > 150){
                        i =300;
                        ContLoop=false;
                    }
                }
                else FoundTime.reset();
            }

            TurnByGyro(45 * Direction,.7,5);
        }
        return returned;
    }
    public int turnScan (double Radius, int Direction){


        TurnByGyro(60 * -Direction,.7,5);
        double Target = 180*Direction;
        double initAngle = getHeading();
        double AngleDistance = initAngle + Target- getHeading();
        double KP = 1.0/90.0;
        double P;
        double AngleTraveled;
        boolean continueloop = true;

        if (AngleDistance > 180) AngleDistance = AngleDistance - 360;
        if (AngleDistance < -180) AngleDistance = AngleDistance + 360;
        while (AngleDistance > 5 && continueloop){


            AngleDistance = initAngle + Target - getHeading();
            if (AngleDistance > 180) AngleDistance = AngleDistance - 360;
            if (AngleDistance < -180) AngleDistance = AngleDistance + 360;
            continueloop = (getDistance() > Radius);
            P = AngleDistance * KP;


            robot.Motor1.setPower(.3 * (P));
            robot.Motor2.setPower(-.3 * (P));
            robot.Motor3.setPower(.3 * (P));
            robot.Motor4.setPower(-.3 * (P));
            telemetry.addData("", String.valueOf(getHeading()));
            telemetry.update();
        }
        AngleTraveled = Math.abs(getHeading() - initAngle);
        if(AngleTraveled >= 5 && AngleTraveled < 45) return 0;
        else if(AngleTraveled >= 45 && AngleTraveled < 95) return 1;
        else if(AngleTraveled >= 95 && AngleTraveled < 180) return 2;
        else return 1;
    }
    public boolean checkDistance(double inches) {
        double distance = robot.ods.getDistance(DistanceUnit.INCH);
        if (distance < inches) {
            return true;
        }
        else {
            return false;
        }
    }
    public double getDistance(){
        return robot.ods.getDistance(DistanceUnit.INCH);

    }
    public void dropPixel() {
        robot.intake.setPower(-.7);
        sleep(1500);
        robot.intake.setPower(0);
    }
    public boolean AprilTagNav(double p, double a, int ID, double RyB, double RxB, double Buffer, float BR, int milis){
        robot.Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor1.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor2.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor3.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor4.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        boolean targetFound     = false;
        boolean SomethingFound;
        double Rx =0;
        double Ry =0;
        double Bearing = 0;
        double yaw = 0;
        double initRx =0;
        double initRy = 0;
        double initD = 0;
        int TimesFound =0;
        double KP = 0;
        double KP1 = 0;
        double AngleD = getHeading() - a;
        if(AngleD > 180) AngleD -= 180;
        if(AngleD < -180) AngleD += 180;
        float Direction;
        int[] Distance = new int[4];
        robot.DESIRED_TAG_ID = ID;
        ElapsedTime Time = new ElapsedTime();
        ElapsedTime NotFound = new ElapsedTime();
        NotFound.reset();
        Time.reset();
        Nav: while (opModeIsActive() && Time.milliseconds() < milis){
            Distance[0]= robot.Motor1.getCurrentPosition();
            Distance[1]= robot.Motor2.getCurrentPosition();
            Distance[2]= robot.Motor3.getCurrentPosition();
            Distance[3]= robot.Motor4.getCurrentPosition();

            targetFound = false;
            robot.desiredTag = null;
            SomethingFound = false;
            Direction = 0;
            AngleD = getHeading() - a;
            if(AngleD > 180) AngleD -= 180;
            if(AngleD < -180) AngleD += 180;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();
            //ArrayList<Integer> DectectedIDs = new ArrayList<Integer>();
            for (AprilTagDetection detection : currentDetections) {
                //DectectedIDs.add(detection.id);
                if ((detection.metadata != null) &&
                        ((robot.DESIRED_TAG_ID < 0) || (detection.id == robot.DESIRED_TAG_ID))) {
                    targetFound = true;
                    TimesFound++;
                    SomethingFound = true;
                    robot.desiredTag = detection;

                    break;  // don't look any further.

                }
                else if (detection.metadata != null) {
                    Direction = Math.signum(robot.DESIRED_TAG_ID - detection.id);
                    SomethingFound =true;
                }
                else {
                    telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if(targetFound){
                NotFound.reset();
            }

            Found: if ((targetFound || NotFound.milliseconds() < 1000) && TimesFound > 0) {
                if(targetFound){

                    //telemetry.addData("", "TargetFound");
                    //telemetry.addData("Target", "ID %d (%s)", robot.desiredTag.id, robot.desiredTag.metadata.name);
                    //telemetry.addData("Range", "%5.1f inches", robot.desiredTag.ftcPose.range);
                    //telemetry.addData("Bearing", "%3.0f degrees", robot.desiredTag.ftcPose.bearing);
                    //telemetry.addData("Yaw", robot.desiredTag.ftcPose.yaw);
                    //double Range = robot.desiredTag.ftcPose.range;
                    AprilB =  robot.desiredTag.ftcPose.bearing;
                    AprilYaw = robot.desiredTag.ftcPose.yaw;
                    AprilX = robot.desiredTag.ftcPose.x;
                    AprilY = robot.desiredTag.ftcPose.y;

                    AprilY -= RyB;
                    AprilX += RxB;
                }
                    KP = Math.min(Math.abs((AprilY - AprilX)/(20)) , 1) * Math.signum(AprilY - AprilX) * p;
                    KP1 = Math.min(Math.abs((AprilY + AprilX)/(20)), 1) * Math.signum(AprilY+AprilX) * p;
                    KP = Math.max(Math.abs(KP), .2)  * Math.signum(KP);
                    KP1 = Math.max(Math.abs(KP1), .2) * Math.signum(KP1);

                robot.Motor1.setPower(KP - (AprilYaw/150) - (AprilB / 90));
                robot.Motor2.setPower(KP1 +(AprilYaw/150) + (AprilB / 90));
                robot.Motor3.setPower(KP1- (AprilYaw/150) - (AprilB / 90));
                robot.Motor4.setPower(KP + (AprilYaw/150) + (AprilB / 90));
                if(Math.abs(AprilY) < Buffer && Math.abs(AprilX) < Buffer && Math.abs(AprilYaw) < 6){

                    robot.Motor1.setPower(0);
                    robot.Motor2.setPower(0);
                    robot.Motor3.setPower(0);
                    robot.Motor4.setPower(0);
                    break Nav;
                }

            }
            else {
                if(SomethingFound){
                telemetry.addData(">", "Correct Target Not Found Moving" + Direction);
                robot.Motor1.setPower(-p*Direction - (AprilYaw/60));
                robot.Motor2.setPower(p*Direction + (AprilYaw/60));
                robot.Motor3.setPower(p*Direction -(AprilYaw/60));
                robot.Motor4.setPower(-p*Direction + (AprilYaw/60));
                }
                else{
                    telemetry.addData(">", "Nothing was Found" + BR);


                }

            }
            telemetry.addData("Angle Distance", AngleD);
            telemetry.addData("Current Heading", getHeading());
            telemetry.addData("TargetFound", targetFound);
            telemetry.addData("Something Found", SomethingFound);
            telemetry.addData("RY",Ry);
            telemetry.addData("Rx", Rx);

            telemetry.update();
        }
        telemetry.addData("Target Was Found", TimesFound);
        telemetry.update();
        return targetFound;
    }
    double AprilX = 0;
    double AprilY = 0;
    double AprilB = 0;
    double AprilYaw = 0;

    public int LocateTag(double p, double a, float D, double Bound){
        robot.Motor1.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor2.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor3.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor4.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        robot.Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AprilX = 0;
        AprilY = 0;
        AprilB = 0;
        AprilYaw = 0;
        boolean SomethingFound = false;
        double AngleD = getHeading() - a;
        int BoundTicks = (int)(Bound * horizontal_ticks_perinch);
        int ret =-1;
        double KP;
        ElapsedTime Time = new ElapsedTime();
        Time.reset();
        move: while(opModeIsActive() && Time.seconds() < 5){
            AngleD = getHeading() - a;
            if(AngleD > 180) AngleD -= 180;
            if(AngleD < -180) AngleD += 180;
            List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    SomethingFound =true;
                    ret = detection.id;
                    AprilX = detection.ftcPose.x;
                    AprilY = detection.ftcPose.y;
                    AprilB = detection.ftcPose.bearing;
                    AprilYaw = detection.ftcPose.yaw;
                    break move;
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
            if(Math.abs(BoundTicks - robot.Motor2.getCurrentPosition()) < 100){
                SomethingFound = false;
                ret = 0;
                break move;
            }
            KP =  Math.abs(BoundTicks - robot.Motor2.getCurrentPosition())/400;
            KP = Math.min(1,KP);
            KP = Math.max(.3,KP);
            robot.Motor1.setPower((p*-D*(KP)) - (AngleD/60));
            robot.Motor2.setPower((p*D* (KP)) +(AngleD/60));
            robot.Motor3.setPower((p*D* (KP)) -(AngleD/60));
            robot.Motor4.setPower((p*-D* (KP)) + (AngleD/60));
        }
        robot.Motor1.setPower(0);
        robot.Motor2.setPower(0);
        robot.Motor3.setPower(0);
        robot.Motor4.setPower(0);

        return ret;
    }
    public void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (robot.visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (robot.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (robot.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(10);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = robot.visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(20);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = robot.visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    @Override
   public void runOpMode()  {

   }
}

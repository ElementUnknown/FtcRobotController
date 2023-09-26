package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Autonomous_Base extends LinearOpMode {
    HardwareMap robot = new HardwareMap();
    private ElapsedTime runtime = new ElapsedTime();
    public double vertical_ticks_perinch = 44.0771349;
    public double horizontal_ticks_perinch = 50.7936507;
    public double intoffset = 0;
    private int checkNum = 0;
    private boolean GoalFound = false;


    boolean turn[] = new boolean[3];


    public void Move(double power, double distanceforward, double distancelateral /*, FinalTurnSpeed*/) {
        double InitHeading = getHeading();
        double AngleDistance = 0;
        ElapsedTime Time = new ElapsedTime();
        double M1Speed;
        double M2Speed;
        double M3Speed;
        double M4Speed;

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
        robot.Motor1.setTargetPosition((int) (Target_ticks_Vertical - Target_tick_Horizontal));
        robot.Motor2.setTargetPosition((int) (Target_ticks_Vertical + Target_tick_Horizontal));
        robot.Motor3.setTargetPosition((int) (Target_ticks_Vertical + Target_tick_Horizontal));
        robot.Motor4.setTargetPosition((int) (Target_ticks_Vertical - Target_tick_Horizontal));

        double accelortation = Math.min(Time.seconds(), 1.0);
        M1Speed = (Math.abs(power * ((distanceforward - distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral)))));
        M2Speed = (Math.abs(power * ((distanceforward + distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral)))));
        M3Speed = (Math.abs(power * ((distanceforward + distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral)))));
        M4Speed = (Math.abs(power * ((distanceforward - distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral)))));

        //robot.Motor1.setPower(accelortation * (Math.abs(power * ((distanceforward - distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral))))));
        //robot.Motor2.setPower(accelortation * (Math.abs(power * ((distanceforward + distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral))))));
        //robot.Motor3.setPower(accelortation * (Math.abs(power * ((distanceforward + distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral))))));
        //robot.Motor4.setPower(accelortation * (Math.abs(power * ((distanceforward - distancelateral) / (Math.abs(distanceforward) + Math.abs(distancelateral))))));

        robot.Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        StraightWait(InitHeading, M1Speed, M2Speed, M3Speed, M4Speed);
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

        while (robot.Motor1.isBusy() && robot.Motor2.isBusy() && robot.Motor3.isBusy() && robot.Motor4.isBusy()) {

        }
    }

    public void StraightWait(double InitHeading, double M1Speed, double M2Speed, double M3Speed, double M4Speed) { //To be Tested
        ElapsedTime Time = new ElapsedTime();
        double Motor1Power = 0;
        double Motor2Power = 0;
        double Motor3Power = 0;
        double Motor4Power = 0;
        double AngleDistance = 0;
        boolean continueloop = true;
        double TickDistance;
        while ((robot.Motor1.isBusy() && robot.Motor2.isBusy() && robot.Motor3.isBusy() && robot.Motor4.isBusy()) && continueloop) {
            AngleDistance = -getHeading() + InitHeading;

            double accceleratioon_ctl = Math.min(Time.milliseconds() / 1000.0,1.0);
            if (AngleDistance > 180) AngleDistance = AngleDistance - 360;
            if (AngleDistance <= -180) AngleDistance = AngleDistance + 360;

            Motor1Power = accceleratioon_ctl * (M1Speed - (Math.signum(robot.Motor1.getTargetPosition()) * (AngleDistance / 270)));
            Motor2Power = accceleratioon_ctl * (M2Speed + (Math.signum(robot.Motor2.getTargetPosition()) * (AngleDistance / 270)));
            Motor3Power = accceleratioon_ctl * (M3Speed - (Math.signum(robot.Motor3.getTargetPosition()) * (AngleDistance / 270)));
            Motor4Power = accceleratioon_ctl * (M4Speed + (Math.signum(robot.Motor4.getTargetPosition()) * (AngleDistance / 270)));


            robot.Motor1.setPower(Motor1Power);
            robot.Motor2.setPower(Motor2Power);
            robot.Motor3.setPower(Motor3Power);
            robot.Motor4.setPower(Motor4Power);

            telemetry.addData("M1", String.valueOf(robot.Motor1.getPower()));
            telemetry.addData("M2", String.valueOf(robot.Motor2.getPower()));
            telemetry.addData("M3", String.valueOf(robot.Motor3.getPower()));
            telemetry.addData("M4", String.valueOf(robot.Motor4.getPower()));
            telemetry.update();

            TickDistance = Math.abs((robot.Motor1.getTargetPosition() - robot.Motor1.getCurrentPosition())) + Math.abs((robot.Motor2.getTargetPosition() - robot.Motor2.getCurrentPosition())) + Math.abs((robot.Motor3.getTargetPosition() - robot.Motor3.getCurrentPosition())) + Math.abs((robot.Motor4.getTargetPosition() - robot.Motor4.getCurrentPosition()));
            if (Math.abs(TickDistance) > 200) continueloop = true;

            else if (Math.abs(TickDistance) < 200) continueloop = false;

            telemetry.addData("", String.valueOf(InitHeading));
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
    public void waitforarmfinish() {
        while (robot.liftArmL.isBusy() || robot.liftArmR.isBusy()) {

        }
    }

    public void ArmGround(int TicksR, int TicksL) {

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftArmL.setPower(1);
        robot.liftArmR.setPower(1);

        robot.liftArmL.setTargetPosition(0 - TicksL);
        robot.liftArmR.setTargetPosition(0 - TicksR);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void Lowgoal(int TicksR, int TicksL) {

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftArmL.setPower(1);
        robot.liftArmR.setPower(1);

        robot.liftArmL.setTargetPosition(500 - TicksL);
        robot.liftArmR.setTargetPosition(500 - TicksR);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void Goalreadjust() {

        int LiftL = robot.liftArmL.getCurrentPosition();
        int LiftR = robot.liftArmR.getCurrentPosition();
        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftArmL.setPower(.75);
        robot.liftArmR.setPower(.75);

        robot.liftArmL.setTargetPosition(LiftL - 160);
        robot.liftArmR.setTargetPosition(LiftR - 160);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void raisearmoffgoal() {

        int LiftL = robot.liftArmL.getCurrentPosition();
        int LiftR = robot.liftArmR.getCurrentPosition();
        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftArmL.setPower(.75);
        robot.liftArmR.setPower(.75);

        robot.liftArmL.setTargetPosition(LiftL + 150);
        robot.liftArmR.setTargetPosition(LiftR + 150);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void Medgoal(int TicksR, int TicksL) {

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmL.setPower(1);
        robot.liftArmR.setPower(1);

        robot.liftArmL.setTargetPosition(850 - TicksL);
        robot.liftArmR.setTargetPosition(850 - TicksR);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void Highgoal(int TicksR, int TicksL) {

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftArmL.setPower(1);
        robot.liftArmR.setPower(1);

        robot.liftArmL.setTargetPosition(1100 - TicksL);
        robot.liftArmR.setTargetPosition(1100 - TicksR);

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void Grabconeheight(int ConesLeft) {

        robot.liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.liftArmL.setPower(.6);
        robot.liftArmR.setPower(.6);

        robot.liftArmL.setTargetPosition(185 + (35 * (ConesLeft -5)));
        robot.liftArmR.setTargetPosition(185 + (35 * (ConesLeft -5)));
        robot.liftArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void TurnByGyro(double target, double speed, double buffer, double multplierquotiant) {//quotiant is the degree from the target with which you want to begin decelaeration
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
        while (truetarget > 180) truetarget = truetarget - 360;
        while (truetarget < -180) truetarget = truetarget + 360;
        angledistance = currentHeading - truetarget;
        if (angledistance > 180) angledistance = angledistance - 360;
        if (angledistance < -180) angledistance = angledistance + 360;
        if (Math.abs(angledistance) >= buffer) continueangleloop = true;
        while (opModeIsActive() && continueangleloop) {
            currentHeading = getHeading();
            TimeTarget = Math.signum(truetarget) * Math.min(AngleInc * NotReset.seconds(), Math.abs(truetarget));
            angledistance = currentHeading - TimeTarget;

            if (angledistance > 180) angledistance = angledistance - 360;
            if (angledistance < -180) angledistance = angledistance + 360;

            P = angledistance * KP;
            D = (angledistance - LastGyroError) / Reset.seconds();
            I = I + (angledistance * Reset.seconds());

            LastGyroError = angledistance;

            robot.Motor1.setPower(speed * ((P) + (I * KI) + (D * KD)));
            robot.Motor2.setPower(-speed * ((P) + (I * KI) + (D * KD)));
            robot.Motor3.setPower(speed * ((P) + (I * KI) + (D * KD)));
            robot.Motor4.setPower(-speed * ((P) + (I * KI) + (D * KD)));
            if (Math.abs(angledistance) >= buffer) continueangleloop = true;
            else continueangleloop = false;
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
        while (heading > 180) heading = heading - 360;
        while (heading < -180) heading = heading + 360;
        return heading;
    }

    public void ODSNavigateGoal(double speed, double CloseBound, double FarBound, int height, double FinalCorrection){
        int CloseBoundTicks = (int) (CloseBound * horizontal_ticks_perinch);
        int FarBoundTicks = (int) (FarBound * vertical_ticks_perinch);
        double Distance = robot.ods.getDistance(DistanceUnit.INCH);
        double error;
        Move(speed, 0, CloseBound);
        CloseToFar(speed, CloseBoundTicks, FarBoundTicks);
        sleep(150);
        if (!GoalFound){
            FarToClose(speed, CloseBoundTicks, FarBoundTicks);
        }
        if (!GoalFound){
            sleep(150);
            Move(speed, 0, FinalCorrection);
        }
        Lowgoal(0,0);
        waitforarmfinish();
        if (GoalFound){
            telemetry.addData("", "Found");
            telemetry.update();
            Distance = robot.ods.getDistance(DistanceUnit.INCH);
            error = Distance - 3.875;
                Move(.2,error,0);
        }

    }


    public void CloseToFar(double speed, int CloseTicks, int FarTicks){
        int DistanceTravel = FarTicks - CloseTicks;
        double Distance = robot.ods.getDistance(DistanceUnit.INCH);
        boolean RunLoop = true;

        while (opModeIsActive() && RunLoop && Distance > 10){
            Distance = robot.ods.getDistance(DistanceUnit.INCH);

            robot.Motor1.setPower(-speed * Math.signum(CloseTicks));
            robot.Motor2.setPower(speed * Math.signum(CloseTicks));
            robot.Motor3.setPower(speed * Math.signum(CloseTicks));
            robot.Motor4.setPower(-speed * Math.signum(CloseTicks));

            if (Math.abs(robot.Motor1.getCurrentPosition()) >= Math.abs(FarTicks) || Math.abs(robot.Motor2.getCurrentPosition()) >= Math.abs(FarTicks) || Math.abs(robot.Motor3.getCurrentPosition()) >= Math.abs(FarTicks) || Math.abs(robot.Motor4.getCurrentPosition()) >= Math.abs(FarTicks)){
                RunLoop = false;
                GoalFound = false;
            }
            else if (Distance < 10){
                RunLoop = false;
                GoalFound = true;
            }
        }
    }
    public void FarToClose(double speed, int CloseTicks, int FarTicks){
        double Distance = robot.ods.getDistance(DistanceUnit.INCH);
        boolean RunLoop = true;
        while (opModeIsActive() && RunLoop && Distance > 10){
            Distance = robot.ods.getDistance(DistanceUnit.INCH);

            robot.Motor1.setPower(-speed * Math.signum(CloseTicks));
            robot.Motor2.setPower(speed * Math.signum(CloseTicks));
            robot.Motor3.setPower(speed * Math.signum(CloseTicks));
            robot.Motor4.setPower(-speed * Math.signum(CloseTicks));

            if(Math.abs(robot.Motor1.getCurrentPosition()) > Math.abs(FarTicks) || Math.abs(robot.Motor2.getCurrentPosition()) > Math.abs(FarTicks) || Math.abs(robot.Motor3.getCurrentPosition()) > Math.abs(FarTicks) || Math.abs(robot.Motor4.getCurrentPosition()) > Math.abs(FarTicks)) {
                RunLoop = false;
                GoalFound = false;
            }
            else if (Distance < 10){
                RunLoop = false;
                GoalFound = true;
            }
        }
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

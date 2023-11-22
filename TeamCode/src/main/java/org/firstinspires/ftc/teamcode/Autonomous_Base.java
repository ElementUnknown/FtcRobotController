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
    private int checkNum = 0;
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
        StraightWait(InitHeading, M1Speed, M2Speed, M3Speed, M4Speed);
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

        while (robot.Motor1.isBusy() || robot.Motor2.isBusy() || robot.Motor3.isBusy() || robot.Motor4.isBusy()) {
            telemetry.addData("Motor 1 Position", robot.Motor1.getCurrentPosition());
            telemetry.addData("Motor 2 Position", robot.Motor2.getCurrentPosition());
            telemetry.addData("Motor 3 Position", robot.Motor3.getCurrentPosition());
            telemetry.addData("Motor 4 Position", robot.Motor4.getCurrentPosition());
            telemetry.update();
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
        //I don't even know how this works, I drank coffee and then the next day this was here
        while ((robot.Motor1.isBusy() && robot.Motor2.isBusy() && robot.Motor3.isBusy() && robot.Motor4.isBusy()) && continueloop) {
            AngleDistance = getHeading() - InitHeading;

            double accceleratioon_ctl = Math.min(Time.milliseconds() / 1000.0, 1.0);
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
            //All I know is that this really long line of code just calculates the total error between the four motors
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
        robot.intake.setPower(-.3);
        sleep(1500);
        robot.intake.setPower(0);
    }

   @Override
   public void runOpMode()  {

   }
}

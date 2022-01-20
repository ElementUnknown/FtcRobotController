package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class Hardware20212022 {

    /* Public OpMode members. */
    public DcMotor Toprightmotor        = null;
    public DcMotor Topleftmotor         = null;
    public DcMotor Bottomrightmotor     = null;
    public DcMotor Bottomleftmotor      = null;
    public DcMotor wheelspin            = null;
    public DcMotor turntable            = null;
    public DcMotor arm                  = null;
    //public DcMotor intake               = null;
    //public ColorSensor colorSensor      = null;
    //public BNO055IMU imu                = null;

    public double restD = .05;
    public double restB = .85;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware20212022(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(com.qualcomm.robotcore.hardware.HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        Topleftmotor            = hwMap.get(DcMotor.class, "Topleftmotor");
        Toprightmotor           = hwMap.get(DcMotor.class, "Toprightmotor");
        Bottomleftmotor         = hwMap.get(DcMotor.class, "Bottomleftmotor");
        Bottomrightmotor        = hwMap.get(DcMotor.class, "Bottomrightmotor");
        wheelspin               = hwMap.get(DcMotor.class, "wheelspin");
        turntable               = hwMap.get(DcMotor.class, "turntable");
        arm                     = hwMap.get(DcMotor.class, "arm");
        //intake                  = hwMap.get(DcMotor.class, "intake");
        //colorSensor             = hwMap.get(ColorSensor.class, "colorSensor");

        Bottomleftmotor.setDirection(DcMotor.Direction.REVERSE);
        Toprightmotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        Toprightmotor.setPower(0);
        Topleftmotor.setPower(0);
        Bottomrightmotor.setPower(0);
        Bottomleftmotor.setPower(0);
        wheelspin.setPower(0);
        turntable.setPower(0);
        arm.setPower(0);
        //intake.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        Toprightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Topleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bottomrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bottomleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelspin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Toprightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Topleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bottomrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bottomleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelspin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turntable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.

        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode             = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit        =BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit        =BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled   =true;
        parameters.loggingTag = "imu";



        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);J*/
    }
}

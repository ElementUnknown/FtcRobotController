package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareMap {

    /* Public OpMode members. */
    public DcMotor Motor1       = null;
    public DcMotor Motor2       = null;
    public DcMotor Motor3       = null;
    public DcMotor Motor4       = null;
    public DcMotor liftArmL     = null;
    public DcMotor liftArmR     = null;
    public Servo   Claw         = null;
    public Servo   PivotClaw    = null;

    public ModernRoboticsI2cColorSensor colorSensor = null;
    public BNO055IMU       imu;
    public Orientation angles;

    /* local OpMode members. */
    com.qualcomm.robotcore.hardware.HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMap(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(com.qualcomm.robotcore.hardware.HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        Motor1          = hwMap.get(DcMotor.class, "Motor1");
        Motor2          = hwMap.get(DcMotor.class, "Motor2");
        Motor3          = hwMap.get(DcMotor.class, "Motor3");
        Motor4          = hwMap.get(DcMotor.class, "Motor4");
        liftArmL        = hwMap.get(DcMotor.class, "liftArmL");
        liftArmR        = hwMap.get(DcMotor.class, "LiftArmR");
        colorSensor     = hwMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor");
        //initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        Motor1.setDirection(DcMotor.Direction.REVERSE);
        Motor3.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);
        liftArmL.setPower(0);
        liftArmR.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArmL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArmR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Claw = hwMap.get(Servo.class, "Claw");
        Claw.setPosition(.83);
        PivotClaw = hwMap.get(Servo.class, "PivotClaw");
        PivotClaw.setPosition(.5);
    }
}

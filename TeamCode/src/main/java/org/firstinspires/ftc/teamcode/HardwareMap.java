package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public DcMotor intake     = null;
    public DcMotor PivotArm     = null;
    //public DcMotor intake1      = null;
    //public DcMotor intake2      = null;
    public DcMotor liftArm     = null;
    //public DcMotor liftArmR     = null;
    public Servo   clawL         = null;
    public Servo   clawR         = null;

    public Servo   Launch       = null;
    public BNO055IMU       imu;
    public Orientation angles;
    public double initAngle;
    public DistanceSensor ods;

    //public DistanceSensor ods;
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
        PivotArm        = hwMap.get(DcMotor.class, "PivotArm");
        intake        = hwMap.get(DcMotor.class, "intake");
        liftArm        = hwMap.get(DcMotor.class, "liftArm");
        //intake1        = hwMap.get(DcMotor.class, "intake1");
        //intake2        = hwMap.get(DcMotor.class, "intake2");
        //initialize IMU
       BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
       parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
       imu = hwMap.get(BNO055IMU.class, "imu");
       imu.initialize(parameters);
       angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

       ods = hwMap.get(DistanceSensor.class, "ods");

       Motor1.setDirection(DcMotor.Direction.REVERSE);
       Motor3.setDirection(DcMotor.Direction.REVERSE);

        //Set all motors to zero power
        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);
        PivotArm.setPower(0);
        intake.setPower(0);
        liftArm.setPower(0);
        //intake1.setPower(0);
        //intake2.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PivotArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PivotArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawL = hwMap.get(Servo.class, "clawL");
        clawR = hwMap.get(Servo.class, "clawR");
        Launch = hwMap.get(Servo.class, "Launch");
        clawL.setPosition(0);
        clawR.setPosition(.8);
        Launch.setPosition(1);
        //PivotClaw = hwMap.get(Servo.class, "PivotClaw");
        //PivotClaw.setPosition(.5);
        initAngle = angles.firstAngle;
    }
}

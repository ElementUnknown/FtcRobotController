package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

public class HardwareMap {

    /* Public OpMode members. */
    public DcMotor Toprightmotor        = null;
    public DcMotor Topleftmotor         = null;
    public DcMotor Bottomrightmotor     = null;
    public DcMotor Bottomleftmotor      = null;

    public ModernRoboticsI2cColorSensor colorSensor = null;

    /* local OpMode members. */
    com.qualcomm.robotcore.hardware.HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMap(){

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
        colorSensor             = hwMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor");

        Bottomleftmotor.setDirection(DcMotor.Direction.REVERSE);
        Bottomrightmotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        Toprightmotor.setPower(0);
        Topleftmotor.setPower(0);
        Bottomrightmotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        Toprightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Topleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bottomrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bottomleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Toprightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Topleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Bottomrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Bottomleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

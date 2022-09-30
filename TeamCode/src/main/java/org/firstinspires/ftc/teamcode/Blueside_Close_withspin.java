package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Autonomous_Base2021_22;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Arrays;


@Autonomous (name = "Blueside_Close_withspin", group = "robot")
public class Blueside_Close_withspin extends LinearOpMode{

    Autonomous_Base2021_22      Auto =  new Autonomous_Base2021_22();
    private ElapsedTime     runtime = new ElapsedTime();
    BNO055IMU imu;
    Orientation angles;
    double currentHeading;

    public void getOrientation() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        currentHeading = angles.firstAngle;
        telemetry.addData("Heading", angles.firstAngle);
        telemetry.update();
    }

    @Override
    public void runOpMode() {

        Auto.robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status","Ready to run");
        telemetry.update();

        waitForStart();
        telemetry.addData("starting", "starting");
        telemetry.update();

        //Autonomous code starts here

        Auto.gyroTurn(0,-95, -85);
        Auto.gyroTurn(1,85, 95);
        Auto.rotateTurntable(30,0.5);
    }
}

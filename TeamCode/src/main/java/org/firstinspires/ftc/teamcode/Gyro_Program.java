package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;

@Autonomous (name = "Gyro Program", group = "robot")
public class Gyro_Program extends LinearOpMode {

    Autonomous_Base2021_22  Auto = new Autonomous_Base2021_22();
    BNO055IMU imu;
    Orientation angles;
    double currentHeading;
    boolean turn[] = new boolean[3];

    private void getOrientation() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        currentHeading = angles.firstAngle;
        telemetry.addData("Heading", angles.firstAngle);
        telemetry.update();
    }

    @Override
    public void runOpMode() {

        Auto.robot.init(hardwareMap);

        Arrays.fill(turn, false);

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

        //Autonomous code starts here.

        while (opModeIsActive()) {
            while (turn[0] == false) {
                Auto.robot.Toprightmotor.setPower(-.3);
                Auto.robot.Bottomrightmotor.setPower(-.3);
                Auto.robot.Topleftmotor.setPower(.3);
                Auto.robot.Bottomleftmotor.setPower(.3);
                if (currentHeading > 85 && currentHeading < 95) {
                    turn[0] = true;
                }
                else {
                    getOrientation();
                }
            }
            while (turn[1] == false) {
                Auto.robot.Toprightmotor.setPower(.3);
                Auto.robot.Bottomrightmotor.setPower(.3);
                Auto.robot.Topleftmotor.setPower(-.3);
                Auto.robot.Bottomleftmotor.setPower(-.3);
                if (currentHeading < -85 && currentHeading > -95) {
                    turn[1] = true;
                }
                else {
                    getOrientation();
                }
            }
            while (turn[2] == false) {
                Auto.robot.Toprightmotor.setPower(-.3);
                Auto.robot.Bottomrightmotor.setPower(-.3);
                Auto.robot.Topleftmotor.setPower(.3);
                Auto.robot.Bottomleftmotor.setPower(.3);
                if (currentHeading > -5 && currentHeading < 5) {
                    turn[2] = true;
                }
                else {
                    getOrientation();
                }
            }
        }

    }
}

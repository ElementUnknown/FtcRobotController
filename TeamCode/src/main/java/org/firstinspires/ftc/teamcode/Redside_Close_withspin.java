package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Autonomous_Base2021_22;




@Autonomous (name = "Redside_Close_withspin", group = "robot")
public class Redside_Close_withspin extends LinearOpMode {
    Autonomous_Base2021_22  Auto = new Autonomous_Base2021_22();


    @Override
    public void runOpMode() {

        Auto.robot.init(hardwareMap);

        telemetry.addData("Status","Ready to run");
        telemetry.update();

        waitForStart();
        telemetry.addData("starting", "starting");
        telemetry.update();

        //Autonomous code starts here.
        Auto.Move(.5, 2,0);
        //Detect square here.
        Auto.Turning(.4,90);
        Auto.Move(.5,.5,0);
        //Auto.dropBlock();
        Auto.Move(.5,-.5,0);
        Auto.Turning(.4,180);
        Auto.Move(.5,2,0);
        Auto.Move(.5,0,2);
        //Auto.spinWheel(5,1);
        Auto.Move(.5,-8,0);
    }
}

package org.firstinspires.ftc.teamcode.poofyutils.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;

public class PropProcessor implements VisionProcessor {

    private Mat hsv = new Mat();
    private Mat blueThresh = new Mat();
    private Mat redThresh = new Mat();
    private ArrayList<MatOfPoint> contours = new ArrayList<>();

    private Scalar purpleColor = new Scalar(50,50,3);

    public Scalar blueLower = new Scalar(75.2,53.9,126.2);
    public Scalar blueUpper = new Scalar(110.9, 207.3, 255.0);

    public Scalar redLower = new Scalar(150.1, 85, 84.8);
    public Scalar redUpper = new Scalar(201.2, 232.5, 255);

    public Rect leftROIBox;
    public Rect centerROIBox;
    public Rect rightROIBox;

    public Rect blueBD_redW_LeftROIBox = new Rect(140, 40, 30, 25);
    public Rect blueBD_redWCenterROIBox = new Rect(395, 45, 30, 25);
    public Rect blueBD_redWRightROIBox = new Rect(0, 265, 30, 25);

    public Rect redBD_blueWLeftROIBox = new Rect(0,265,30,25);
    public Rect redBD_BlueWCenterROIBox = new Rect(220, 40, 30, 25);
    public Rect redBD_blueWRightROIBox = new Rect(475, 60, 30, 25);

    public Mat leftMat = new Mat();
    public Mat centerMat = new Mat();
    public Mat rightMat = new Mat();

    public ArrayList<Mat> ROIs;

    public int spike;

    public Alliance alliance;

    public boolean tuneBlue = false;
    public boolean tuneRed = false;

    /*public PropProcessor(Alliance alliance) {
        this.alliance = alliance;
    }*/

    public PropProcessor() {
        if (tuneBlue) {
            this.alliance = Alliance.BLUE;
        } else if (tuneRed) {
            this.alliance = Alliance.RED;
        } else {
            this.alliance = Alliance.BLUE;
        }
    }


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        ROIs = new ArrayList<>();
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        spike = 0;

        if (tuneBlue) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
            Core.inRange(input, blueLower, blueUpper, input);

            leftROIBox = blueBD_redW_LeftROIBox;
            centerROIBox = blueBD_redWCenterROIBox;
            rightROIBox = blueBD_redWRightROIBox;

            //create ROIs
            leftMat = new Mat(input, leftROIBox);
            centerMat = new Mat(input, centerROIBox);
            rightMat = new Mat(input, rightROIBox);
        } else if (tuneRed) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
            Core.inRange(input, redLower, redUpper, input);

            leftROIBox = redBD_blueWLeftROIBox;
            centerROIBox = redBD_BlueWCenterROIBox;
            rightROIBox = redBD_blueWRightROIBox;

            //create ROIs
            leftMat = new Mat(input, leftROIBox);
            centerMat = new Mat(input, centerROIBox);
            rightMat = new Mat(input, rightROIBox);
        } else if (alliance == Alliance.BLUE || alliance == Alliance.BLUE_BD) {
            //filter to blue
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, blueLower, blueUpper, blueThresh);

            leftROIBox = blueBD_redW_LeftROIBox;
            centerROIBox = blueBD_redWCenterROIBox;
            rightROIBox = blueBD_redWRightROIBox;

            //create ROIs
            leftMat = new Mat(blueThresh, leftROIBox);
            centerMat = new Mat(blueThresh, centerROIBox);
            rightMat = new Mat(blueThresh, rightROIBox);
        } else if (alliance == Alliance.BLUE_W) {
            //filter to blue
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, blueLower, blueUpper, blueThresh);

            leftROIBox = redBD_blueWLeftROIBox;
            centerROIBox = redBD_BlueWCenterROIBox;
            rightROIBox = redBD_blueWRightROIBox;

            //create ROIs
            leftMat = new Mat(blueThresh, leftROIBox);
            centerMat = new Mat(blueThresh, centerROIBox);
            rightMat = new Mat(blueThresh, rightROIBox);
        } else if (alliance == Alliance.RED || alliance == Alliance.RED_BD) {
            //filter to red
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, redLower, redUpper, redThresh);

            leftROIBox = redBD_blueWLeftROIBox;
            centerROIBox = redBD_BlueWCenterROIBox;
            rightROIBox = redBD_blueWRightROIBox;

            //create ROIs
            leftMat = new Mat(redThresh, leftROIBox);
            centerMat = new Mat(redThresh, centerROIBox);
            rightMat = new Mat(redThresh, rightROIBox);
        } else if (alliance == Alliance.RED_W) {
            //filter to red
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, redLower, redUpper, redThresh);

            leftROIBox = blueBD_redW_LeftROIBox;
            centerROIBox = blueBD_redWCenterROIBox;
            rightROIBox = blueBD_redWRightROIBox;

            //create ROIs
            leftMat = new Mat(redThresh, leftROIBox);
            centerMat = new Mat(redThresh, centerROIBox);
            rightMat = new Mat(redThresh, rightROIBox);
        }

        //draw ROIs
        Imgproc.rectangle(input, leftROIBox, purpleColor);
        Imgproc.rectangle(input, centerROIBox, purpleColor);
        Imgproc.rectangle(input, rightROIBox, purpleColor);

        //left ROI
        Imgproc.findContours(leftMat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(leftMat, contours, 0, purpleColor, 2);

        for (MatOfPoint contour : contours) {
            double width = calculateWidth(contour);
            double height = calculateHeight(contour);

            width -= 2;
            height -= 2;

            Moments moments = Imgproc.moments(contour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();

            Imgproc.rectangle(
                    input,
                    new Point(cX - (width/2) + leftROIBox.x, cY - (height/2) + leftROIBox.y),
                    new Point(cX + (width/2) + leftROIBox.x, cY + (height/2) + leftROIBox.y),
                    new Scalar(240,240,240),
                    2
            );
            spike = 1;
        }
        Imgproc.putText(input, Integer.toString(contours.size()), new Point(leftROIBox.x,400), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255,255,255));
        contours.clear();


        //center ROI
        Imgproc.findContours(centerMat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(centerMat, contours, 0, purpleColor, 2);

        for (MatOfPoint contour : contours) {
            double width = calculateWidth(contour);
            double height = calculateHeight(contour);

            width -= 2;
            height -= 2;

            Moments moments = Imgproc.moments(contour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();

            Imgproc.rectangle(
                    input,
                    new Point(cX - (width/2) + centerROIBox.x, cY - (height/2) + centerROIBox.y),
                    new Point(cX + (width/2) + centerROIBox.x, cY + (height/2) + centerROIBox.y),
                    new Scalar(240,240,240),
                    2
            );
            spike = 2;
        }
        Imgproc.putText(input, Integer.toString(contours.size()), new Point(centerROIBox.x,400), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255,255,255));
        contours.clear();


        //right ROI
        Imgproc.findContours(rightMat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(rightMat, contours, 0, purpleColor, 2);

        for (MatOfPoint contour : contours) {
            double width = calculateWidth(contour);
            double height = calculateHeight(contour);

            width -= 2;
            height -= 2;

            Moments moments = Imgproc.moments(contour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();

            Imgproc.rectangle(
                    input,
                    new Point(cX - (width/2) + rightROIBox.x, cY - (height/2) + rightROIBox.y),
                    new Point(cX + (width/2) + rightROIBox.x, cY + (height/2) + rightROIBox.y),
                    new Scalar(240,240,240),
                    2
            );
            spike = 3;
        }
        Imgproc.putText(input, Integer.toString(contours.size()), new Point(rightROIBox.x,400), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255,255,255));
        contours.clear();

        //clear the ROI mats
        ROIs.clear();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    private double calculateHeight(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.height;
    }

    public int getSpike() {
        if (spike == 0 && (alliance == Alliance.BLUE_BD || alliance == Alliance.RED_W)) {
            return 3;
        } else if (spike == 0 && (alliance == Alliance.RED_BD || alliance == Alliance.BLUE_W)) {
            return 1;
        } else {
            return spike;
        }
    }
}
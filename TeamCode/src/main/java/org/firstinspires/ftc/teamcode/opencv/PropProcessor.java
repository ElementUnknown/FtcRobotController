package org.firstinspires.ftc.teamcode.opencv;

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

    public Scalar blueLower = new Scalar(94.9, 103.4, 43.9);
    public Scalar blueUpper = new Scalar(144.5, 255.0, 255.0);

    public Scalar redLower = new Scalar(0, 119, 82.2);
    public Scalar redUpper = new Scalar(46.8, 255, 255);

    public Rect leftROIBox;
    public Rect centerROIBox;
    public Rect rightROIBox;

    public Rect blueLEFT_LeftROIBox = new Rect(130, 340, 230, 180);
    public Rect blueLEFT_CenterROIBox = new Rect(700, 350, 190, 165);
    public Rect blueLEFT_RightROIBox = new Rect(900, 690, 5, 5);

    public Rect blueRIGHT_LeftROIBox = new Rect(900, 690, 5, 5);
    public Rect blueRIGHT_CenterROIBox = new Rect(390, 340, 180, 160);
    public Rect blueRIGHT_RightROIBox = new Rect(960, 350, 240, 160);

    public Rect redLEFT_LeftROIBox = new Rect(130, 340, 260, 200);
    public Rect redLEFT_CenterROIBox = new Rect(680, 350, 200, 170);
    public Rect redLEFT_RightROIBox = new Rect(900, 690, 5, 5);

    public Rect redRIGHT_LeftROIBox = new Rect(900, 690, 5, 5);
    public Rect redRIGHT_CenterROIBox = new Rect(430, 350, 180, 160);
    public Rect redRIGHT_RightROIBox = new Rect(920, 360, 230, 180);

    public Mat leftMat = new Mat();
    public Mat centerMat = new Mat();
    public Mat rightMat = new Mat();

    public ArrayList<Mat> ROIs;

    public int spike;

    public Alliance alliance;

    public boolean tuneBlue = false;
    public boolean tuneRed = false;

    public PropProcessor(Alliance alliance) {
        this.alliance = alliance;
    }

    /*public PropProcessor() {
        if (tuneBlue) {
            this.alliance = Alliance.BLUE_RIGHT;
        } else if (tuneRed) {
            this.alliance = Alliance.RED;
        } else {
            this.alliance = Alliance.BLUE;
        }
    }*/


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

            leftROIBox = blueRIGHT_LeftROIBox;
            centerROIBox = blueRIGHT_CenterROIBox;
            rightROIBox = blueRIGHT_RightROIBox;

            //create ROIs
            leftMat = new Mat(input, leftROIBox);
            centerMat = new Mat(input, centerROIBox);
            rightMat = new Mat(input, rightROIBox);
        } else if (tuneRed) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
            Core.inRange(input, redLower, redUpper, input);

            leftROIBox = redRIGHT_LeftROIBox;
            centerROIBox = redRIGHT_CenterROIBox;
            rightROIBox = redRIGHT_RightROIBox;

            //create ROIs
            leftMat = new Mat(input, leftROIBox);
            centerMat = new Mat(input, centerROIBox);
            rightMat = new Mat(input, rightROIBox);
        } else if (alliance == Alliance.BLUE || alliance == Alliance.BLUE_LEFT) {
            //filter to blue
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, blueLower, blueUpper, blueThresh);

            leftROIBox = blueLEFT_LeftROIBox;
            centerROIBox = blueLEFT_CenterROIBox;
            rightROIBox = blueLEFT_RightROIBox;

            //create ROIs
            leftMat = new Mat(blueThresh, leftROIBox);
            centerMat = new Mat(blueThresh, centerROIBox);
            rightMat = new Mat(blueThresh, rightROIBox);
        } else if (alliance == Alliance.BLUE_RIGHT) {
            //filter to blue
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, blueLower, blueUpper, blueThresh);

            leftROIBox = blueRIGHT_LeftROIBox;
            centerROIBox = blueRIGHT_CenterROIBox;
            rightROIBox = blueRIGHT_RightROIBox;

            //create ROIs
            leftMat = new Mat(blueThresh, leftROIBox);
            centerMat = new Mat(blueThresh, centerROIBox);
            rightMat = new Mat(blueThresh, rightROIBox);
        } else if (alliance == Alliance.RED || alliance == Alliance.RED_RIGHT) {
            //filter to red
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, redLower, redUpper, redThresh);

            leftROIBox = redRIGHT_LeftROIBox;
            centerROIBox = redRIGHT_CenterROIBox;
            rightROIBox = redRIGHT_RightROIBox;

            //create ROIs
            leftMat = new Mat(redThresh, leftROIBox);
            centerMat = new Mat(redThresh, centerROIBox);
            rightMat = new Mat(redThresh, rightROIBox);
        } else if (alliance == Alliance.RED_LEFT) {
            //filter to red
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, redLower, redUpper, redThresh);

            leftROIBox = redLEFT_LeftROIBox;
            centerROIBox = redLEFT_CenterROIBox;
            rightROIBox = redLEFT_RightROIBox;

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
        if (spike == 0 && (alliance == Alliance.BLUE_LEFT || alliance == Alliance.RED_LEFT)) {
            return 3;
        } else if (spike == 0 && (alliance == Alliance.RED_RIGHT || alliance == Alliance.BLUE_RIGHT)) {
            return 1;
        } else {
            return spike;
        }
    }
}
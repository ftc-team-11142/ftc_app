package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ColorPipeline extends OpenCvPipeline {

    private int x1;
    private int y1;
    private int x2;
    private int y2;
    private String side;
    String location = "left";
    String color = "";

    public ColorPipeline(int x1, int y1, int x2, int y2, String side) {
        this.x1 = x1;
        this.y1 = y1;

        this.x2 = x2;
        this.y2 = y2;

        this.side = side;
    }

    @Override
    public Mat processFrame(Mat input) {

        if (input.empty()) {
            location = "center";
            return input;
        }

        if (side.equals("far red")) {
            if ((input.get(y1,x1)[0] - input.get(y1,x1)[2]) > 60) {
                location = "center";
            }
            else if ((input.get(y2,x2)[0] - input.get(y2,x2)[2]) > 60) {
                location = "right";
            }
            else {
                location = "left";
            }
        }

        if (side.equals("far blue")) {
            if ((input.get(y1,x1)[2] - input.get(y1,x1)[0]) > 60) {
                location = "center";
            }
            else if ((input.get(y2,x2)[2] - input.get(y2,x2)[0]) > 60) {
                location = "left";
            }
            else {
                location = "right";
            }
        }

        if (side.equals("close blue")) {
            if ((input.get(y1,x1)[2] - input.get(y1,x1)[0]) > 60) {
                location = "center";
            }
            else if ((input.get(y2,x2)[2] - input.get(y2,x2)[0]) > 60) {
                location = "right";
            }
            else {
                location = "left";
            }
        }

        if (side.equals("close red")) {
            if ((input.get(y1,x1)[0] - input.get(y1,x1)[2]) > 60) {
                location = "center";
            }
            else if ((input.get(y2,x2)[0] - input.get(y2,x2)[2]) > 60) {
                location = "left";
            }
            else {
                location = "right";
            }
        }



        color = Arrays.toString(input.get(y1,x1));

        return input;
    }

    public String getLocation() {
        return this.location;
    }
    public String getColorValue() {
        return this.color;
    }
}
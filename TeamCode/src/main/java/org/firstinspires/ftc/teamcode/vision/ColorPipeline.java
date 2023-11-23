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
import java.util.List;

public class ColorPipeline extends OpenCvPipeline {

    private int x1;
    private int y1;
    private int w1;
    private int h1;
    private int x2;
    private int y2;
    private int w2;
    private int h2;
    private String side;

    private int cr;
    private int cg;
    private int cb;

    private int counter_center;

    private int rr;
    private int rg;
    private int rb;

    private int counter_right;

    String location = "left";

    public ColorPipeline(int x1, int y1, int w1, int h1, int x2, int y2, int w2, int h2, String side) {
        this.x1 = x1;
        this.y1 = y1;
        this.w1 = w1;
        this.h1 = h1;

        this.x2 = x2;
        this.y2 = y2;
        this.w2 = w2;
        this.h2 = h2;

        this.side = side;
    }

    @Override
    public Mat processFrame(Mat input) {

        if (input.empty()) {
            location = "center";
            return input;
        }

        Mat center = input;
        Mat right = input;

        Rect center_rect = new Rect(x1, y1,w1, h1);
        Rect right_rect = new Rect(x2,y2,w2,h2);

        Mat center_crop = new Mat(center,center_rect);
        Mat right_crop = new Mat(right,right_rect);

        for (int i = 0; i < center_crop.cols(); i++) {
            for (int j = 0; j < center_crop.rows(); j++) {
                cb += center_crop.get(i, j)[0];
                cg += center_crop.get(i, j)[1];
                cr += center_crop.get(i, j)[2];
                counter_center += 1;

            }
        }

        cb = cb/ counter_center;
        cg = cg/ counter_center;
        cr = cr/ counter_center;

        for (int i = 0; i < right_crop.cols(); i++) {
            for (int j = 0; j < right_crop.rows(); j++) {
                rb += right_crop.get(i, j)[0];
                rg += right_crop.get(i, j)[1];
                rr += right_crop.get(i, j)[2];
                counter_right += 1;

            }
        }

        rb = rb/ counter_right;
        rg = rg/ counter_right;
        rr = rr/ counter_right;

        if (side.equals("blue")) {
            if(cr < 150 && cb > 100) {
                location = "center";
            }
            else if (rr < 150 && rb > 100) {
                location = "right";
            }
            else {
                location = "left";
            }
        }

        if (side.equals("red")) {
            if(cb < 150 && cr > 100) {
                location = "center";
            }
            else if (rb < 150 && rr > 100) {
                location = "right";
            }
            else {
                location = "left";
            }
        }

        return input;
    }

    public String getLocation() {
        return this.location;
    }
}
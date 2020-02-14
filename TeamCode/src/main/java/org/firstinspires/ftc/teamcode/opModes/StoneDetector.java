package org.firstinspires.ftc.teamcode.opModes;

import com.disnodeteam.dogecv.filters.DogeCVColorFilter;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class StoneDetector extends DogeCVColorFilter {
    private int x, y, width, height;
    private Rect rect;

    StoneDetector(Point p1, Point p2){
        x = (int) Math.floor(p1.x);
        y = (int) Math.floor(p1.y);
        width = (int) Math.floor(p2.x - p1.x);
        height = (int) Math.floor(p2.y - p1.y);
        rect = new Rect(x,y,width,height);
    }

    @Override
    public void process(Mat input, Mat mask){
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,5));
        Mat lab = new Mat();
        Mat maskTemp;
        maskTemp = Mat.zeros(mask.size(), CvType.CV_8UC(1));
        Imgproc.rectangle(maskTemp, rect, new Scalar(255),-1);

        Imgproc.cvtColor(input, lab, Imgproc.COLOR_RGB2Lab);
        Core.inRange(lab, new Scalar(0, configs.aMini, configs.bMini), new Scalar(255, configs.aMaxi, configs.bMaxi), lab);
        Imgproc.GaussianBlur(lab, lab, new Size(3,3), 0);
        Imgproc.dilate(lab, lab, kernel, new Point(-1,-1),5);

        Core.bitwise_and(lab, maskTemp, mask);

        maskTemp.release();
        lab.release();
        input.release();
    }
}

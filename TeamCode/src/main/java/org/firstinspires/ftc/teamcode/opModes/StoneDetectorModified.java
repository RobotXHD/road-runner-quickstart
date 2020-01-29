package org.firstinspires.ftc.teamcode.opModes;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;


public class StoneDetectorModified extends DogeCVDetector {
    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer

    //Create the default filters and scorers
    public DogeCVColorFilter filter = new YellowStoneDetector(); //Default Yellow blackFilter

    public int stonesToFind = 1;
    public MaxAreaScorer maxAreaScorer = new MaxAreaScorer(5);                    // Used to find largest objects

    // Results of the detector
    public ArrayList<Double> pozitieCub = new ArrayList<>();
    private ArrayList<Point> screenPositions = new ArrayList<>(); // Screen positions of the stones
    private ArrayList<Rect> foundRects = new ArrayList<>(); // Found rect


    private Mat rawImage = new Mat();
    private Mat workingMat = new Mat();
    private Mat displayMat = new Mat();
    private Mat yellowMask = new Mat();
    private Mat hierarchy  = new Mat();

    private int x, y, width, height;
    private Rect rect;

    public List<Point> foundScreenPositions() {
        return screenPositions;
    }

    public List<Rect> foundRectangles() {
        return foundRects;
    }

    public List<Double> foundPozitionare(){
        return pozitieCub;
    }

    StoneDetectorModified(Point p1, Point p2) {
        detectorName = "Stone Detector";
        x = (int) Math.floor(p1.x);
        y = (int) Math.floor(p1.y);
        width = (int) Math.floor(p2.x - p1.x);
        height = (int) Math.floor(p2.y - p1.y);
        rect = new Rect(x,y,width,height);
    }

    @Override
    public Mat process(Mat input) {

        input.copyTo(rawImage);
        input.copyTo(workingMat);
        input.copyTo(displayMat);
        input.copyTo(yellowMask);

        // Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        filter.process(workingMat.clone(), yellowMask);

        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Imgproc.findContours(yellowMask, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat, contoursYellow,-1, new Scalar(230,70,70),2);

        Collections.sort(contoursYellow, new Comparator<MatOfPoint>() {
            @Override
            public int compare(MatOfPoint matOfPoint, MatOfPoint t1) {
                return ((int) Math.round(calculateScore(matOfPoint) - calculateScore(t1)));
            }
        });

        List<MatOfPoint> subList = contoursYellow;

        if (contoursYellow.size() > stonesToFind) {
            subList = contoursYellow.subList(0, stonesToFind);
        }
        int i = 0;
        for (MatOfPoint contour : subList) {
            Rect rect = Imgproc.boundingRect(contour);

            // Show chosen result
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(255, 0, 0), 4);
            Imgproc.putText(displayMat, "Chosen", rect.tl(), 0, 1, new Scalar(255, 255, 255));

            pozitieCub.set(i,Map(38,91,115.5,49,rect.height));
            screenPositions.set(i, new Point(rect.x, rect.y));
            foundRects.set(i, rect);
            i++;
        }
        Imgproc.rectangle(displayMat, rect, new Scalar(0,0,255), 3);


        if (foundRects.size() > 0) {
            found = true;
        }
        else {
            found = false;
        }

        switch (stageToRenderToViewport) {
            case THRESHOLD: {
                Imgproc.cvtColor(yellowMask, yellowMask, Imgproc.COLOR_GRAY2BGR);

                return yellowMask;
            }
            case RAW_IMAGE: {
                return rawImage;
            }
            default: {
                return displayMat;
            }
        }
    }

    private double Map(double minVal, double maxVal, double newMin, double newMax, double val) {
           return (val-minVal)/(maxVal-minVal)*(newMax-newMin)+newMin;
    }

    @Override
    public void useDefaults() {
        for(int i = 1; i <= stonesToFind; i++){
            screenPositions.add(new Point(0,0));
            foundRects.add(new Rect(0,0,0,0));
            pozitieCub.add(0.0);
        }
        addScorer(maxAreaScorer);
    }
}
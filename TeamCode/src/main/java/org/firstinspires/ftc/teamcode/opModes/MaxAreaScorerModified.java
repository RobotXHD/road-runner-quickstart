package org.firstinspires.ftc.teamcode.opModes;

import com.disnodeteam.dogecv.scoring.DogeCVScorer;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;

/**
 * Created by MasterGigel on 9/10/2018.
 */


public class MaxAreaScorerModified extends DogeCVScorer {
    public double weight = 1.0;
    /**
     * Constructor
     * @param weight - How much to weight the final score (1-10 is usually good)
     */
    public MaxAreaScorerModified( double weight){
        this.weight = weight;
    }

    /**
     * Calculate the score
     * @param input - Input mat (Can be MatOfPoint for contours)
     * @return - Difference from perfect score
     */
    @Override
    public double calculateScore(Mat input) {
        if(!(input instanceof MatOfPoint)) return Double.MAX_VALUE;
        MatOfPoint contour = (MatOfPoint) input;
        double area = Imgproc.contourArea(contour);
        return -area * weight;
    }

}

package frc.robot;

import java.io.Closeable;
import java.lang.Math;
import java.util.*;

public class PathInterpolator {
    static List<Float> raw_r = new ArrayList<Float>() {
        {
            // Raw right encoder values
        }
    };
    static List<Float> raw_l = new ArrayList<Float>() {
        {
            // Left encoder values
        }
    };
    static List<Float> raw_t = new ArrayList<Float>() {
        {
            // unscaled time. may be scaled to make program run faster or slower.
        }
    };
    static List<Float> f_l = new ArrayList<Float>() {
        {
            // Interpolated left encoder function
        }
    };
    static List<Float> f_r = new ArrayList<Float>() {
        {
            // Interpolated right encoder function
        }
    };

    static SplineInterpolator right_Drive;
    static SplineInterpolator left_Drive;

    public static double[] positions;

    static Boolean init = false;

    static int sequencer = 0;

    public void setAll(String fileName) { 
        FileLogger.scanToList(fileName, raw_r, 0);
        FileLogger.scanToList(fileName, raw_l, 1);
        FileLogger.scanToList(fileName, raw_t, 2);
        right_Drive = SplineInterpolator.createMonotoneCubicSpline(raw_t, raw_r);
        left_Drive = SplineInterpolator.createMonotoneCubicSpline(raw_t, raw_l);
        sequencer++;
    }

    /**
     * 
     * @param time - time value. If we want to speed up or slow down the program, we
     *             can multiply/divide the actual time.
     * @return - double[] array containing right and left encoder values robot
     *         should be at at that instant
     * @throws Exception
     */
    public double[] calcPositions(float time) throws Exception {

        double x1 = right_Drive.interpolate(time);
        double x2 = left_Drive.interpolate(time);
        double[] positions = { x1, x2 };
        return positions;
    }
}

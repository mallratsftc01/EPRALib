package com.epra.epralib.ftclib.camera.pipeline;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class UniversalColorDeterminationPipeline extends OpenCvPipeline {
    //-1 means no regions match
    public int elementRegion = -1;
    //x = Cb, y = Cr
    final Point WHITE = new Point(128, 128);
    final Point GREEN = new Point(16, 16);
    final Point YELLOW = new Point(16, 128);
    final Point RED = new Point(16, 240);
    final Point MAGENTA = new Point(240, 240);
    final Point BLUE = new Point(240, 16);
    final Point CYAN = new Point(128, 16);
    //contains potential  colors
    public enum ElementColor {
        WHITE,
        GREEN,
        YELLOW,
        RED,
        MAGENTA,
        BLUE,
        CYAN
    }

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile ElementColor color = ElementColor.WHITE;

    //color constants
    static final Scalar BLUE_S = new Scalar(0, 0, 255);
    static final Scalar GREEN_S = new Scalar(0, 255, 0);

    //core values to determine the scanning position and size
    //Not finals as they are set in init, but effectively finals after init
    static Point[] regions;
    static int regionWidth;
    static int regionHeight;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    Point[] regionsPointA;
    Point[] regionsPointB;

    //working variables
    Mat[] regionsCr;
    Mat[] regionsCb;
    Mat YCrCb = new Mat();
    Mat Cr = new Mat();
    Mat Cb = new Mat();
    int[] avgCr;
    int[] avgCb;
    int targetCr;
    int targetCb;
    int reqProx;

    /**This class will detect a specified color in any number of regions and return the region that most closely matches that color.
     * <p></p>
     * This code was derived from code from EasyOpenCV by Windwoes and edited by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * You can find OpenCV at <a href="https://github.com/OpenFTC/EasyOpenCV">https://github.com/OpenFTC/EasyOpenCV</a>
     * <p></p>
     * Each x, y pair determines the upper left-hand corner of each analysis area,
     * the width and height determines the size of each analysis area.
     * <p>
     * The target parameters determine which color the program is searching for,
     * the requiredProximity determines how close a region must be to that color to qualify.
     */
    public UniversalColorDeterminationPipeline(int[] x, int[] y, int height, int width, int targetCrIn, int targetCbIn, int requiredProximity) {
        regions = new Point[x.length];
        for (int ii = 0; ii < regions.length; ii++) {regions[ii] = new Point(x[ii], y[ii]);}
        regionWidth = width;
        regionHeight = height;

        regionsPointA = new Point[regions.length];
        regionsPointB = new Point[regions.length];
        for (int ii = 0; ii < regions.length; ii++) {
            regionsPointA[ii] = new Point(regions[ii].x,
                    regions[ii].y);
            regionsPointB[ii] = new Point(regions[ii].x + regionWidth,
                    regions[ii].y + regionHeight);
        }

        targetCr = targetCrIn;
        targetCb = targetCbIn;
        reqProx = requiredProximity;
    }

    /**
     * Takes the RGB frame, converts to YCrCb,
     * extracts the Cb channel to the 'Cb' variable
     * and does the same for Cr.
     */
    void inputToCbCr(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
        Core.extractChannel(YCrCb, Cr, 1);
    }
    /**Initializes the class.*/
    @Override
    public void init(Mat firstFrame) {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCbCr(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        regionsCr = new Mat[regions.length];
        regionsCb = new Mat[regions.length];

        for (int ii = 0; ii < regions.length; ii++) {
            regionsCr[ii] = Cr.submat(new Rect(regionsPointA[ii], regionsPointB[ii]));
            regionsCb[ii] = Cb.submat(new Rect(regionsPointA[ii], regionsPointB[ii]));
        }
    }

    /**Finds the position and color of the element for a frame*/
    @Override
    public Mat processFrame (Mat input) {
        inputToCbCr(input);

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        avgCr = new int[regions.length];
        avgCb = new int[regions.length];

        for (int ii = 0; ii < regions.length; ii++) {
            avgCr[ii] = (int) Core.mean(regionsCr[ii]).val[0];
            avgCb[ii] = (int) Core.mean(regionsCb[ii]).val[0];
        }

        /*
         * Draw a rectangle showing the regions on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        for (int ii = 0; ii < regions.length; ii++) {
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    regionsPointA[ii], // First point which defines the rectangle
                    regionsPointB[ii], // Second point which defines the rectangle
                    BLUE_S, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
        }


        //find which value is closest to the target

        int[] proxTotal = new int[regions.length];
        for (int ii = 0; ii < proxTotal.length; ii++) {proxTotal[ii] = (int)Math.sqrt(Math.pow(targetCr - avgCr[ii], 2) + Math.pow(targetCb - avgCb[ii], 2));}
        int min = proxTotal[0];
        if (proxTotal.length > 1) {
            min = Math.min(proxTotal[0], proxTotal[1]);
            for (int ii = 2; ii < proxTotal.length; ii++) {min = Math.min(min, proxTotal[ii]);}
        }

        if (min < reqProx) {
            //determine the region of the min
            for (int ii = 0; ii < proxTotal.length; ii++) {
                if (min == proxTotal[ii]) {
                    elementRegion = ii;
                    /*
                     * Draw a solid rectangle on top of the chosen region.
                     * Simply a visual aid. Serves no functional purpose.
                     */
                    Imgproc.rectangle(
                            input, // Buffer to draw on
                            regionsPointA[ii], // First point which defines the rectangle
                            regionsPointB[ii], // Second point which defines the rectangle
                            GREEN_S, // The color the rectangle is drawn in
                            -1); // Negative thickness means solid fill
                }
            }
        }

        if (elementRegion > -1) {
            //determines the closest color
            int proxWhite = (int) Math.sqrt(Math.pow(WHITE.x - avgCr[elementRegion], 2) + Math.pow(WHITE.y - avgCb[elementRegion], 2));
            int proxGreen = (int) Math.sqrt(Math.pow(GREEN.x - avgCr[elementRegion], 2) + Math.pow(GREEN.y - avgCb[elementRegion], 2));
            int proxYellow = (int) Math.sqrt(Math.pow(YELLOW.x - avgCr[elementRegion], 2) + Math.pow(YELLOW.y - avgCb[elementRegion], 2));
            int proxRed = (int) Math.sqrt(Math.pow(RED.x - avgCr[elementRegion], 2) + Math.pow(RED.y - avgCb[elementRegion], 2)) / 2;
            int proxMagenta = (int) Math.sqrt(Math.pow(MAGENTA.x - avgCr[elementRegion], 2) + Math.pow(MAGENTA.y - avgCb[elementRegion], 2));
            int proxBlue = (int) Math.sqrt(Math.pow(BLUE.x - avgCr[elementRegion], 2) + Math.pow(BLUE.y - avgCb[elementRegion], 2));
            int proxCyan = (int) Math.sqrt(Math.pow(CYAN.x - avgCr[elementRegion], 2) + Math.pow(CYAN.y - avgCb[elementRegion], 2));

            int colorMin = Math.min(proxWhite, proxGreen);
            colorMin = Math.min(colorMin, proxYellow);
            colorMin = Math.min(colorMin, proxRed);
            colorMin = Math.min(colorMin, proxMagenta);
            colorMin = Math.min(colorMin, proxBlue);
            colorMin = Math.min(colorMin, proxCyan);

            if (colorMin == proxWhite) {
                color = ElementColor.WHITE;
            } else if (colorMin == proxGreen) {
                color = ElementColor.GREEN;
            } else if (colorMin == proxYellow) {
                color = ElementColor.YELLOW;
            } else if (colorMin == proxRed) {
                color = ElementColor.RED;
            } else if (colorMin == proxMagenta) {
                color = ElementColor.MAGENTA;
            } else if (colorMin == proxBlue) {
                color = ElementColor.BLUE;
            } else if (colorMin == proxCyan) {
                color = ElementColor.CYAN;
            }
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    /**Returns the region with an average color closest to the target*/
    public int getRegion() {return elementRegion;}
    /**Returns the average Cr value of the closest region*/
    public int getCr() {return avgCr[elementRegion];}
    /**Returns the average Cb value of the closest region*/
    public int getCb() {return avgCb[elementRegion];}
    /**Returns the color of the closest region*/
    public ElementColor getColor() {return color;}
}

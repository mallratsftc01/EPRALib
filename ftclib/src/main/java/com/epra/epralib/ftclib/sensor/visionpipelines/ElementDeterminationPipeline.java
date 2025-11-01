package com.epra.epralib.ftclib.sensor.visionpipelines;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ElementDeterminationPipeline extends OpenCvPipeline {
    //contains potential element positions
    public enum ElementPosition {
        LEFT,
        CENTER,
        RIGHT
    }
    //contains potential element colors
    public enum ElementColor {
        BLUE,
        RED
    }

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile ElementPosition position = ElementPosition.LEFT;
    private volatile ElementColor color = ElementColor.BLUE;

    //color constants
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    //core values to determine the scanning position and size
    //Not finals as they are set in init, but effectively finals after init
    static Point REGION1_TOPLEFT_ANCHOR_POINT;
    static Point REGION2_TOPLEFT_ANCHOR_POINT;
    static Point REGION3_TOPLEFT_ANCHOR_POINT;
    static int REGION_WIDTH;
    static int REGION_HEIGHT;

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
    Point region1_pointA;
    Point region1_pointB;
    Point region2_pointA;
    Point region2_pointB;
    Point region3_pointA;
    Point region3_pointB;

    //working variables
    Mat region1_Cb, region2_Cb, region3_Cb, region1_Cr, region2_Cr, region3_Cr;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    Mat Cr = new Mat();
    int avg1Cb, avg2Cb, avg3Cb, avg1Cr, avg2Cr, avg3Cr;

    /**This class will detect a solidly red or solidly blue team element and return its position and color.
     * <p></p>
     * This code was derived from code from EasyOpenCV by Windwoes and edited by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * You can find OpenCV at <a href="https://github.com/OpenFTC/EasyOpenCV">https://github.com/OpenFTC/EasyOpenCV</a>
     * <p></p>
     * Each x, y pair determines the upper left-hand corner of each analysis area,
     * the width and height determines the size of each analysis area.
     */
    public ElementDeterminationPipeline(int x1, int x2, int x3, int y1, int y2, int y3, int height, int width) {
        REGION1_TOPLEFT_ANCHOR_POINT = new Point(x1,y1);
        REGION2_TOPLEFT_ANCHOR_POINT = new Point(x2,y2);
        REGION3_TOPLEFT_ANCHOR_POINT = new Point(x3,y3);
        REGION_WIDTH = width;
        REGION_HEIGHT = height;

        region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
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
    public void init (Mat firstFrame) {
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
        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cr = Cr.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cr = Cr.submat(new Rect(region3_pointA, region3_pointB));
    }

    /**Finds the position and color of the element for a frame*/
    @Override
    public Mat processFrame (Mat input) {
        /*
         * Overview of what we're doing:
         *
         * We first convert to YCrCb color space, from RGB color space.
         * Why do we do this? Well, in the RGB color space, chroma and
         * luma are intertwined. In YCrCb, chroma and luma are separated.
         * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
         * are Y, the luma channel (which essentially just a B&W image), the
         * Cr channel, which records the difference from red, and the Cb channel,
         * which records the difference from blue. Because chroma and luma are
         * not related in YCrCb, vision code written to look for certain values
         * in the Cr/Cb channels will not be severely affected by differing
         * light intensity, since that difference would most likely just be
         * reflected in the Y channel.
         *
         * For detecting team elements we extract both Cr and Cb channels.
         * The Cr channel is used to identify the red element and the Cb does the
         * same for the blue. If the Cr value is high, it is red, the same goes
         * for Cb and blue. We do not just use one channel because the inverse of red
         * in YCrCb is not blue, nor is red the opposite of red.
         *
         * We then take the average pixel value of 3 different regions on that Cb
         * channel, one positioned over potential location of the element. The brightest
         * of the 3 regions in blue or red is where the element is positioned. By
         * looking at the two channels and seeing which is brighter we can also identify
         * the color of the element, which may prove helpful.
         *
         * We also draw rectangles on the screen showing where the sample regions
         * are, as well as drawing a solid rectangle over top the sample region
         * we believe is on top of the element.
         *
         * In order for this whole process to work correctly, each sample region
         * should be positioned in the center of each of the potential locations, and
         * be small enough such that only the element is sampled, and not any of the
         * surroundings.
         */

        inputToCbCr(input);

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        avg1Cb = (int) Core.mean(region1_Cb).val[0];
        avg2Cb = (int) Core.mean(region2_Cb).val[0];
        avg3Cb = (int) Core.mean(region3_Cb).val[0];
        avg1Cr = (int) Core.mean(region1_Cr).val[0];
        avg2Cr = (int) Core.mean(region2_Cr).val[0];
        avg3Cr = (int) Core.mean(region3_Cr).val[0];

        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines


        /*
         * Find the max of the 3 averages
         */
        int maxOneTwoCb = Math.max(avg1Cb, avg2Cb);
        int maxCb = Math.max(maxOneTwoCb, avg3Cb);
        int maxOneTwoCr = Math.max(avg1Cr, avg2Cr);
        int maxCr = Math.max(maxOneTwoCr, avg3Cr);
        int max = Math.max(maxCb, maxCr);
        int avg1;
        int avg2;
        int avg3;

        if (max == maxCr) {
            color = ElementColor.RED;
            avg1 = avg1Cr;
            avg2 = avg2Cr;
            avg3 = avg3Cr;
        } else {
            color = ElementColor.BLUE;
            avg1 = avg1Cb;
            avg2 = avg2Cb;
            avg3 = avg3Cb;
        }

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if(max == avg1) // Was it from region 1?
        {
            position = ElementPosition.LEFT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == avg2) // Was it from region 2?
        {
            position = ElementPosition.CENTER; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == avg3) // Was it from region 3?
        {
            position = ElementPosition.RIGHT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    /**Returns the position of the element*/
    public ElementPosition getPosition() {return position;}
    /**Returns the color of the element*/
    public ElementColor getColor() {return color;}
}

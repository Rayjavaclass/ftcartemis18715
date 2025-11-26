package org.firstinspires.ftc.teamcode.subsystems;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorArtifactPipeline extends OpenCvPipeline {
    public double distanceFromCenter = -1;  // Distance of blob center from camera center

    private Mat hsv = new Mat();
    private Mat purpleMask = new Mat();
    private Mat greenMask = new Mat();
    private Mat combinedMask = new Mat();
    private Mat hierarchy = new Mat();

    // HSV ranges — tune these for your lighting
    private final Scalar lowerPurple = new Scalar(130, 50, 50);
    private final Scalar upperPurple = new Scalar(160, 255, 255);

    private final Scalar lowerGreen = new Scalar(40, 70, 70);
    private final Scalar upperGreen = new Scalar(80, 255, 255);

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Threshold for purple and green
        Core.inRange(hsv, lowerPurple, upperPurple, purpleMask);
        Core.inRange(hsv, lowerGreen, upperGreen, greenMask);

        // Combine masks
        Core.bitwise_or(purpleMask, greenMask, combinedMask);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find largest contour
        double maxArea = 0;
        Rect largestRect = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestRect = Imgproc.boundingRect(contour);
            }
        }

        // Calculate distance from center
        if (largestRect != null) {
            int blobCenterX = largestRect.x + largestRect.width / 2;
            int frameCenterX = input.width() / 2;
            distanceFromCenter = blobCenterX - frameCenterX;

            // Draw rectangle and center lines
            Imgproc.rectangle(input, largestRect, new Scalar(0, 255, 0), 2);
            Imgproc.line(input, new Point(blobCenterX, 0), new Point(blobCenterX, input.height()), new Scalar(255, 0, 0), 2);
            Imgproc.line(input, new Point(frameCenterX, 0), new Point(frameCenterX, input.height()), new Scalar(0, 0, 255), 2);

            Imgproc.putText(input, "Offset: " + distanceFromCenter, new Point(10, 30),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
        } else {
            distanceFromCenter = -1;
        }

        return input;
    }
}

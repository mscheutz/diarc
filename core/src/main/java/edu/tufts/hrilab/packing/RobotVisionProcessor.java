package edu.tufts.hrilab.packing;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.util.List;

/**
 * process a camera image to determine the necessary robot action for pickup of the box
 */
public class RobotVisionProcessor {
    private final double boxWidthMeters;
    private final double boxLengthMeters;
    private final double cameraZDistanceMeters;
    private final int cannyLowThreshold;
    private final int cannyHighThreshold;
    private final int dilationIterations;
    private ContourDetectionTool tool;

    public RobotVisionProcessor(double boxWidthMeters, double boxLengthMeters, double cameraZDistanceMeters) {
        this.boxWidthMeters = boxWidthMeters;
        this.boxLengthMeters = boxLengthMeters;
        this.cameraZDistanceMeters = cameraZDistanceMeters;

        // defaul canny edge parameters that worked on all the test images
        this.cannyLowThreshold = 41;
        this.cannyHighThreshold = 155;
        this.dilationIterations = 4;
        SwingUtilities.invokeLater(() -> {
          tool = new ContourDetectionTool();
          tool.setVisible(true);
        });
    }

    /**
     * takes in a captured image and returns the adjustment to rotate and center the box in the image
     * @param capturedImage the image from the robot camera
     * @return the required adjustments, or null if no object was found
     */
    public Vec3 calculatePickupAction(BufferedImage capturedImage) {
        // call the contour detection to find the tightest rectangle
        List<Point> rectangleCorners = ContourDetector.findLargestObjectRectangle(
            capturedImage, cannyLowThreshold, cannyHighThreshold, dilationIterations);

        if (rectangleCorners == null) {
            System.out.println("Vision Processor: No object found.");
            return null; // No object found
        }

        // turn pixel coordinates into real-world actions
        Point p0 = rectangleCorners.get(0);
        Point p1 = rectangleCorners.get(1);
        Point p3 = rectangleCorners.get(3);
        double edge1 = p0.distance(p1);
        double edge2 = p0.distance(p3);

        double pixelWidth = Math.min(edge1, edge2);
        double realWidth = Math.min(boxWidthMeters, boxLengthMeters);
        
        Point widthEdgeP1 = (edge1 < edge2) ? p0 : p3;
        Point widthEdgeP2 = (edge1 < edge2) ? p1 : p0;

        // determine the factor "a" -- note this is computed anew on the fly every time
	// we should really track this over multiple iteration to learn the best value
        double a = (cameraZDistanceMeters * pixelWidth) / realWidth;

        // calculate the center of the image and the detected rectangle
        double imageCenterX = capturedImage.getWidth() / 2.0;
        double imageCenterY = capturedImage.getHeight() / 2.0;
        double rectCenterX = (rectangleCorners.get(0).x + rectangleCorners.get(2).x) / 2.0;
        double rectCenterY = (rectangleCorners.get(0).y + rectangleCorners.get(2).y) / 2.0;
        
        // calculate the shift required in pixels
        double shiftX_pixels = rectCenterX - imageCenterX;
        double shiftY_pixels = rectCenterY - imageCenterY;

        // convert the pixel shift into a real-world shift in meters
        double realWorldShiftX = (shiftX_pixels * cameraZDistanceMeters) / a;
        double realWorldShiftY = (shiftY_pixels * cameraZDistanceMeters) / a;

        // calculate the rotation required in degrees
        double rotationAngleRad = Math.atan2(widthEdgeP2.y - widthEdgeP1.y, widthEdgeP2.x - widthEdgeP1.x);

        //update GUI
        tool.update(capturedImage, rectangleCorners, imageCenterX, imageCenterY, rectCenterX, rectCenterY, rotationAngleRad, a, realWorldShiftX, realWorldShiftY);

        // return the required adjustment in x,y shift and rotation angle in degrees
        return new Vec3(realWorldShiftX, realWorldShiftY, rotationAngleRad);
    }
}

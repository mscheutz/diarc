package edu.tufts.hrilab.packing;

/**
 * Represents the camera's physical setup.
 */
record Camera(double heightAboveConveyor, double angleFromVerticalDegrees) {
    public double angleFromVerticalRadians() {
        return Math.toRadians(angleFromVerticalDegrees);
    }
}

/**
 * Represents the real-world dimensions of the target box.
 */
record Box(double width, double length, double height) {}


public class VisualServoingCalculator {

    private static final double VALIDATION_TOLERANCE = 0.1; // 10% tolerance for validation

    /**
     * Calculates the required adjustment for the robot arm to center over and align with a box.
     *
     * @param camera The camera's configuration.
     * @param box The real-world dimensions of the box.
     * @param boxProjection The quadrilateral projection of the box in the image.
     * @param imageCenter The pixel coordinates of the center of the camera image.
     * @return A RobotAdjustment object containing the (X, Y) offset and the required rotation in degrees.
     */
    public static RobotAdjustment calculateRobotAdjustment(Camera camera, Box box, Quadrilateral boxProjection, Vector2D imageCenter) {
        // --- Step 1: Calculate Projected Side Lengths ---
        double side12 = boxProjection.getP1().distanceTo(boxProjection.getP2());
        double side23 = boxProjection.getP2().distanceTo(boxProjection.getP3());
        double side34 = boxProjection.getP3().distanceTo(boxProjection.getP4());
        double side41 = boxProjection.getP4().distanceTo(boxProjection.getP1());

        double avgPixPairA = (side12 + side34) / 2.0;
        double avgPixPairB = (side23 + side41) / 2.0;

        // --- Step 2 & 3: Hypothesis Testing and Finalizing Scale ---
        double foreshorteningFactor = Math.cos(camera.angleFromVerticalRadians());
        double scaleHypothesisA = (box.length() * foreshorteningFactor) / avgPixPairA;
        double calculatedWidth = avgPixPairB * scaleHypothesisA;

        double finalScale;
        Vector2D pLength1, pLength2, pLength3, pLength4; // Points defining the length sides

        // System.out.println("--- Hypothesis Test ---");
        // ... (logging from previous step)

        if (Math.abs(calculatedWidth - box.width()) / box.width() < VALIDATION_TOLERANCE) {
            // Hypothesis Correct: Pair A is Length, Pair B is Width
            pLength1 = boxProjection.getP1(); pLength2 = boxProjection.getP2();
            pLength3 = boxProjection.getP4(); pLength4 = boxProjection.getP3(); // Keep vectors parallel
            double scaleFromLength = (box.length() * foreshorteningFactor) / avgPixPairA;
            double scaleFromWidth = box.width() / avgPixPairB;
            finalScale = (scaleFromLength + scaleFromWidth) / 2.0;
        } else {
            // Hypothesis Incorrect: Pair A is Width, Pair B is Length
            pLength1 = boxProjection.getP2(); pLength2 = boxProjection.getP3();
            pLength3 = boxProjection.getP1(); pLength4 = boxProjection.getP4(); // Keep vectors parallel
            double scaleFromWidth = box.width() / avgPixPairA;
            double scaleFromLength = (box.length() * foreshorteningFactor) / avgPixPairB;
            finalScale = (scaleFromWidth + scaleFromLength) / 2.0;
        }
        // System.out.printf("Final Averaged Scale Factor: %.6f m/px\n\n", finalScale);

        // --- Step 4: Calculate Translational Offset ---
        Vector2D boxPixelCenter = boxProjection.getPixelCenter();
        double deltaPixelX = boxPixelCenter.x() - imageCenter.x();
        double deltaPixelY = boxPixelCenter.y() - imageCenter.y();
        Vector2D offset = new Vector2D(deltaPixelX * finalScale, deltaPixelY * finalScale);

        // --- Step 5: Calculate Rotational Offset ---
        // Create vectors for the two long sides of the box
        Vector2D lengthVector1 = new Vector2D(pLength2.x() - pLength1.x(), pLength2.y() - pLength1.y());
        Vector2D lengthVector2 = new Vector2D(pLength4.x() - pLength3.x(), pLength4.y() - pLength3.y());

        // Average the vectors to get a robust direction
        double avgDx = (lengthVector1.x() + lengthVector2.x()) / 2.0;
        double avgDy = (lengthVector1.y() + lengthVector2.y()) / 2.0;

        // Calculate the angle of the average vector relative to the horizontal image axis
        double rotationRadians = Math.atan2(avgDy, avgDx);
        double rotationDegrees = Math.toDegrees(rotationRadians);

        return new RobotAdjustment(offset, rotationDegrees);
    }

//        System.out.println("--- Robot Arm Adjustment Calculation (General Solution) ---");
//
//        // --- ASSUMPTIONS ---
//        System.out.println("\n--- Underlying Assumptions ---");
//        System.out.println("1. The four corner points of the box's top surface are visible and provided sequentially.");
//        System.out.println("2. Camera lens distortion is minimal or has been corrected.");
//        System.out.println("3. The camera's height and tilt angle relative to the conveyor are known and accurate.");
//        System.out.println("4. The real-world dimensions (width, length, height) of the box are known and accurate.");
//        System.out.println("5. The robot arm's X-Y movement plane is parallel to the camera's image plane.");
//        System.out.println("6. The conveyor belt surface is a flat plane.");
//
//        // --- INPUTS ---
//        Camera camera = new Camera(1.0, 30.0);
//        Box box = new Box(0.3, 0.5, 0.1);
//        Vector2D imageCenter = new Vector2D(960, 540);
//
//        // These points would come from a computer vision system in a real application.
//        // They are provided sequentially (clockwise) around the perimeter.
//        Quadrilateral boxProjection = new Quadrilateral(
//            new Vector2D(1254, 353), // p1
//            new Vector2D(1588, 545), // p2 (p1-p2 is a long side)
//            new Vector2D(1491, 648), // p3
//            new Vector2D(1157, 456)  // p4 (p3-p4 is a long side)
//        );
//
//        System.out.println("\n--- Known Inputs ---");
//        System.out.printf("- Camera: %.1fm height, %.1f degrees angle\n", camera.heightAboveConveyor(), camera.angleFromVerticalDegrees());
//        System.out.printf("- Box: %.2fm W x %.2fm L x %.2fm H\n", box.width(), box.length(), box.height());
//        System.out.printf("- Image Center: (%.0f, %.0f) px\n", imageCenter.x(), imageCenter.y());
//        System.out.printf("- Detected Box Corners (p1, p2, p3, p4): (%.0f,%.0f), (%.0f,%.0f), (%.0f,%.0f), (%.0f,%.0f)\n",
//                boxProjection.getP1().x(), boxProjection.getP1().y(),
//                boxProjection.getP2().x(), boxProjection.getP2().y(),
//                boxProjection.getP3().x(), boxProjection.getP3().y(),
//                boxProjection.getP4().x(), boxProjection.getP4().y());
//
//
//        // --- CALCULATION ---
//        try {
//            RobotAdjustment adjustment = calculateRobotAdjustment(camera, box, boxProjection, imageCenter);
//            System.out.println("\n--- RESULTS ---");
//            System.out.printf("Required Robot Adjustment: %s\n", adjustment);
//            System.out.printf("-> Move arm by %.4f meters in X and %.4f meters in Y.\n", adjustment.offset().x(), adjustment.offset().y());
//            System.out.printf("-> Rotate gripper by %.2f degrees relative to the image's horizontal axis.\n", adjustment.rotationDegrees());
//        } catch (Exception e) {
//            System.err.println("Error during calculation: " + e.getMessage());
//            e.printStackTrace();
//        }
//    }
}

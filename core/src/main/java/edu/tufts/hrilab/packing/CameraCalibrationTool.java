package edu.tufts.hrilab.packing;

import ai.thinkingrobots.trade.*;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.List;

public class CameraCalibrationTool extends JFrame {
    private JButton loadImageButton;
    private JTextField wField, zField;
    private ComplicatedImagePanel originalImagePanel, correctedImagePanel;
    private JLabel infoLabel, resultLabel, thresholdLabel;
    private JSlider thresholdSlider;
    private BufferedImage originalImage;
    private double objectWidthW, distanceZ;
    
    // create a Camera class to store the instrinsic camera state
    private final ComplicatedCamera camera;
  private int itemNumber;

  public CameraCalibrationTool() {
        this.camera = new ComplicatedCamera();
        setTitle("Camera Calibration Tool - Drag corners to adjust");
        setSize(1400, 900);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLayout(new BorderLayout());

        // GUI setup
        JPanel topPanel = new JPanel(new BorderLayout());
        JPanel inputFieldPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
        inputFieldPanel.add(new JLabel("Object Width (W) in meters:"));
        wField = new JTextField(5);
        inputFieldPanel.add(wField);
        inputFieldPanel.add(new JLabel("Distance (Z) in meters:"));
        zField = new JTextField(5);
        inputFieldPanel.add(zField);
        loadImageButton = new JButton("Load Image");
        inputFieldPanel.add(loadImageButton);
        topPanel.add(inputFieldPanel, BorderLayout.NORTH);

        JPanel sliderPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
        thresholdLabel = new JLabel("Edge Threshold: 100");
        sliderPanel.add(thresholdLabel);
        thresholdSlider = new JSlider(0, 255, 100);
        thresholdSlider.setPreferredSize(new Dimension(300, 20));
        sliderPanel.add(thresholdSlider);
        topPanel.add(sliderPanel, BorderLayout.SOUTH);

        add(topPanel, BorderLayout.NORTH);

        JPanel imagesContainer = new JPanel(new GridLayout(1, 2));
        originalImagePanel = new ComplicatedImagePanel("Original Image (draggable corners)");
        originalImagePanel.setCornerUpdateCallback(this::recalculateFromCorners);
        
        correctedImagePanel = new ComplicatedImagePanel("Corrected Image");
        imagesContainer.add(originalImagePanel);
        imagesContainer.add(correctedImagePanel);
        add(imagesContainer, BorderLayout.CENTER);

        JPanel bottomPanel = new JPanel(new GridLayout(2, 1));
        infoLabel = new JLabel("Please provide object width, distance, and load an image.", SwingConstants.CENTER);
        resultLabel = new JLabel("Rotation and Shift will be shown here.", SwingConstants.CENTER);
        bottomPanel.add(infoLabel);
        bottomPanel.add(resultLabel);
        add(bottomPanel, BorderLayout.SOUTH);

        loadImageButton.addActionListener(e -> loadImage());
        thresholdSlider.addChangeListener(e -> {
            if (!thresholdSlider.getValueIsAdjusting()) {
                thresholdLabel.setText(String.format("Edge Threshold: %d", thresholdSlider.getValue()));
                if (originalImage != null) {
                    processImage();
                }
            }
        });
    }

    private void loadImage() {
      BufferedImage image = null;
      try {
        TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getImage"));
        image = tsi.call(BufferedImage.class);
      } catch (TRADEException e) {
        throw new RuntimeException(e);
      }
      originalImage = image;
      processImage();
    }

    private void processImage() {
        if (originalImage == null) return;
        infoLabel.setText("Processing...");
        int sobelThreshold = thresholdSlider.getValue();

        DetectionResult detectionResult = RectangleDetection.findRectangle(originalImage, sobelThreshold);

        if (detectionResult == null || detectionResult.corners == null) {
            infoLabel.setText("Could not detect a rectangle. Try adjusting the threshold or drag corners manually.");
            originalImagePanel.setImage(originalImage, null, null, null, detectionResult != null ? detectionResult.lines : null);
            correctedImagePanel.setImage(null, null, null, null, null);
            resultLabel.setText("Rotation and Shift will be shown here.");
            return;
        }

        recalculateFromCorners(detectionResult.corners);
        originalImagePanel.setImage(originalImage, new Vec2(originalImage.getWidth() / 2.0, originalImage.getHeight() / 2.0), Vec2.average(detectionResult.corners), detectionResult.corners, detectionResult.lines);
    }
    
    private void recalculateFromCorners(List<Vec2> corners) {
        if (originalImage == null || corners == null || corners.size() != 4) return;

        Vec2 rectangleCenter = Vec2.average(corners);
        
        Vec2 edgeA = corners.get(1).subtract(corners.get(0));
        Vec2 edgeB = corners.get(3).subtract(corners.get(0));
        Vec2 widthEdge = (edgeA.magnitudeSq() < edgeB.magnitudeSq()) ? edgeA : edgeB;
        double pixelWidthL = widthEdge.magnitude();
        
        double a = distanceZ * pixelWidthL / objectWidthW;
        double imageCenterX = originalImage.getWidth() / 2.0;
        double imageCenterY = originalImage.getHeight() / 2.0;

        double shiftX_pixels = rectangleCenter.x - imageCenterX;
        double shiftY_pixels = rectangleCenter.y - imageCenterY;

        double realWorldShiftX = shiftX_pixels * distanceZ / a;
        double realWorldShiftY = shiftY_pixels * distanceZ / a;

        double rotationAngleRad = Math.atan2(widthEdge.y, widthEdge.x);

        // update the instrinsic camera matrix K
        camera.updateIntrinsics(a, imageCenterX, imageCenterY);
        camera.updateAlignment(rotationAngleRad, realWorldShiftX, realWorldShiftY);
        
        // and display the results
        double focalLength = a * 0.036 / originalImage.getWidth();
        infoLabel.setText(String.format("Detected Rectangle. Pixel Width (L): %.2f. Calculated 'a': %.2f. Est. Focal Length: %.2f mm", pixelWidthL, a, focalLength*1000));
        resultLabel.setText(String.format("Real-World Shift: (%.4f m, %.4f m). Rotation: %.2f degrees.", realWorldShiftX, realWorldShiftY, Math.toDegrees(rotationAngleRad)));

        // output the final camera state and a sample pose calculation
        System.out.println("--- Camera State Updated ---");
        System.out.println(camera);
        
        /* Example:
        Camera.Vec3 currentPos = new Camera.Vec3(10, 5, 2);
        Camera.Vec3 currentRot = new Camera.Vec3(0, 0, Math.toRadians(15)); // 15 degrees yaw
        Camera.Pose newPose = camera.getAlignedCameraPose(currentPos, currentRot);
        System.out.println("\n--- Example Pose Calculation ---");
        System.out.println("Assuming Current Pose: " + new Camera.Pose(currentPos, currentRot));
        System.out.println("Calculated Aligned Pose: " + newPose);
        System.out.println("--------------------------\n");
	*/
	
        BufferedImage corrected = createCorrectedImage(originalImage, rectangleCenter, rotationAngleRad);
        correctedImagePanel.setImage(corrected, new Vec2(corrected.getWidth()/2.0, corrected.getHeight()/2.0), null, null, null);

        originalImagePanel.updateCorners(corners, rectangleCenter);
    }

    private BufferedImage createCorrectedImage(BufferedImage source, Vec2 center, double angleRad) {
        BufferedImage target = new BufferedImage(source.getWidth(), source.getHeight(), source.getType());
        Graphics2D g2d = target.createGraphics();
        g2d.setColor(Color.LIGHT_GRAY);
        g2d.fillRect(0, 0, target.getWidth(), target.getHeight());

        AffineTransform tx = new AffineTransform();
        tx.translate(target.getWidth() / 2.0, target.getHeight() / 2.0);
        tx.rotate(-angleRad);
        tx.translate(-center.x, -center.y);
        
        g2d.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
        g2d.drawImage(source, tx, null);
        g2d.dispose();
        
        return target;
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> new CameraCalibrationTool().setVisible(true));
    }
}

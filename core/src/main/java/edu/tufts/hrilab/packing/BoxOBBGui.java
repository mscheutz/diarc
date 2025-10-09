/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025
 */

package edu.tufts.hrilab.packing;

import javax.swing.*;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.util.List;

public class BoxOBBGui extends JFrame {

  private final JSlider lowerThresholdSlider;
  private final JSlider upperThresholdSlider;
  private final JLabel lowerThresholdLabel;
  private final JLabel upperThresholdLabel;

  private int upperThreshold;
  private int lowerThreshold;

  // Updated record to hold the residual angle for verification
  public record ProcessResult(BufferedImage debugImage, BufferedImage rotatedImage, double angle, double residualAngle,
                              boolean success) {
  }

  private final ImagePanel originalImagePanel;
  private final ImagePanel debugImagePanel;
  private final ImagePanel rotatedImagePanel;
  private final JLabel angleLabel;
  private final TitledBorder rotatedImageBorder; // To update the title
  private final JPanel rotatedPanelContainer; // The panel that owns the border

  public BoxOBBGui(int lowerThreshold, int upperThreshold) {

    // --- Initialize UI components ---
    originalImagePanel = new ImagePanel();
    debugImagePanel = new ImagePanel();
    rotatedImagePanel = new ImagePanel();

    angleLabel = new JLabel("Angle: 0.00°");
    angleLabel.setFont(new Font("Arial", Font.BOLD, 16));
    angleLabel.setHorizontalAlignment(SwingConstants.CENTER);

    this.lowerThreshold = lowerThreshold;
    this.upperThreshold =  upperThreshold;

    lowerThresholdLabel = new JLabel("Lower Threshold: " + lowerThreshold);
    lowerThresholdSlider = new JSlider(0, 255, lowerThreshold);

    upperThresholdLabel = new JLabel("Upper Threshold: " + upperThreshold);
    upperThresholdSlider = new JSlider(0, 255, upperThreshold);

    // --- Layout the components ---
    JPanel imagePanels = new JPanel(new GridLayout(1, 3, 10, 10));
    imagePanels.add(createTitledPanel("Original Image", originalImagePanel));
    imagePanels.add(createTitledPanel("Debug View", debugImagePanel));

    // Create a titled border that we can reference later to update the text
    rotatedImageBorder = BorderFactory.createTitledBorder("Rotated Image (Aligned)");
    rotatedPanelContainer = new JPanel(new BorderLayout()); // Use the class field
    rotatedPanelContainer.setBorder(rotatedImageBorder);
    rotatedPanelContainer.add(rotatedImagePanel, BorderLayout.CENTER);
    imagePanels.add(rotatedPanelContainer);

    JPanel slidersPanel = new JPanel(new GridLayout(2, 2));
    slidersPanel.add(lowerThresholdLabel);
    slidersPanel.add(upperThresholdLabel);
    slidersPanel.add(lowerThresholdSlider);
    slidersPanel.add(upperThresholdSlider);

    lowerThresholdSlider.addChangeListener(e -> {
      if (lowerThresholdSlider.getValue() > upperThresholdSlider.getValue()) {
        upperThresholdSlider.setValue(lowerThresholdSlider.getValue());
      }
      updateThresholdLabels();
      this.lowerThreshold = lowerThresholdSlider.getValue();
    });

    upperThresholdSlider.addChangeListener(e -> {
      if (upperThresholdSlider.getValue() < lowerThresholdSlider.getValue()) {
        lowerThresholdSlider.setValue(upperThresholdSlider.getValue());
      }
      updateThresholdLabels();
      this.upperThreshold =  upperThresholdSlider.getValue();
    });

    JPanel controlPanel = new JPanel(new BorderLayout(10, 10));
    controlPanel.add(slidersPanel, BorderLayout.CENTER);
    controlPanel.add(angleLabel, BorderLayout.SOUTH);
    controlPanel.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));

    setLayout(new BorderLayout());
    add(imagePanels, BorderLayout.CENTER);
    add(controlPanel, BorderLayout.SOUTH);

  }

  private void updateThresholdLabels() {
    lowerThresholdLabel.setText("Lower Luma (Y): " + lowerThresholdSlider.getValue());
    upperThresholdLabel.setText("Upper Luma (Y): " + upperThresholdSlider.getValue());
  }

  private JPanel createTitledPanel(String title, JComponent component) {
    JPanel panel = new JPanel(new BorderLayout());
    panel.setBorder(BorderFactory.createTitledBorder(title));
    panel.add(component, BorderLayout.CENTER);
    return panel;
  }

  public void update(BufferedImage image, List<Vector2d> largestComponent, Point2d[] boundingBox, double angleDegrees) {
    Object[] rotatedStuff = createRotatedImage(image, angleDegrees, boundingBox);
    ProcessResult result = new ProcessResult(createDebugImage(image, largestComponent, boundingBox), (BufferedImage) rotatedStuff[0], angleDegrees, (double) rotatedStuff[1], true);
    originalImagePanel.setImage(image);
    try {
      if (result.success) {
        angleLabel.setText(String.format("Angle: %.2f°", result.angle));
        rotatedImageBorder.setTitle(String.format("Rotated Image (Alignment: %.2f°)", result.residualAngle));
        rotatedPanelContainer.repaint(); // FIX: Repaint the container, not the inner panel
        debugImagePanel.setImage(result.debugImage);
        rotatedImagePanel.setImage(result.rotatedImage);
      } else {
        angleLabel.setText("Angle: N/A");
        rotatedImageBorder.setTitle("Rotated Image (Aligned)");
        rotatedPanelContainer.repaint(); // FIX: Repaint the container
        debugImagePanel.setImage(createBlankImageWithMessage("No object found"));
        rotatedImagePanel.setImage(createBlankImageWithMessage(""));
      }
    } catch (Exception e) {
      e.printStackTrace();
      angleLabel.setText("Error");
      rotatedImageBorder.setTitle("Rotated Image (Error)");
      rotatedPanelContainer.repaint(); // FIX: Repaint the container
      debugImagePanel.setImage(createBlankImageWithMessage("Processing Error"));
    } finally {
      setCursor(Cursor.getDefaultCursor());
      this.repaint();
    }
  }

  private BufferedImage createBlankImageWithMessage(String message) {
    BufferedImage img = new BufferedImage(480, 320, BufferedImage.TYPE_INT_ARGB);
    Graphics2D g = img.createGraphics();
    g.setColor(Color.LIGHT_GRAY);
    g.fillRect(0, 0, img.getWidth(), img.getHeight());
    g.setColor(Color.DARK_GRAY);
    g.setFont(new Font("Arial", Font.BOLD, 14));
    FontMetrics fm = g.getFontMetrics();
    int stringWidth = fm.stringWidth(message);
    g.drawString(message, (img.getWidth() - stringWidth) / 2, img.getHeight() / 2);
    g.dispose();
    return img;
  }

  private BufferedImage createDebugImage(BufferedImage originalImage, List<Vector2d> component, Point2d[] boundingBox) {
    BufferedImage img = new BufferedImage(originalImage.getWidth(), originalImage.getHeight(), BufferedImage.TYPE_INT_ARGB);
    Graphics2D g = img.createGraphics();
    g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
    g.drawImage(originalImage, 0, 0, null);

    g.setColor(new Color(255, 0, 0, 100));
    for (Vector2d p : component) {
      g.fillRect((int) p.x, (int) p.y, 1, 1);
    }

    g.setColor(Color.CYAN);
    g.setStroke(new BasicStroke(2));
    for (int i = 0; i < 4; i++) {
      g.drawLine((int) boundingBox[i].x, (int)boundingBox[i].y, (int)boundingBox[(i + 1) % 4].x, (int)boundingBox[(i + 1) % 4].y);
    }

    g.dispose();
    return img;
  }

  private Object[] createRotatedImage(BufferedImage originalImage, double angleDegrees, Point2d[] boundingBox) {
    double angleRad = Math.toRadians(-angleDegrees);
    double sin = Math.abs(Math.sin(angleRad));
    double cos = Math.abs(Math.cos(angleRad));
    int w = originalImage.getWidth();
    int h = originalImage.getHeight();
    int newW = (int) Math.floor(w * cos + h * sin);
    int newH = (int) Math.floor(h * cos + w * sin);

    BufferedImage rotated = new BufferedImage(newW, newH, originalImage.getType());
    Graphics2D g = rotated.createGraphics();
    g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
    g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);

    AffineTransform at = new AffineTransform();
    at.translate((newW - w) / 2.0, (newH - h) / 2.0);
    at.rotate(angleRad, w / 2.0, h / 2.0);
    g.transform(at);

    g.drawImage(originalImage, 0, 0, null);

    Point2D[] srcPoints = new Point2D[4];
    for (int i = 0; i < 4; i++) {
      Point2D point = new Point2D.Double();
      point.setLocation(boundingBox[i].x, boundingBox[i].y);
      srcPoints[i] = point;
    }
    Point2D[] destPoints = new Point2D[4];
    at.transform(srcPoints, 0, destPoints, 0, 4);

    g.setTransform(new AffineTransform());
    g.setColor(Color.MAGENTA);
    g.setStroke(new BasicStroke(2));
    for (int i = 0; i < 4; i++) {
      g.drawLine((int) destPoints[i].getX(), (int) destPoints[i].getY(), (int) destPoints[(i + 1) % 4].getX(), (int) destPoints[(i + 1) % 4].getY());
    }

    // Calculate the residual angle of the now-aligned box
    Point2D p1 = destPoints[0];
    Point2D p2 = destPoints[1];
    Point2D p3 = destPoints[3];
    double side1Sq = p1.distanceSq(p2);
    double side2Sq = p1.distanceSq(p3);

    double dx, dy;
    if (side1Sq > side2Sq) { // p1-p2 is the longer side
      dx = p2.getX() - p1.getX();
      dy = p2.getY() - p1.getY();
    } else { // p1-p3 is the longer side
      dx = p3.getX() - p1.getX();
      dy = p3.getY() - p1.getY();
    }
    double residualAngle = Math.toDegrees(Math.atan2(dy, dx));

    g.dispose();
    return new Object[]{rotated, residualAngle};
  }

  public int getUpperThreshold() {
    return upperThreshold;
  }

  public int getLowerThreshold() {
    return lowerThreshold;
  }
}


class ImagePanel extends JPanel {
  private BufferedImage image;
  public void setImage(BufferedImage image) { this.image = image; repaint(); }
  @Override
  protected void paintComponent(Graphics g) {
    super.paintComponent(g);
    if (image != null) {
      int panelW = getWidth(); int panelH = getHeight();
      double panelAspect = (double) panelW / panelH;
      double imageAspect = (double) image.getWidth() / image.getHeight();
      int drawW, drawH, drawX, drawY;
      if (panelAspect > imageAspect) {
        drawH = panelH; drawW = (int) (drawH * imageAspect);
        drawX = (panelW - drawW) / 2; drawY = 0;
      } else {
        drawW = panelW; drawH = (int) (drawW / imageAspect);
        drawX = 0; drawY = (panelH - drawH) / 2;
      }
      g.drawImage(image, drawX, drawY, drawW, drawH, null);
    }
  }
}

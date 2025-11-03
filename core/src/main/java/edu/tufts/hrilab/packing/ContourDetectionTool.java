package edu.tufts.hrilab.packing;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.packing.vision.PackingCameraInfo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class ContourDetectionTool extends JFrame {
  private final RobotVisionProcessor visionProcessor;
  private JButton startStopCameraButton;
  private JButton setAlphaButton;
  //private JButton saveImageButton;
  private JLabel lastCalibratedLabel;
  private ContourImagePanel originalPanel, correctedPanel;
  private JLabel infoLabel, resultLabel, dimensionsLabel;
  private JSlider lowThresholdSlider, highThresholdSlider, dilationSlider;
  private JLabel wField, lField;
  private BufferedImage sourceImage;
  private ContourCamera camera;
  private int selectedCamera = -1;
  private boolean capture;
  private int imgNum = 0;
  private JComboBox<PackingCameraInfo> cameraSelectionBox;
  private static final Logger log =  LoggerFactory.getLogger(ContourDetectionTool.class);
  private double alpha = -1;

  public ContourDetectionTool(RobotVisionProcessor robotVisionProcessor) {
    this.visionProcessor = robotVisionProcessor;
    this.camera = new ContourCamera();
    setTitle("Interactive Camera Calibration Tool");
    setSize(1600, 900);
    setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    setLayout(new BorderLayout());

    JPanel topPanel = new JPanel();
    topPanel.setLayout(new BoxLayout(topPanel, BoxLayout.Y_AXIS));

    JPanel mainControlsPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));


    int defaultCamera = robotVisionProcessor.defaultCamera;
    this.selectedCamera = defaultCamera;
    setCamera(selectedCamera);

    cameraSelectionBox = new JComboBox<>();
    cameraSelectionBox.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        JComboBox cb = (JComboBox) e.getSource();
        PackingCameraInfo selected = (PackingCameraInfo) cb.getSelectedItem();
        selectedCamera = selected.id();
        setCamera(selected.id());
      }
    });
    mainControlsPanel.add(cameraSelectionBox);

    startStopCameraButton = new JButton("Start/Stop Camera");
    mainControlsPanel.add(startStopCameraButton);
    mainControlsPanel.add(new JSeparator(SwingConstants.VERTICAL));
    mainControlsPanel.add(new JLabel("Box W (mm):"));
    //wField = new JTextField(String.valueOf(boxWidthMeters*1000), 5);  // default width of box
    wField = new JLabel(String.valueOf(robotVisionProcessor.getBoxWidthMeters()*1000));  // default width of box
    mainControlsPanel.add(wField);
    mainControlsPanel.add(new JLabel("L (mm):"));
    //lField = new JTextField(String.valueOf(boxLengthMeters*1000), 5);  // default length of box
    lField = new JLabel(String.valueOf(robotVisionProcessor.getBoxLengthMeters()*1000));  // default length of box
    mainControlsPanel.add(lField);
    setAlphaButton = new JButton("Set Calibration");
    //saveImageButton = new JButton("Save Image");
    if (robotVisionProcessor.getA() != -1) {
      lastCalibratedLabel = new JLabel(String.format("Last Calibrated with (Low/High/Iter): %d, %d, %d", robotVisionProcessor.getCannyLowThreshold(), robotVisionProcessor.getCannyHighThreshold(), robotVisionProcessor.getDilationIterations()));
    } else {
      lastCalibratedLabel = new JLabel(String.format("Not Calibrated, hit set calibration when a box has been detected"));
    }

    JPanel slidersPanel = new JPanel(new FlowLayout(FlowLayout.LEFT, 10, 5));
    // default canny edge detection values that worked on all the test images
    lowThresholdSlider = createSlider(0, 255, robotVisionProcessor.getCannyLowThreshold(), "Canny Low Threshold");
    highThresholdSlider = createSlider(0, 255, robotVisionProcessor.getCannyHighThreshold(), "Canny High Threshold");
    dilationSlider = createSlider(0, 10, robotVisionProcessor.getDilationIterations(), "Dilation Iterations");
    slidersPanel.add(lowThresholdSlider.getParent());
    slidersPanel.add(highThresholdSlider.getParent());
    slidersPanel.add(dilationSlider.getParent());
    slidersPanel.add(setAlphaButton);
    //slidersPanel.add(saveImageButton);
    slidersPanel.add(lastCalibratedLabel);

    lowThresholdSlider.addChangeListener(e -> visionProcessor.setCannyLowThreshold(lowThresholdSlider.getValue()));
    highThresholdSlider.addChangeListener(e -> visionProcessor.setCannyHighThreshold(highThresholdSlider.getValue()));
    dilationSlider.addChangeListener(e -> visionProcessor.setDilationIterations(dilationSlider.getValue()));

    topPanel.add(mainControlsPanel);
    topPanel.add(slidersPanel);
    add(topPanel, BorderLayout.NORTH);

    JPanel bottomPanel = new JPanel(new GridLayout(2, 1));
    infoLabel = new JLabel("Load an image to begin processing.");
    resultLabel = new JLabel("Calibration results will appear here.");
    dimensionsLabel = new JLabel("Detected dimensions (px) will appear here.");
    bottomPanel.add(infoLabel);
    bottomPanel.add(dimensionsLabel);
    bottomPanel.add(resultLabel);
    add(bottomPanel, BorderLayout.SOUTH);

    JPanel imagesContainer = new JPanel(new GridLayout(1, 2));
    originalPanel = new ContourImagePanel("Original Image + Detection");
    correctedPanel = new ContourImagePanel("Corrected View");
    imagesContainer.add(originalPanel);
    imagesContainer.add(correctedPanel);
    add(imagesContainer, BorderLayout.CENTER);

    startStopCameraButton.addActionListener(e -> startCameraCapture());

    setAlphaButton.addActionListener(e -> setAlpha());
    //saveImageButton.addActionListener(e -> saveImage());

    Timer timer = new Timer(500, e -> {
      if (capture) {
        try {
          TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getImage"));
          sourceImage = tsi.call(BufferedImage.class);
          processImage();
        } catch (TRADEException ex) {
          throw new RuntimeException(ex);
        }
      }
    });

    timer.start();

    try {
      TRADE.registerAllServices(this, "");
      Collection<TRADEServiceInfo> services = TRADE.getAvailableServices(new TRADEServiceConstraints().name("getAvailableCameras"));
      if (services.isEmpty()) {
        TRADE.requestNotification(this, "joined", new TRADEServiceConstraints().name("getAvailableCameras"),null, "getAvailableCamerasCallback");
      } else {
        TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAvailableCameras"));
        getAvailableCamerasCallback(tsi);
      }
    } catch (TRADEException e) {
      log.error("[init] error registering for TRADE notification for getAvailableCameras join", e);
    }
  }

  private void setAlpha() {
    if (visionProcessor != null && alpha != -1) {
      visionProcessor.setA(alpha);
    }
    //if (alpha != -1) {
    //  try {
    //    TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("setAlpha"));
    //    tsi.call(void.class, alpha);
    //  } catch (TRADEException e) {
    //    log.error("[setAlpha] error calling trade service to set value of constant value a", e);
    //  }
    //} else {
    //  log.warn("[setAlpha] TODO: display error message that no box has been detected");
    //}
  }

  private void setCamera(int selectedCamera){
    boolean result;
    try {
      TRADEServiceInfo tsi =  TRADE.getAvailableService(new TRADEServiceConstraints().name("setCamera").argTypes(Integer.class));
      result = tsi.call(boolean.class, selectedCamera);
    } catch (TRADEException ex) {
      result = false;
      log.error("[setCamera] error setting camera: ", ex);
    }
    if (!result) {
      JOptionPane.showMessageDialog(this, "Error setting camera!", "Error", JOptionPane.ERROR_MESSAGE);
    }
  }

  @TRADEService
  public void getAvailableCamerasCallback(TRADEServiceInfo tsi) {
    List<PackingCameraInfo> camerasList = new ArrayList<>();
    try {
      camerasList = tsi.call(List.class);
    } catch (TRADEException e) {
      log.error("[getAvailableCameras] error calling TRADE service to get available cameras.", e);
    }
    int previouslySetCamera = selectedCamera;
    int i = 0;
    int id = -1;
    for (PackingCameraInfo packingCameraInfo : camerasList) {
      cameraSelectionBox.addItem(packingCameraInfo);
      if (packingCameraInfo.id() == previouslySetCamera) {
        id = i;
      }
      i++;
    }
    if (id != -1) {
      cameraSelectionBox.setSelectedIndex(id);
    }
    log.warn("[getAvailableCameras] cameras were found: " + camerasList);
    SwingUtilities.invokeLater(() -> {
      cameraSelectionBox.repaint();
      this.setVisible(true);
    });
  }

  private void saveImage() {
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getImage"));
      sourceImage = tsi.call(BufferedImage.class);
      processImage();
    } catch (TRADEException ex) {
      throw new RuntimeException(ex);
    }
    try {
      ImageIO.write(sourceImage, "png", new File("/home/pete/matthias/" + ++imgNum + ".png"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  private void startCameraCapture() {
    capture = !capture;
  }


  private JSlider createSlider(int min, int max, int value, String title) {
    JSlider slider = new JSlider(min, max, value);
    slider.setMajorTickSpacing(Math.max((max - min) / 5, 1));
    slider.setPaintTicks(true);
    slider.setPaintLabels(true);
    JPanel panel = new JPanel(new BorderLayout());
    panel.setBorder(BorderFactory.createTitledBorder(title));
    panel.add(slider, BorderLayout.CENTER);
    slider.addChangeListener(e -> {
      if (!slider.getValueIsAdjusting() && sourceImage != null) {
        processImage();
      }
    });
    return slider;
  }

  private void loadImage() {
    JFileChooser fileChooser = new JFileChooser();
    if (fileChooser.showOpenDialog(this) == JFileChooser.APPROVE_OPTION) {
      try {
        sourceImage = ImageIO.read(fileChooser.getSelectedFile());
        processImage();
      } catch (Exception ex) {
        JOptionPane.showMessageDialog(this, "Error: " + ex.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
      }
    }
  }

  private void processImage() {
    if (sourceImage == null) {
      return;
    }
    int lowT = lowThresholdSlider.getValue();
    int highT = highThresholdSlider.getValue();
    int dilations = dilationSlider.getValue();
    infoLabel.setText(String.format("Processing with LowT: %d, HighT: %d, Dilations: %d", lowT, highT, dilations));
    paint(getGraphics());

    List<Point> minRectangle = ContourDetector.findLargestObjectRectangle(sourceImage, lowT, highT, dilations);

    if (minRectangle != null) {
      recalculateFromCorners(minRectangle);
    } else {
      this.alpha = -1;
      infoLabel.setText(String.format("Could not find object. LowT: %d, HighT: %d, Dilations: %d", lowT, highT, dilations));
      originalPanel.setImage(sourceImage, null, null, null);
      correctedPanel.setImage(null, null, null, null);
    }
  }

  private void recalculateFromCorners(List<Point> corners) {
    double objectWidthW, objectLengthL, distanceZ;
    try {
      objectWidthW = Double.parseDouble(wField.getText()) / 1000;
      objectLengthL = Double.parseDouble(lField.getText()) / 1000;
    } catch (NumberFormatException ex) {
      infoLabel.setText("Invalid dimensions (W, L, Z). Please enter valid numbers.");
      return;
    }

    Point p0 = corners.get(0), p1 = corners.get(1), p3 = corners.get(3);
    double edge1 = p0.distance(p1);
    double edge2 = p0.distance(p3);

    // get the pixel width and length of the box
    double pixelWidth = Math.min(edge1, edge2);
    double pixelLength = Math.max(edge1, edge2);
    double realWidth = Math.min(objectWidthW, objectLengthL);

    Point widthEdgeP1 = (edge1 < edge2) ? p0 : p3;
    Point widthEdgeP2 = (edge1 < edge2) ? p1 : p0;

    // calculate scaling factor 'a'
    double a = pixelWidth / realWidth;

    // calculate center points
    double imageCenterX = sourceImage.getWidth() / 2.0;
    double imageCenterY = sourceImage.getHeight() / 2.0;
    double rectCenterX = (corners.get(0).x + corners.get(1).x + corners.get(2).x + corners.get(3).x) / 4.0;
    double rectCenterY = (corners.get(0).y + corners.get(1).y + corners.get(2).y + corners.get(3).y) / 4.0;

    // calculate shift in meters
    double shiftX_pixels = rectCenterX - imageCenterX;
    double shiftY_pixels = rectCenterY - imageCenterY;
    double realWorldShiftX = shiftX_pixels / a;
    double realWorldShiftY = shiftY_pixels / a;

    // calculate rotation in degrees
    double rotationAngleRad = Math.atan2(widthEdgeP2.y - widthEdgeP1.y, widthEdgeP2.x - widthEdgeP1.x);

    // and update the camera object and labels
    camera.updateIntrinsics(a, imageCenterX, imageCenterY);
    camera.updateAlignment(rotationAngleRad, realWorldShiftX, realWorldShiftY);
    resultLabel.setText(String.format("Shift: (%.4f, %.4f) m. Rotation: %.2f deg. 'a' factor: %.2f", realWorldShiftX, realWorldShiftY, Math.toDegrees(rotationAngleRad), a));
    dimensionsLabel.setText(String.format("Width (px): %.4f.", pixelWidth));
    System.out.println("--- Camera State Updated ---\n" + camera + "\n--------------------------\n");

    // also update image panels
    originalPanel.setImage(sourceImage, corners, new Point((int) imageCenterX, (int) imageCenterY), new Point((int) rectCenterX, (int) rectCenterY));
    BufferedImage corrected = createCorrectedImage(sourceImage, new Point((int) rectCenterX, (int) rectCenterY), rotationAngleRad);
    correctedPanel.setImage(corrected, null, new Point(corrected.getWidth() / 2, corrected.getHeight() / 2), null);
    this.alpha = a;
  }

  protected void update(BufferedImage sourceImage, List<Point> corners, double imageCenterX, double imageCenterY, double rectCenterX, double rectCenterY, double rotationAngleRad, double a, double aCalculated, double realWorldShiftX, double realWorldShiftY, double pixelWidth) {
    camera.updateIntrinsics(a, imageCenterX, imageCenterY);
    camera.updateAlignment(rotationAngleRad, realWorldShiftX, realWorldShiftY);
    resultLabel.setText(String.format("Shift: (%.4f, %.4f) m. Rotation: %.2f radians. 'a' factor: %.2f. 'a calculated': %.2f", realWorldShiftX, realWorldShiftY, rotationAngleRad, a, aCalculated));
    originalPanel.setImage(sourceImage, corners, new Point((int) imageCenterX, (int) imageCenterY), new Point((int) rectCenterX, (int) rectCenterY));
    BufferedImage corrected = createCorrectedImage(sourceImage, new Point((int) rectCenterX, (int) rectCenterY), rotationAngleRad);
    correctedPanel.setImage(corrected, null, new Point(corrected.getWidth() / 2, corrected.getHeight() / 2), null);
  }

  private BufferedImage createCorrectedImage(BufferedImage source, Point center, double angleRad) {
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

  public int getSelectedCamera() {
    return selectedCamera;
  }

  public void setWFieldValue(double value) {
    wField.setText(String.valueOf(value));
  }

  public void setLFieldValue(double value) {
    lField.setText(String.valueOf(value));
  }

  public void setCalibrationValues(int cannyLowThreshold, int cannyHighThreshold, int dilationIterations) {
    lastCalibratedLabel.setText(String.format("Last Calibrated with (Low/High/Iter): %d, %d, %d", cannyLowThreshold, cannyHighThreshold, dilationIterations));
  }
}

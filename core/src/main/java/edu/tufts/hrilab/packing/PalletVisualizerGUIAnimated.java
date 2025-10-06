package edu.tufts.hrilab.packing;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

/**
 * visualization of the results from the PalletPacker algorithm
 */
public class PalletVisualizerGUIAnimated extends JFrame {

  private final JTextField palletLField, palletWField, palletHField, rectLField, rectWField, rectHField;
  private final JLabel resultLabel;
  private final JTextArea bottomLayerInfoArea;
  private final DrawingPanel drawingPanel;
  private final JButton calculateButton, animateButton, pauseResumeButton, endAnimationButton;
  private PackingResult3D lastResult;
  private Timer animationStateTimer;

  public PalletVisualizerGUIAnimated() {
    setTitle("Pallet Packer Visualizer");
    setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    setLayout(new BorderLayout(10, 10));

    // --- Input Panel ---
    JPanel inputPanel = new JPanel(new GridLayout(0, 2, 5, 5));
    inputPanel.setBorder(BorderFactory.createTitledBorder("Dimensions"));
    palletLField = new JTextField("50");
    palletWField = new JTextField("40");
    palletHField = new JTextField("18");
    rectLField = new JTextField("7");
    rectWField = new JTextField("11");
    rectHField = new JTextField("5");

    inputPanel.add(new JLabel("Pallet Length (L):"));
    inputPanel.add(palletLField);
    inputPanel.add(new JLabel("Pallet Width (W):"));
    inputPanel.add(palletWField);
    inputPanel.add(new JLabel("Pallet Height (H):"));
    inputPanel.add(palletHField);
    inputPanel.add(new JSeparator());
    inputPanel.add(new JSeparator());
    inputPanel.add(new JLabel("Item Length (l):"));
    inputPanel.add(rectLField);
    inputPanel.add(new JLabel("Item Width (w):"));
    inputPanel.add(rectWField);
    inputPanel.add(new JLabel("Item Height (h):"));
    inputPanel.add(rectHField);

    // --- Button Panel ---
    calculateButton = new JButton("Calculate & Visualize");
    animateButton = new JButton("Animate");
    pauseResumeButton = new JButton("Pause");
    endAnimationButton = new JButton("End Animation");

    JPanel buttonPanel = new JPanel(new FlowLayout(FlowLayout.CENTER));
    buttonPanel.add(calculateButton);
    buttonPanel.add(animateButton);
    buttonPanel.add(pauseResumeButton);
    buttonPanel.add(endAnimationButton);

    // --- Result Panel ---
    JPanel resultPanel = new JPanel(new BorderLayout(5, 5));
    resultLabel = new JLabel("Enter dimensions and click calculate.");
    resultLabel.setHorizontalAlignment(SwingConstants.CENTER);

    bottomLayerInfoArea = new JTextArea(5, 40);
    bottomLayerInfoArea.setEditable(false);
    bottomLayerInfoArea.setFont(new Font("Monospaced", Font.PLAIN, 12));
    bottomLayerInfoArea.setBorder(BorderFactory.createTitledBorder("2d item layout (repeated in every layer)"));
    resultPanel.add(resultLabel, BorderLayout.NORTH);
    resultPanel.add(new JScrollPane(bottomLayerInfoArea), BorderLayout.CENTER);

    // --- Top Panel using GridBagLayout for proper structure ---
    JPanel topPanel = new JPanel(new GridBagLayout());
    GridBagConstraints gbc = new GridBagConstraints();
    gbc.gridx = 0; gbc.gridy = 0; gbc.weightx = 1.0; gbc.fill = GridBagConstraints.HORIZONTAL; gbc.insets = new Insets(5, 5, 5, 5);
    topPanel.add(inputPanel, gbc);
    gbc.gridy++;
    topPanel.add(buttonPanel, gbc);
    gbc.gridy++; gbc.weighty = 1.0; gbc.fill = GridBagConstraints.BOTH;
    topPanel.add(resultPanel, gbc);

    // --- Main Layout ---
    drawingPanel = new DrawingPanel();
    add(topPanel, BorderLayout.NORTH);
    add(new JScrollPane(drawingPanel), BorderLayout.CENTER);

    // --- Action Listeners ---
    calculateButton.addActionListener(e -> runCalculation());
    animateButton.addActionListener(e -> runAnimation());
    pauseResumeButton.addActionListener(e -> togglePauseResume());
    endAnimationButton.addActionListener(e -> endAnimation());

    setAnimationControlsEnabled(false);

    setSize(800, 900);
    setLocationRelativeTo(null);
  }

  private void setAnimationControlsEnabled(boolean enabled) {
    pauseResumeButton.setEnabled(enabled);
    endAnimationButton.setEnabled(enabled);
    calculateButton.setEnabled(!enabled);
    animateButton.setEnabled(!enabled && lastResult != null && lastResult.totalCount > 0);
  }

  private void runCalculation() {
    if (animationStateTimer != null && animationStateTimer.isRunning()) {
      animationStateTimer.stop();
    }
    drawingPanel.stopAnimation();
    try {
      int palletL = Integer.parseInt(palletLField.getText());
      int palletW = Integer.parseInt(palletWField.getText());
      int palletH = Integer.parseInt(palletHField.getText());
      int rectL = Integer.parseInt(rectLField.getText());
      int rectW = Integer.parseInt(rectWField.getText());
      int rectH = Integer.parseInt(rectHField.getText());

      resultLabel.setText("Calculating...");
      bottomLayerInfoArea.setText("");
      setCursor(Cursor.getPredefinedCursor(Cursor.WAIT_CURSOR));
      animateButton.setEnabled(false);

      new SwingWorker<PackingResult3D, Void>() {
        @Override
        protected PackingResult3D doInBackground() {
          PalletPackingComponent packer = new PalletPackingComponent();
          return packer.solvePalletPacking3D(palletL, palletW, palletH, rectL, rectW, rectH);
        }

        @Override
        protected void done() {
          try {
            lastResult = get();
            String resultText;
            if (lastResult.totalCount > 0) {
              resultText = String.format("Max items: %d (%d items/layer x %d layers). Item dimension is %d x %d.", lastResult.totalCount, lastResult.layerPacking.count, lastResult.numLayers, rectL, rectW);
              printBottomLayerInfo(lastResult.allItems);
              animateButton.setEnabled(true);
            } else {
              resultText = "No items fit on the pallet.";
              bottomLayerInfoArea.setText("N/A");
              lastResult = null;
            }
            resultLabel.setText(resultText);
            drawingPanel.setPackingResult(palletL, palletW, lastResult != null ? lastResult.layerPacking.rectangles : Collections.emptyList());
          } catch (Exception e) {
            resultLabel.setText("An error occurred during calculation.");
            e.printStackTrace();
          } finally {
            setCursor(Cursor.getDefaultCursor());
          }
        }
      }.execute();
    } catch (NumberFormatException ex) {
      JOptionPane.showMessageDialog(this, "Please enter valid integers for all dimensions.", "Input Error", JOptionPane.ERROR_MESSAGE);
    }
  }

  private void runAnimation() {
    if (lastResult == null || lastResult.totalCount == 0) return;

    setAnimationControlsEnabled(true);
    pauseResumeButton.setText("Pause");
    drawingPanel.startAnimation(lastResult.allItems, Integer.parseInt(palletLField.getText()), Integer.parseInt(palletWField.getText()));

    animationStateTimer = new Timer(100, e -> {
      if (!drawingPanel.isAnimationRunning()) {
        endAnimation();
      }
    });
    animationStateTimer.start();
  }

  private void togglePauseResume() {
    if (drawingPanel.isPaused()) {
      drawingPanel.resumeAnimation();
      pauseResumeButton.setText("Pause");
    } else {
      drawingPanel.pauseAnimation();
      pauseResumeButton.setText("Resume");
    }
  }

  private void endAnimation() {
    if (animationStateTimer != null) animationStateTimer.stop();
    drawingPanel.stopAnimation();
    setAnimationControlsEnabled(false);
    pauseResumeButton.setText("Pause");
    if (lastResult != null) {
      drawingPanel.setPackingResult(Integer.parseInt(palletLField.getText()), Integer.parseInt(palletWField.getText()), lastResult.layerPacking.rectangles);
    }
  }

  private void printBottomLayerInfo(List<PackedItem> allItems) {
    List<PackedItem> bottomLayerItems = allItems.stream().filter(item -> item.layer == 1).collect(Collectors.toList());
    StringBuilder sb = new StringBuilder();
    sb.append(String.format("Found %d items for the first layer:\n", bottomLayerItems.size()));
    for (PackedItem item : bottomLayerItems) {
      sb.append(item.toString()).append("\n");
    }
    bottomLayerInfoArea.setText(sb.toString());
  }

  private static class DrawingPanel extends JPanel {
    private int palletL = 1, palletW = 1;
    private List<Rectangle> packedRectangles;

    private enum AnimState { IDLE, APPROACHING_PICKUP, PICKING, MOVING_TO_PALLET, PLACING, FINISHED }
    private Timer animationTimer;
    private List<PackedItem> allItems;
    private int currentItemIndex, currentLayer;
    private List<Rectangle> placedItemsInCurrentLayer;
    private Rectangle movingItem;
    private PackedItem currentPackedItem;

    private Point2D.Double armShoulder, armElbow, armGripper;
    private Point2D.Double conveyorStart, conveyorEnd;

    private AnimState animState = AnimState.IDLE;
    private Point2D.Double itemAnimFrom, itemAnimTo, armAnimFrom, armAnimTo;
    private int animFrame, animTotalFrames;
    private boolean isPaused = false;

    DrawingPanel() {
      setBackground(Color.WHITE);
      setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
      armShoulder = new Point2D.Double(); armElbow = new Point2D.Double(); armGripper = new Point2D.Double();
    }

    public void setPackingResult(int palletL, int palletW, List<Rectangle> rectangles) {
      stopAnimation();
      this.palletL = palletL; this.palletW = palletW;
      this.packedRectangles = new ArrayList<>(rectangles);
      this.allItems = null; this.animState = AnimState.IDLE;
      revalidate(); repaint();
    }

    public void stopAnimation() {
      if (animationTimer != null) animationTimer.stop();
      isPaused = false; animState = AnimState.FINISHED;
    }

    public void pauseAnimation() { if(animationTimer != null) animationTimer.stop(); isPaused = true; }
    public void resumeAnimation() { if(animationTimer != null) animationTimer.start(); isPaused = false; }
    public boolean isPaused() { return isPaused; }

    public void startAnimation(List<PackedItem> allItems, int palletL, int palletW) {
      stopAnimation();
      this.palletL = palletL; this.palletW = palletW;
      this.allItems = allItems.stream()
              .sorted(Comparator.comparingInt((PackedItem i) -> i.layer)
                      .thenComparingInt((PackedItem i) -> i.rect.y)
                      .thenComparingInt((PackedItem i) -> i.rect.x))
              .collect(Collectors.toList());
      this.currentItemIndex = 0; this.currentLayer = 1;
      this.placedItemsInCurrentLayer = new ArrayList<>();
      this.packedRectangles = null;

      animationTimer = new Timer(20, e -> updateAnimation());
      animationTimer.start();

      animState = AnimState.IDLE;
      setupNextItem();
    }

    public boolean isAnimationRunning() { return animState != AnimState.IDLE && animState != AnimState.FINISHED; }

    private void setupNextItem() {
      if (currentItemIndex >= allItems.size()) {
        animState = AnimState.FINISHED; return;
      }
      currentPackedItem = allItems.get(currentItemIndex);
      if(currentPackedItem.layer > currentLayer) {
        currentLayer = currentPackedItem.layer;
        placedItemsInCurrentLayer.clear();
      }
      movingItem = new Rectangle(currentPackedItem.rect);
      startTransition(AnimState.APPROACHING_PICKUP, 40);
    }

    private void startTransition(AnimState nextState, int frames) {
      this.animState = nextState; this.animFrame = 0; this.animTotalFrames = frames;

      conveyorEnd = new Point2D.Double(-movingItem.width / 2.0, palletW * 0.8);

      switch(nextState) {
        case APPROACHING_PICKUP:
          itemAnimFrom = new Point2D.Double(conveyorStart.x, conveyorStart.y);
          itemAnimTo = conveyorEnd;
          armAnimFrom = (Point2D.Double) armGripper.clone(); armAnimTo = conveyorEnd;
          break;
        case PICKING: break;
        case MOVING_TO_PALLET:
          armAnimFrom = (Point2D.Double) armGripper.clone();
          armAnimTo = new Point2D.Double(currentPackedItem.rect.getCenterX(), currentPackedItem.rect.getCenterY());
          break;
        case PLACING: break;
      }
    }

    private void updateAnimation() {
      if (!isAnimationRunning()) { stopAnimation(); return; }

      double t = Math.min(1.0, (double) animFrame / animTotalFrames);
      switch (animState) {
        case APPROACHING_PICKUP:
          double itemX = itemAnimFrom.x + t * (itemAnimTo.x - itemAnimFrom.x);
          double itemY = itemAnimFrom.y + t * (itemAnimTo.y - itemAnimFrom.y);
          movingItem.setLocation((int)(itemX - movingItem.width/2.0), (int)(itemY - movingItem.height/2.0));

          armGripper.x = armAnimFrom.x + t * (armAnimTo.x - armAnimFrom.x);
          armGripper.y = armAnimFrom.y + t * (armAnimTo.y - armAnimFrom.y);
          break;
        case MOVING_TO_PALLET:
          armGripper.x = armAnimFrom.x + t * (armAnimTo.x - armAnimFrom.x);
          armGripper.y = armAnimFrom.y + t * (armAnimTo.y - armAnimFrom.y);
          movingItem.setLocation((int)(armGripper.x - movingItem.width/2.0), (int)(armGripper.y - movingItem.height/2.0));
          break;
        case PICKING: case PLACING: break;
      }

      animFrame++;
      if (animFrame > animTotalFrames) {
        switch (animState) {
          case APPROACHING_PICKUP: startTransition(AnimState.PICKING, 10); break;
          case PICKING:            startTransition(AnimState.MOVING_TO_PALLET, 40); break;
          case MOVING_TO_PALLET:   startTransition(AnimState.PLACING, 10); break;
          case PLACING:
            placedItemsInCurrentLayer.add(currentPackedItem.rect);
            currentItemIndex++;
            setupNextItem();
            break;
        }
      }
      repaint();
    }

    @Override public Dimension getPreferredSize() { return new Dimension(600, 600); }
    private double getScale(int width, int height) {
      double panelWidth = width - getInsets().left - getInsets().right;
      double panelHeight = height - getInsets().top - getInsets().bottom;
      return Math.min(panelWidth / (palletL * 1.4), panelHeight / palletW);
    }

    @Override
    protected void paintComponent(Graphics g) {
      super.paintComponent(g);
      Graphics2D g2d = (Graphics2D) g.create();
      g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

      double scale = getScale(getWidth(), getHeight());
      int drawXOffset = getInsets().left + (int)(palletL * 0.4 * scale);
      int drawYOffset = getInsets().top;

      conveyorStart = new Point2D.Double(-palletL * 0.4, palletW * 0.8);
      armShoulder.setLocation(palletL * 1.1, -palletW * 0.1);

      g2d.setColor(Color.LIGHT_GRAY);
      g2d.fillRect(drawXOffset, drawYOffset, (int)(palletL * scale), (int)(palletW * scale));

      if (packedRectangles != null) {
        for (Rectangle r : packedRectangles) drawRectangle(g2d, r, scale, drawXOffset, drawYOffset, 1, false);
      }

      double conveyorWidthScale = (movingItem != null) ? -movingItem.width/2.0 : -palletL * 0.1;
      g2d.setColor(Color.DARK_GRAY);
      g2d.fillRect(drawXOffset + (int)(conveyorStart.x * scale), drawYOffset + (int)(conveyorStart.y * scale) - 5, (int)((conveyorWidthScale - conveyorStart.x) * scale * 1.2), 10);

      if (isAnimationRunning() && movingItem != null) {
        g2d.setColor(new Color(0,0,0,50));
        g2d.drawString("Layer: " + currentLayer, 10, 20);

        for (Rectangle r : placedItemsInCurrentLayer) {
          drawRectangle(g2d, r, scale, drawXOffset, drawYOffset, currentLayer, false);
        }

        if (animState.compareTo(AnimState.FINISHED) < 0) {
          // FIX: Check if the item is held during the placing state as well
          boolean isHeld = animState.compareTo(AnimState.PICKING) >= 0 && animState.compareTo(AnimState.PLACING) <= 0;
          drawRectangle(g2d, movingItem, scale, drawXOffset, drawYOffset, currentLayer, isHeld);
        }

        calculateArmJoints(armShoulder, armGripper, palletL, palletW, armElbow);
        drawArm(g2d, scale, drawXOffset, drawYOffset);
      }

      g2d.setColor(Color.BLACK); g2d.setStroke(new BasicStroke(3));
      g2d.drawRect(drawXOffset, drawYOffset, (int)(palletL * scale), (int)(palletW * scale));
      g2d.dispose();
    }

    private void calculateArmJoints(Point2D.Double shoulder, Point2D.Double gripper, double l1, double l2, Point2D.Double elbow) {
      double dist = shoulder.distance(gripper);
      l1 *= 0.7; l2 *= 0.7;
      if (dist > l1 + l2) {
        double angle = Math.atan2(gripper.y - shoulder.y, gripper.x - shoulder.x);
        elbow.x = shoulder.x + l1 * Math.cos(angle); elbow.y = shoulder.y + l1 * Math.sin(angle);
      } else {
        double a = (l1*l1 - l2*l2 + dist*dist) / (2 * dist);
        double h = Math.sqrt(Math.max(0, l1*l1 - a*a));
        Point2D.Double p2 = new Point2D.Double(shoulder.x + a * (gripper.x - shoulder.x) / dist, shoulder.y + a * (gripper.y - shoulder.y) / dist);
        elbow.x = p2.x - h * (gripper.y - shoulder.y) / dist; elbow.y = p2.y + h * (gripper.x - shoulder.x) / dist;
      }
    }

    private void drawArm(Graphics2D g2d, double scale, int offX, int offY) {
      g2d.setColor(Color.DARK_GRAY); g2d.setStroke(new BasicStroke(8, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      int x1 = offX + (int)(armShoulder.x * scale), y1 = offY + (int)(armShoulder.y * scale);
      int x2 = offX + (int)(armElbow.x * scale), y2 = offY + (int)(armElbow.y * scale);
      int x3 = offX + (int)(armGripper.x * scale), y3 = offY + (int)(armGripper.y * scale);
      g2d.drawLine(x1, y1, x2, y2); g2d.drawLine(x2, y2, x3, y3);
      g2d.setColor(Color.GRAY);
      g2d.fillOval(x1 - 12, y1 - 12, 24, 24); g2d.fillOval(x2 - 10, y2 - 10, 20, 20);
      g2d.setColor(animState == AnimState.PICKING || animState == AnimState.MOVING_TO_PALLET ? Color.RED : Color.BLACK);
      g2d.fillOval(x3 - 8, y3 - 8, 16, 16);
    }

    private void drawRectangle(Graphics2D g2d, Rectangle r, double scale, int offX, int offY, int layer, boolean isHeld) {
      int x = offX + (int) (r.x * scale), y = offY + (int) (r.y * scale);
      int width = (int) (r.width * scale), height = (int) (r.height * scale);

      Color baseColor = r.height > r.width ? new Color(200, 220, 255) : new Color(255, 220, 200);
      float[] hsb = Color.RGBtoHSB(baseColor.getRed(), baseColor.getGreen(), baseColor.getBlue(), null);
      g2d.setColor(Color.getHSBColor(hsb[0], hsb[1], Math.max(0, 0.9f - ((layer-1) * 0.1f))));
      g2d.fillRect(x, y, width, height);

      g2d.setColor(isHeld ? Color.RED : Color.BLACK); g2d.setStroke(isHeld ? new BasicStroke(2) : new BasicStroke(1));
      g2d.drawRect(x, y, width, height);

      g2d.setColor(Color.DARK_GRAY);
      int centerX = x + width / 2, centerY = y + height / 2;
      g2d.drawLine(centerX - 2, centerY - 2, centerX + 2, centerY + 2); g2d.drawLine(centerX - 2, centerY + 2, centerX + 2, centerY - 2);
    }
  }

  public static void main(String[] args) {
    SwingUtilities.invokeLater(() -> new PalletVisualizerGUIAnimated().setVisible(true));
  }
}
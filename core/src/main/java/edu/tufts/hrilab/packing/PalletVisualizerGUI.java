package edu.tufts.hrilab.packing;

import javax.swing.*;
import java.awt.*;
import java.util.List;
import java.util.stream.Collectors;

/**
 * visualization of the results from the PalletPacker algorithm
 */
public class PalletVisualizerGUI extends JFrame {

  private final JTextField palletLField, palletWField, palletHField, rectLField, rectWField, rectHField;
  private final JLabel resultLabel;
  private final JTextArea bottomLayerInfoArea;
  private final DrawingPanel drawingPanel;

  public PalletVisualizerGUI() {
    setTitle("Pallet Packer Visualizer");
    setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    setLayout(new BorderLayout(10, 10));

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

    JButton calculateButton = new JButton("Calculate & Visualize");
    JPanel buttonPanel = new JPanel(new FlowLayout(FlowLayout.CENTER));
    buttonPanel.add(calculateButton);

    JPanel resultPanel = new JPanel(new BorderLayout(5, 5));
    resultLabel = new JLabel("Enter dimensions and click calculate.");
    resultLabel.setHorizontalAlignment(SwingConstants.CENTER);

    bottomLayerInfoArea = new JTextArea(5, 40);
    bottomLayerInfoArea.setEditable(false);
    bottomLayerInfoArea.setFont(new Font("Monospaced", Font.PLAIN, 12));
    bottomLayerInfoArea.setBorder(BorderFactory.createTitledBorder("2d item layout (repeated in every layer)"));

    resultPanel.add(resultLabel, BorderLayout.NORTH);
    resultPanel.add(new JScrollPane(bottomLayerInfoArea), BorderLayout.CENTER);

    drawingPanel = new DrawingPanel();

    JPanel topContentPanel = new JPanel(new BorderLayout());
    topContentPanel.add(inputPanel, BorderLayout.NORTH);
    topContentPanel.add(buttonPanel, BorderLayout.CENTER);

    JPanel topPanel = new JPanel(new BorderLayout());
    topPanel.add(topContentPanel, BorderLayout.NORTH);
    topPanel.add(resultPanel, BorderLayout.CENTER);
    add(topPanel, BorderLayout.NORTH);
    add(new JScrollPane(drawingPanel), BorderLayout.CENTER);

    calculateButton.addActionListener(e -> runCalculation());
    setSize(800, 900);
    setLocationRelativeTo(null);
  }

  private void runCalculation() {
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

      new SwingWorker<PackingResult3D, Void>() {
        @Override
        protected PackingResult3D doInBackground() {
          PalletPackingComponent packer = new PalletPackingComponent();
          return packer.solvePalletPacking3D(palletL, palletW, palletH, rectL, rectW, rectH);
        }

        @Override
        protected void done() {
          try {
            PackingResult3D result = get();
            String resultText;
            if (result.totalCount > 0) {
              resultText = String.format(
                      "Max items: %d (%d items/layer x %d layers). Item dimension is %d x %d.",
                      result.totalCount, result.layerPacking.count, result.numLayers, rectL, rectW
              );
              printBottomLayerInfo(result.allItems);
            } else {
              resultText = "No items fit on the pallet.";
              bottomLayerInfoArea.setText("N/A");
            }
            resultLabel.setText(resultText);
            drawingPanel.setPackingResult(palletL, palletW, result.layerPacking.rectangles);
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

  private void printBottomLayerInfo(List<PackedItem> allItems) {
    List<PackedItem> bottomLayerItems = allItems.stream()
            .filter(item -> item.layer == 1)
            .collect(Collectors.toList());

    StringBuilder sb = new StringBuilder();
    System.out.println("--- 2d item layout ---");
    for (PackedItem item : bottomLayerItems) {
      String itemInfo = item.toString();
      System.out.println(itemInfo);
      sb.append(itemInfo).append("\n");
    }
    System.out.println("--------------------------");
    bottomLayerInfoArea.setText(sb.toString());
  }

  private static class DrawingPanel extends JPanel {
    private int palletL = 1, palletW = 1;
    private List<Rectangle> packedRectangles;

    DrawingPanel() {
      setBackground(Color.WHITE);
      setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
    }

    public void setPackingResult(int palletL, int palletW, List<Rectangle> rectangles) {
      this.palletL = palletL;
      this.palletW = palletW;
      this.packedRectangles = rectangles;
      revalidate();
      repaint();
    }

    @Override
    public Dimension getPreferredSize() {
      return new Dimension(600, 600);
    }

    private double getScale() {
      if (getParent() == null) return 1.0;
      double panelWidth = getParent().getWidth() - 20;
      double panelHeight = getParent().getHeight() - 20;
      if (panelWidth <= 0 || panelHeight <= 0) return 1.0;
      return Math.min(panelWidth / palletL, panelHeight / palletW);
    }

    @Override
    protected void paintComponent(Graphics g) {
      super.paintComponent(g);
      if (packedRectangles == null) return;

      Graphics2D g2d = (Graphics2D) g;
      g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

      double scale = getScale();
      int drawXOffset = getInsets().left;
      int drawYOffset = getInsets().top;

      int palletDrawWidth = (int)(palletL * scale);
      int palletDrawHeight = (int)(palletW * scale);
      g2d.setColor(Color.LIGHT_GRAY);
      g2d.fillRect(drawXOffset, drawYOffset, palletDrawWidth, palletDrawHeight);

      for (Rectangle r : packedRectangles) {
        int x = drawXOffset + (int) (r.x * scale);
        int y = drawYOffset + (int) (r.y * scale);
        int width = (int) (r.width * scale);
        int height = (int) (r.height * scale);

        boolean isVertical = r.height > r.width;
        g2d.setColor(isVertical ? new Color(200, 220, 255) : new Color(255, 220, 200));
        g2d.fillRect(x, y, width, height);
        g2d.setColor(Color.BLACK);
        g2d.drawRect(x, y, width, height);

        // Draw an 'X' at the center of the rectangle
        int centerX = x + width / 2;
        int centerY = y + height / 2;
        g2d.setColor(Color.DARK_GRAY);
        g2d.drawLine(centerX - 3, centerY - 3, centerX + 3, centerY + 3);
        g2d.drawLine(centerX - 3, centerY + 3, centerX + 3, centerY - 3);
      }

      g2d.setColor(Color.BLACK);
      g2d.setStroke(new BasicStroke(3));
      g2d.drawRect(drawXOffset, drawYOffset, palletDrawWidth, palletDrawHeight);
    }
  }

  public static void main(String[] args) {
    SwingUtilities.invokeLater(() -> new PalletVisualizerGUI().setVisible(true));
  }
}
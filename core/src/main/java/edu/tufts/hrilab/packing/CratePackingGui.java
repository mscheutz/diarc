/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025
 */

package edu.tufts.hrilab.packing;

import javax.swing.*;
import java.awt.*;

public class CratePackingGui extends JFrame {
  private JButton updateButton;
  private JTextField crateLField;
  private JTextField crateWField;
  private JTextField crateHField;
  private JTextField itemWField;
  private JTextField itemHField;
  private JTextField itemLField;
  private JTextField crateLocationField;
  private JLabel  crateLLabel;
  private JLabel crateWLabel;
  private JLabel crateHLabel;
  private JLabel itemWLabel;
  private JLabel itemHLabel;
  private JLabel itemLLabel;
  private JLabel crateLocationLabel;
  private JLabel crateXLabel;
  private JLabel crateYLabel;
  private JLabel crateZLabel;
  private JTextField crateXField;
  private JTextField crateYField;
  private JTextField crateZField;


  public CratePackingGui(CratePackingComponent component) {
    setTitle("Crate Packing Configuration");
    setSize(1400, 900);
    setDefaultCloseOperation(EXIT_ON_CLOSE);
    setLayout(new BorderLayout());
    JPanel cratePanel = new  JPanel(new FlowLayout(FlowLayout.LEFT));
    JPanel itemPanel = new  JPanel(new FlowLayout(FlowLayout.LEFT));
    crateLLabel = new JLabel("Crate Length (L) in mm:");
    crateLField = new JTextField(5);
    crateLField.setText(String.valueOf(component.getCrateLength()));
    cratePanel.add(crateLLabel);
    cratePanel.add(crateLField);
    crateWLabel = new JLabel("Crate Width (W) in mm:");
    crateWField = new JTextField(5);
    crateWField.setText(String.valueOf(component.getCrateWidth()));
    cratePanel.add(crateWLabel);
    cratePanel.add(crateWField);
    crateHLabel = new JLabel("Crate Height (H) in mm:");
    crateHField = new JTextField(5);
    crateHField.setText(String.valueOf(component.getCrateHeight()));
    cratePanel.add(crateHLabel);
    cratePanel.add(crateHField);
    itemLLabel = new JLabel("Item Length (L) in mm:");
    itemLField = new JTextField(5);
    itemLField.setText(String.valueOf(component.getPackedItemLength()));
    itemPanel.add(itemLLabel);
    itemPanel.add(itemLField);
    itemWLabel = new JLabel("Item Width (W) in mm:");
    itemWField = new JTextField(5);
    itemWField.setText(String.valueOf(component.getPackedItemWidth()));
    itemPanel.add(itemWLabel);
    itemPanel.add(itemWField);
    itemHLabel = new JLabel("Item Height (H) in mm:");
    itemHField = new JTextField(5);
    itemHField.setText(String.valueOf(component.getPackedItemHeight()));
    itemPanel.add(itemHLabel);
    itemPanel.add(itemHField);
    add(cratePanel, BorderLayout.NORTH);
    add(itemPanel, BorderLayout.CENTER);
    JButton updateButton = new JButton("Update");
    itemPanel.add(updateButton, BorderLayout.SOUTH);
    updateButton.addActionListener(e -> component.updateArgs(Integer.parseInt(crateLField.getText()),
            Integer.parseInt(crateWField.getText()),
            Integer.parseInt(crateHField.getText()),
            Integer.parseInt(itemLField.getText()),
            Integer.parseInt(itemWField.getText()),
            Integer.parseInt(itemHField.getText())));


  }

  public static void main(String[] args) {
    SwingUtilities.invokeLater(() -> new CratePackingGui(new CratePackingComponent()).setVisible(true));
  }
}

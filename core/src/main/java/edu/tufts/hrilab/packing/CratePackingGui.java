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
  private JLabel crateLCurrentValue;
  private JLabel crateWCurrentValue;
  private JLabel crateHCurrentValue;
  private JLabel itemLCurrentValue;
  private JLabel itemWCurrentValue;
  private JLabel itemHCurrentValue;
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
  private JLabel warningMessage;


  public CratePackingGui(CratePackingComponent component) {
    setTitle("Crate Packing Configuration");
    setSize(900, 150);
    setDefaultCloseOperation(EXIT_ON_CLOSE);
    setLayout(new BorderLayout());
    JPanel cratePanel = new  JPanel(new FlowLayout(FlowLayout.LEFT));
    JPanel itemPanel = new  JPanel(new FlowLayout(FlowLayout.LEFT));
    crateLLabel = new JLabel("Crate Length (L) in mm:");
    JPanel crateLFieldPanel = new JPanel();
    crateLFieldPanel.setLayout(new BoxLayout(crateLFieldPanel, BoxLayout.Y_AXIS));
    crateLField = new JTextField(5);
    crateLField.setText(String.valueOf(component.getCrateLength()));
    crateLCurrentValue = new JLabel("Current Value: "+component.getCrateLength());
    crateLFieldPanel.add(crateLField);
    crateLFieldPanel.add(crateLCurrentValue);
    cratePanel.add(crateLLabel);
    cratePanel.add(crateLFieldPanel);
    crateWLabel = new JLabel("Crate Width (W) in mm:");
    crateWField = new JTextField(5);
    crateWField.setText(String.valueOf(component.getCrateWidth()));
    JPanel crateWFieldPanel = new JPanel();
    crateWFieldPanel.setLayout(new BoxLayout(crateWFieldPanel, BoxLayout.Y_AXIS));
    crateWCurrentValue = new JLabel("Current Value: "+component.getCrateWidth());
    crateWFieldPanel.add(crateWField);
    crateWFieldPanel.add(crateWCurrentValue);
    cratePanel.add(crateWLabel);
    cratePanel.add(crateWFieldPanel);
    crateHLabel = new JLabel("Crate Height (H) in mm:");
    crateHField = new JTextField(5);
    crateHField.setText(String.valueOf(component.getCrateHeight()));
    JPanel crateHFieldPanel = new JPanel();
    crateHFieldPanel.setLayout(new BoxLayout(crateHFieldPanel, BoxLayout.Y_AXIS));
    crateHCurrentValue = new JLabel("Current Value: "+component.getCrateHeight());
    crateHFieldPanel.add(crateHField);
    crateHFieldPanel.add(crateHCurrentValue);
    cratePanel.add(crateHLabel);
    cratePanel.add(crateHFieldPanel);
    itemLLabel = new JLabel("Item Length (L) in mm:");
    itemLField = new JTextField(5);
    itemLField.setText(String.valueOf(component.getPackedItemLength()));
    JPanel itemLFieldPanel = new JPanel();
    itemLFieldPanel.setLayout(new BoxLayout(itemLFieldPanel, BoxLayout.Y_AXIS));
    itemLCurrentValue = new JLabel("Current Value: "+component.getPackedItemLength());
    itemLFieldPanel.add(itemLField);
    itemLFieldPanel.add(itemLCurrentValue);
    itemPanel.add(itemLLabel);
    itemPanel.add(itemLFieldPanel);
    itemWLabel = new JLabel("Item Width (W) in mm:");
    itemWField = new JTextField(5);
    itemWField.setText(String.valueOf(component.getPackedItemWidth()));
    JPanel itemWFieldPanel = new JPanel();
    itemWFieldPanel.setLayout(new BoxLayout(itemWFieldPanel, BoxLayout.Y_AXIS));
    itemWCurrentValue = new JLabel("Current Value: "+component.getPackedItemWidth());
    itemWFieldPanel.add(itemWField);
    itemWFieldPanel.add(itemWCurrentValue);
    itemPanel.add(itemWLabel);
    itemPanel.add(itemWFieldPanel);
    itemHLabel = new JLabel("Item Height (H) in mm:");
    itemHField = new JTextField(5);
    itemHField.setText(String.valueOf(component.getPackedItemHeight()));
    JPanel itemHFieldPanel = new JPanel();
    itemHFieldPanel.setLayout(new BoxLayout(itemHFieldPanel, BoxLayout.Y_AXIS));
    itemHCurrentValue = new JLabel("Current Value: "+component.getPackedItemHeight());
    itemHFieldPanel.add(itemHField);
    itemHFieldPanel.add(itemHCurrentValue);
    itemPanel.add(itemHLabel);
    itemPanel.add(itemHFieldPanel);
    JPanel updatePanel = new JPanel();
    JButton updateButton = new JButton("Update");
    updatePanel.add(updateButton);
    warningMessage = new JLabel("Error: Please only input positive integers");
    updatePanel.add(warningMessage);
    warningMessage.setVisible(false);
    add(cratePanel, BorderLayout.NORTH);
    add(itemPanel, BorderLayout.CENTER);
    add(updatePanel, BorderLayout.SOUTH);
    updateButton.addActionListener(e -> {
      warningMessage.setVisible(false);
      try {
        int crateL = Integer.parseInt(crateLField.getText());
        int crateW = Integer.parseInt(crateWField.getText());
        int crateH = Integer.parseInt(crateHField.getText());
        int itemL = Integer.parseInt(itemLField.getText());
        int itemW = Integer.parseInt(itemWField.getText());
        int itemH = Integer.parseInt(itemHField.getText());
        if (crateL <= 0 || crateW <= 0 || crateH <= 0 || itemL <= 0 || itemW <= 0 || itemH <= 0) {
          warningMessage.setVisible(true);
          return;
        }
        component.updateArgs(crateL,crateW,crateH,itemL,itemW,itemH);
      } catch (NumberFormatException ignored) {
        warningMessage.setVisible(true);
      }
    });
  }

  public static void main(String[] args) {
    SwingUtilities.invokeLater(() -> new CratePackingGui(new CratePackingComponent()).setVisible(true));
  }

  public void setCrateLFieldValue(int value) {
    this.crateLField.setText(String.valueOf(value));
    this.crateLCurrentValue.setText("Current Value: " + String.valueOf(value));
  }

  public void setCrateWFieldValue(int value) {
    this.crateWField.setText(String.valueOf(value));
    this.crateWCurrentValue.setText("Current Value: " + String.valueOf(value));
  }

  public void setCrateHFieldValue(int value) {
    this.crateHField.setText(String.valueOf(value));
    this.crateHCurrentValue.setText("Current Value: " + String.valueOf(value));
  }

  public void setItemWFieldValue(int value) {
    this.itemWField.setText(String.valueOf(value));
    this.itemWCurrentValue.setText("Current Value: " + String.valueOf(value));
  }

  public void setItemHFieldValue(int value) {
    this.itemHField.setText(String.valueOf(value));
    this.itemHCurrentValue.setText("Current Value: " + String.valueOf(value));
  }

  public void setItemLFieldValue(int value) {
    this.itemLField.setText(String.valueOf(value));
    this.itemLCurrentValue.setText("Current Value: " + String.valueOf(value));
  }
}

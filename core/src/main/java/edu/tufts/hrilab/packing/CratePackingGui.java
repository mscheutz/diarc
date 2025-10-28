/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025
 */

package edu.tufts.hrilab.packing;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.llm.serviceproviders.CratePackingLLMServiceProvider;
import edu.tufts.hrilab.slug.nlg.hybrid.HybridNLGComponent;
import edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;

public class CratePackingGui extends JFrame {
  private JButton updateButton;
  private JTextField crateLField;
  private JTextField crateWField;
  private JTextField crateHField;
  private JTextField itemWField;
  private JTextField itemHField;
  private JTextField itemLField;
  private JTextField toleranceField;
  private JLabel crateLCurrentValue;
  private JLabel crateWCurrentValue;
  private JLabel crateHCurrentValue;
  private JLabel itemLCurrentValue;
  private JLabel itemWCurrentValue;
  private JLabel itemHCurrentValue;
  private JLabel toleranceCurrentValue;
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
  //TODO: last second gui cleanup during packaging rush, don't do this
  private HybridNLGComponent hybridNLGComponent;
  private HybridParserComponent hybridParserComponent;
  private CratePackingComponent cratePackingComponent;
  private CratePackingLLMServiceProvider serviceProvider;
  private boolean japanese;

  //TODO: add tolerance, number of items to pack?
  //      update display correctly
  //      Write to file on close
  public CratePackingGui(CratePackingComponent cratePackingComponent, HybridParserComponent hybridParserComponent, HybridNLGComponent hybridNLGComponent, CratePackingLLMServiceProvider serviceProvider, boolean japanese) {
    this.cratePackingComponent = cratePackingComponent;
    this.hybridParserComponent = hybridParserComponent;
    this.hybridNLGComponent = hybridNLGComponent;
    this.serviceProvider = serviceProvider;
    this.japanese = japanese;

    setTitle("TRACS Crate Packing Parameter Console");
    setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    setLayout(new GridLayout(0,1));
    //setSize(1800,300);


    //JPanel languagePanel = new JPanel();
    //languagePanel.setSize(1000,30);
    //languagePanel.setLayout(new BoxLayout(languagePanel, BoxLayout.Y_AXIS));
    JPanel innerPanel = new JPanel();
    //innerPanel.setSize(1800,50);
    innerPanel.setLayout(new FlowLayout(FlowLayout.LEFT));
    JCheckBox languageToggle = new JCheckBox();
    JLabel languageToggleLabel = new JLabel("Set Language to Japanese: ");
    languageToggle.addActionListener(e -> toggleLanguage());
    innerPanel.add(languageToggleLabel);
    innerPanel.add(languageToggle);
    //languagePanel.add(innerPanel);
    //getContentPane().add(languagePanel);
    //add(languagePanel);
    add(innerPanel);

    //JPanel dimensionsPanel = new JPanel();
    //dimensionsPanel.setSize(1000, 150);
    //dimensionsPanel.setLayout(new BorderLayout());
    JPanel cratePanel = new  JPanel(new FlowLayout(FlowLayout.LEFT));
    cratePanel.setBorder(BorderFactory.createEmptyBorder(0, 0, 0, 100));
    JPanel itemPanel = new  JPanel(new FlowLayout(FlowLayout.LEFT));
    itemPanel.setBorder(BorderFactory.createEmptyBorder(0, 0, 0, 100));
    crateLLabel = new JLabel("Crate Length (L) in mm:");
    JPanel crateLFieldPanel = new JPanel();
    crateLFieldPanel.setLayout(new BoxLayout(crateLFieldPanel, BoxLayout.Y_AXIS));
    crateLField = new JTextField(5);
    crateLField.setText(String.valueOf(cratePackingComponent.getCrateLength()));
    crateLCurrentValue = new JLabel("Current Value: "+cratePackingComponent.getCrateLength());
    crateLFieldPanel.add(crateLField);
    crateLFieldPanel.add(crateLCurrentValue);
    cratePanel.add(crateLLabel);
    cratePanel.add(crateLFieldPanel);
    crateWLabel = new JLabel("Crate Width (W) in mm:");
    crateWField = new JTextField(5);
    crateWField.setText(String.valueOf(cratePackingComponent.getCrateWidth()));
    JPanel crateWFieldPanel = new JPanel();
    crateWFieldPanel.setLayout(new BoxLayout(crateWFieldPanel, BoxLayout.Y_AXIS));
    crateWCurrentValue = new JLabel("Current Value: "+cratePackingComponent.getCrateWidth());
    crateWFieldPanel.add(crateWField);
    crateWFieldPanel.add(crateWCurrentValue);
    cratePanel.add(crateWLabel);
    cratePanel.add(crateWFieldPanel);
    crateHLabel = new JLabel("Crate Height (H) in mm:");
    crateHField = new JTextField(5);
    crateHField.setText(String.valueOf(cratePackingComponent.getCrateHeight()));
    JPanel crateHFieldPanel = new JPanel();
    crateHFieldPanel.setLayout(new BoxLayout(crateHFieldPanel, BoxLayout.Y_AXIS));
    crateHCurrentValue = new JLabel("Current Value: "+cratePackingComponent.getCrateHeight());
    crateHFieldPanel.add(crateHField);
    crateHFieldPanel.add(crateHCurrentValue);
    cratePanel.add(crateHLabel);
    cratePanel.add(crateHFieldPanel);
    itemLLabel = new JLabel("Item Length (L) in mm:");
    itemLField = new JTextField(5);
    itemLField.setText(String.valueOf(cratePackingComponent.getPackedItemLength()));
    JPanel itemLFieldPanel = new JPanel();
    itemLFieldPanel.setLayout(new BoxLayout(itemLFieldPanel, BoxLayout.Y_AXIS));
    itemLCurrentValue = new JLabel("Current Value: "+cratePackingComponent.getPackedItemLength());
    itemLFieldPanel.add(itemLField);
    itemLFieldPanel.add(itemLCurrentValue);
    itemPanel.add(itemLLabel);
    itemPanel.add(itemLFieldPanel);
    itemWLabel = new JLabel("Item Width (W) in mm:");
    itemWField = new JTextField(5);
    itemWField.setText(String.valueOf(cratePackingComponent.getPackedItemWidth()));
    JPanel itemWFieldPanel = new JPanel();
    itemWFieldPanel.setLayout(new BoxLayout(itemWFieldPanel, BoxLayout.Y_AXIS));
    itemWCurrentValue = new JLabel("Current Value: "+cratePackingComponent.getPackedItemWidth());
    itemWFieldPanel.add(itemWField);
    itemWFieldPanel.add(itemWCurrentValue);
    itemPanel.add(itemWLabel);
    itemPanel.add(itemWFieldPanel);
    itemHLabel = new JLabel("Item Height (H) in mm:");
    itemHField = new JTextField(5);
    itemHField.setText(String.valueOf(cratePackingComponent.getPackedItemHeight()));
    JPanel itemHFieldPanel = new JPanel();
    itemHFieldPanel.setLayout(new BoxLayout(itemHFieldPanel, BoxLayout.Y_AXIS));
    itemHCurrentValue = new JLabel("Current Value: "+cratePackingComponent.getPackedItemHeight());
    itemHFieldPanel.add(itemHField);
    itemHFieldPanel.add(itemHCurrentValue);
    itemPanel.add(itemHLabel);
    itemPanel.add(itemHFieldPanel);

    JPanel updatePanel = new JPanel();
    itemPanel.setBorder(BorderFactory.createEmptyBorder(0, 0, 10, 0));
    JButton updateButton = new JButton("Update");
    updatePanel.add(updateButton);
    JButton resetButton = new JButton("Reset");
    updatePanel.add(resetButton);
    warningMessage = new JLabel("Error: Please only input positive integers");
    updatePanel.add(warningMessage);
    warningMessage.setVisible(false);
    add(cratePanel);
    add(itemPanel);
    add(updatePanel);
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
          warningMessage = new JLabel("Error: Please only input positive integers");
          warningMessage.setVisible(true);
          return;
        }

        try {
          TRADEServiceInfo submitGoalService = TRADE.getAvailableService(
                  new TRADEServiceConstraints()
                          .name("submitGoal")
                          .argTypes(Predicate.class)
                          .returnType(long.class)
          );
          submitGoalService.call(
                  long.class,
                  Factory.createPredicate(String.format("updateAllDimensions(robotone,%d,%d,%d,%d,%d,%d)",crateL,crateW,crateH,itemL,itemW,itemH))
          );
        } catch (TRADEException ignored) {
          warningMessage = new JLabel("Network Error: Could not update fields");
          warningMessage.setVisible(true);
        }
        //cratePackingComponent.updateArgs(crateL,crateW,crateH,itemL,itemW,itemH);
      } catch (NumberFormatException ignored) {
        warningMessage = new JLabel("Error: Please only input positive integers");
        warningMessage.setVisible(true);
      }
    });
    resetButton.addActionListener(e -> {
      warningMessage.setVisible(false);
      try {
        TRADEServiceInfo submitGoalService = TRADE.getAvailableService(
                new TRADEServiceConstraints()
                        .name("submitGoal")
                        .argTypes(Predicate.class)
                        .returnType(long.class)
        );
        submitGoalService.call(
                long.class,
                Factory.createPredicate("clearPackingParams(robotone)")
        );
      } catch (TRADEException ignored) {
        warningMessage = new JLabel("Network Error: Could not reset params");
        warningMessage.setVisible(true);
      }
    });
    //add(dimensionsPanel);

    //TODO: need to make it so we need to rerun solver if changing this
    JPanel tolerancePanel = new JPanel();
    tolerancePanel.setLayout(new FlowLayout(FlowLayout.LEFT));
    JLabel toleranceLabel = new JLabel("Item Spacing Tolerance in mm:");
    toleranceField = new JTextField(5);
    toleranceField.setText(String.valueOf(cratePackingComponent.getTolerance()));
    toleranceCurrentValue = new JLabel("Current Value: "+cratePackingComponent.getTolerance());
    JPanel toleranceFieldPanel = new JPanel();
    toleranceFieldPanel.setLayout(new BoxLayout(toleranceFieldPanel, BoxLayout.Y_AXIS));
    tolerancePanel.add(toleranceLabel);
    toleranceFieldPanel.add(toleranceField);
    toleranceFieldPanel.add(toleranceCurrentValue);
    tolerancePanel.add(toleranceFieldPanel);
    JButton toleranceUpdateButton = new JButton("Update Tolerance");
    tolerancePanel.add(toleranceUpdateButton);
    toleranceUpdateButton.addActionListener(e -> {
      try {
        int tolerance = Integer.parseInt(toleranceField.getText());
        try {
          TRADEServiceInfo submitGoalService = TRADE.getAvailableService(
                  new TRADEServiceConstraints()
                          .name("submitGoal")
                          .argTypes(Predicate.class)
                          .returnType(long.class)
          );
          submitGoalService.call(
                  long.class,
                  Factory.createPredicate(String.format("updateItemTolerance(robotone,%d)",tolerance))
          );
        } catch (TRADEException ignored) {
        }
        //cratePackingComponent.setTolerance(tolerance);
      } catch (NumberFormatException ignored) {
      }
    });
    add(tolerancePanel);

    pack();
    setVisible(true);
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

  public void setToleranceFieldValue(int value) {
    this.toleranceField.setText(String.valueOf(value));
    this.toleranceCurrentValue.setText("Current Value: " + String.valueOf(value));
  }

  //Should be one of "English" or "Japanese" (case matters)
  public void setLanguage(String language) {
    hybridNLGComponent.getCacheNLG().setLanguage(language);
    hybridParserComponent.getCacheParser().setLanguage(language);
    if (serviceProvider != null) {
      serviceProvider.setLanguage(language);
    }
    java.util.List<String> currentPoseNames = new ArrayList<>();
    List<String> newPoseNames = new ArrayList<>();
    TRADEServiceInfo tsi;
    try {
      if (language.equals("Japanese")) {
        tsi = getServiceForAgent("robotone", "getEnglishPoseDescriptors");
        if (tsi != null) {
          currentPoseNames = tsi.call(List.class);
        }
        //currentPoseNames = assista.getEnglishPoseDescriptors();

        tsi = getServiceForAgent("robotone", "getJapanesePoseDescriptors");
        if (tsi != null) {
          newPoseNames = tsi.call(List.class);
        }
        //newPoseNames = assista.getJapanesePoseDescriptors();

      } else {
        tsi = getServiceForAgent("robotone", "getJapanesePoseDescriptors");
        if (tsi != null) {
          currentPoseNames = tsi.call(List.class);
        }
        //currentPoseNames = assista.getJapanesePoseDescriptors();

        tsi = getServiceForAgent("robotone", "getEnglishPoseDescriptors");
        if (tsi != null) {
          newPoseNames = tsi.call(List.class);
        }
        //newPoseNames = assista.getEnglishPoseDescriptors();
      }
      tsi = getServiceForAgent("robotone", "updatePoseName", Symbol.class, Symbol.class);
      for (int i = 0; i < currentPoseNames.size(); i++) {
        tsi.call(boolean.class, Factory.createSymbol(currentPoseNames.get(i), "pose"), Factory.createSymbol(newPoseNames.get(i), "pose"));
        //assista.updatePoseName(Factory.createSymbol(currentPoseNames.get(i), "pose"), Factory.createSymbol(newPoseNames.get(i), "pose"));
      }
    } catch (TRADEException ignored) {
      //TODO: proper error logging
    }
  }

  TRADEServiceInfo getServiceForAgent(String agentName, String serviceName, Class<?>... argTypes) {
    HashSet<TRADEServiceInfo> services = (HashSet<TRADEServiceInfo>) TRADE.getAvailableServices(new TRADEServiceConstraints().name(serviceName).argTypes(argTypes));
    for (TRADEServiceInfo tsi : services) {
      Collection<String> groups = tsi.getGroups();
      for (String group : groups) {
        if (group.contains(agentName)) {
          return tsi;
        }
      }
    }
    return null;
  }

  private void toggleLanguage() {
    japanese = !japanese;
    setLanguage(japanese ? "Japanese" : "English");
  }
}
/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.gui;

import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.gui.tablemodels.ConditionsTableModel;
import edu.tufts.hrilab.action.gui.tablemodels.EffectsTableModel;
import edu.tufts.hrilab.action.gui.tablemodels.RolesTableModel;

import javax.swing.*;
import javax.swing.border.TitledBorder;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import javax.swing.plaf.FontUIResource;
import javax.swing.text.StyleContext;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeModel;
import java.awt.*;
import java.util.Locale;

/**
 * Script Editor. Provides fields to edit an action script.
 * The event spec editing (main text field on the bottom) is
 * handled by EventSpecEditor class.
 */
public class ScriptEditor {
  private JTextField typeInput;
  private JTextField descrInput;
  private JTextField benefitInput;
  private JTextField costInput;
  private JTextField minUrgInput;
  private JTextField maxUrgInput;
  private JTextField locksInput;
  private JTextField timeoutInput;
  private JPanel mainPanel;
  private JTextPane editorPane;
  private JButton saveButton;
  private JPanel conditionsPanel;
  private JPanel effectsPanel;
  private JPanel rolesPanel;

  private String id;

  private EventSpecEditor editor;
  private ScriptEditorTab tab;

  private OrderedTable conditions;
  private OrderedTable effects;
  private OrderedTable roles;

  /**
   * Goal manager gui reference.
   */
  private static DatabaseViewer gm;

  /**
   * JTabbedPane containing opened actions.
   */
  private static JTabbedPane tabs;

  /**
   * JTree containing actions.
   */
  private static JTree tree;

  /**
   * Node this ScriptEditor instance is in charge of.
   */
  private DefaultMutableTreeNode node;

  /**
   * Action this ScriptEditor instance is in charge of.
   */
  private ActionDBEntry action;


  public static void setGMGUI(DatabaseViewer gui) {
    gm = gui;
  }

  public static void setActionsTree(JTree actionsTree) {
    tree = actionsTree;
  }

  public static void setTabs(JTabbedPane actionsTabs) {
    tabs = actionsTabs;
  }

  public ScriptEditor(DefaultMutableTreeNode actionNode, boolean disabled) {
    node = actionNode;

    ADBEWrapper adbew = (ADBEWrapper) node.getUserObject();
    action = adbew.getEntry();

    id = adbew.getID(); // Get unique identifier for this action
    typeInput.setText(action.getType());
    descrInput.setText(action.getDescription());
    benefitInput.setText(Double.toString(action.getBenefit()));
    costInput.setText(Double.toString(action.getCost()));
    minUrgInput.setText(Double.toString(action.getMinUrg()));
    maxUrgInput.setText(Double.toString(action.getMaxUrg()));
    //locksInput.setText(action.getResourceLockNames()); TODO: Handle this.
    if (action.getTimeout() == Integer.MAX_VALUE) {
      timeoutInput.setText("inf");
    } else {
      timeoutInput.setText(Long.toString(action.getTimeout()));
    }

    conditions = new OrderedTable(conditionsPanel, new ConditionsTableModel(action));
    conditions.setColumnWidth(0, 65);
    conditions.table().setDefaultEditor(Condition.class, new ConditionEditor());

    effects = new OrderedTable(effectsPanel, new EffectsTableModel(action));
    effects.setColumnWidth(0, 65);
    effects.setColumnWidth(2, 65);
    effects.setColumnWidth(3, 30);
    effects.table().setDefaultRenderer(String.class, new EffectsTableModel.Renderer());

    roles = new OrderedTable(rolesPanel, new RolesTableModel(action));
    roles.setColumnWidth(3, 60);
    roles.table().setDefaultRenderer(String.class, new RolesTableModel.Renderer());

    setupListeners();

    // Move event specs into editor
    editor = new EventSpecEditor(editorPane, roles.table(), tree);
    editor.addEventSpecs(action.getEventSpecs());

    if (disabled) {
      disable(); // Disable all editing
    }

    // Create new tab
    tab = new ScriptEditorTab(gm, tabs, action.getName(), id);
    tabs.addTab(id, mainPanel);
    tabs.setTabComponentAt(tabs.indexOfTab(id), tab.getPanel());
    tabs.setSelectedIndex(tabs.indexOfTab(id));
  }

  public String getID() {
    return id;
  }

  /**
   * Trigger component update.
   */
  public void update() {
    editor.update();
  }

  private void setupListeners() {
    saveButton.addActionListener(e -> saveScript());

    typeInput.getDocument().addDocumentListener(new DocumentListener() {
      @Override
      public void insertUpdate(DocumentEvent e) {
        tab.setName(typeInput.getText());
        ((ADBEWrapper) node.getUserObject()).rename(typeInput.getText());
        ((DefaultTreeModel) tree.getModel()).nodeChanged(node);
      }

      @Override
      public void removeUpdate(DocumentEvent e) {
        tab.setName(typeInput.getText());
        ((ADBEWrapper) node.getUserObject()).rename(typeInput.getText());
        ((DefaultTreeModel) tree.getModel()).nodeChanged(node);
      }

      @Override
      public void changedUpdate(DocumentEvent e) {

      }
    });
  }

  private void saveScript() {
    boolean saved;
    ActionDBEntry save = build();
    Database.getInstance().addActionDBEntry(save);

    if (save != null) {
      if (action.getDBFile() == null) {
        saved = ScriptExport.saveScriptToNewFile(save);
      } else {
        saved = ScriptExport.saveScriptToExistingFile(action.getDBFile(), save);
        if (saved) {
          ADBEWrapper adbew = (ADBEWrapper) node.getUserObject();
          adbew.replaceEntry(save);
        }
      }

      if (saved) {
        gm.savedFile();
      }
    } else {
      JOptionPane.showMessageDialog(mainPanel, "Could not save, action is invalid.\n" +
                      "Please make sure the data you entered is correct.",
              "Error", JOptionPane.ERROR_MESSAGE);
    }
  }

  /**
   * Build an ActionDBEntry from the information in the editor.
   *
   * @return new action DB Entry, or null if fail
   */
  private ActionDBEntry build() {
    ActionDBEntry.Builder builder = new ActionDBEntry.Builder(typeInput.getText());
    builder.setDescription(descrInput.getText());
    builder.setBenefit(benefitInput.getText());
    builder.setCost(costInput.getText());
    builder.setMinUrg(minUrgInput.getText());
    builder.setMaxUrg(maxUrgInput.getText());
    // TODO: Handle locks

    if (timeoutInput.getText().length() > 0 && !timeoutInput.getText().equalsIgnoreCase("inf")) {
      builder.setTimeout(timeoutInput.getText());
    }

    if (!((ConditionsTableModel) conditions.table().getModel()).exportConditions(builder)) return null;
    if (!((EffectsTableModel) effects.table().getModel()).exportEffects(builder)) return null;
    if (!((RolesTableModel) roles.table().getModel()).exportRoles(builder)) return null;

    editor.getEventSpecs().forEach(builder::addEventSpec);

    builder.setDBFile(action.getDBFile());

    return builder.build(false);
  }

  private void disable() {
    typeInput.setEnabled(false);
    descrInput.setEnabled(false);
    benefitInput.setEnabled(false);
    costInput.setEnabled(false);
    minUrgInput.setEnabled(false);
    maxUrgInput.setEnabled(false);
    locksInput.setEnabled(false);
    timeoutInput.setEnabled(false);
    mainPanel.setEnabled(false);
    editorPane.setEditable(false);
    //saveButton.setEnabled(false);
    saveButton.setText("Export");
  }

  {
// GUI initializer generated by IntelliJ IDEA GUI Designer
// >>> IMPORTANT!! <<<
// DO NOT EDIT OR ADD ANY CODE HERE!
    $$$setupUI$$$();
  }

  /**
   * Method generated by IntelliJ IDEA GUI Designer
   * >>> IMPORTANT!! <<<
   * DO NOT edit this method OR call it in your code!
   *
   * @noinspection ALL
   */
  private void $$$setupUI$$$() {
    mainPanel = new JPanel();
    mainPanel.setLayout(new GridBagLayout());
    mainPanel.setMinimumSize(new Dimension(640, 480));
    mainPanel.setPreferredSize(new Dimension(640, 480));
    final JPanel panel1 = new JPanel();
    panel1.setLayout(new GridBagLayout());
    GridBagConstraints gbc;
    gbc = new GridBagConstraints();
    gbc.gridx = 0;
    gbc.gridy = 0;
    gbc.weightx = 1.0;
    gbc.fill = GridBagConstraints.BOTH;
    mainPanel.add(panel1, gbc);
    final JLabel label1 = new JLabel();
    label1.setText("Description:");
    gbc = new GridBagConstraints();
    gbc.gridx = 0;
    gbc.gridy = 1;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    panel1.add(label1, gbc);
    descrInput = new JTextField();
    gbc = new GridBagConstraints();
    gbc.gridx = 1;
    gbc.gridy = 1;
    gbc.weightx = 1.0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    gbc.fill = GridBagConstraints.HORIZONTAL;
    panel1.add(descrInput, gbc);
    final JPanel panel2 = new JPanel();
    panel2.setLayout(new GridBagLayout());
    gbc = new GridBagConstraints();
    gbc.gridx = 0;
    gbc.gridy = 0;
    gbc.gridwidth = 2;
    gbc.fill = GridBagConstraints.BOTH;
    panel1.add(panel2, gbc);
    final JLabel label2 = new JLabel();
    label2.setText("Type/Name:");
    gbc = new GridBagConstraints();
    gbc.gridx = 0;
    gbc.gridy = 0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    panel2.add(label2, gbc);
    typeInput = new JTextField();
    gbc = new GridBagConstraints();
    gbc.gridx = 1;
    gbc.gridy = 0;
    gbc.weightx = 1.0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    gbc.fill = GridBagConstraints.HORIZONTAL;
    panel2.add(typeInput, gbc);
    saveButton = new JButton();
    saveButton.setMargin(new Insets(2, 2, 2, 2));
    saveButton.setMaximumSize(new Dimension(70, 20));
    saveButton.setMinimumSize(new Dimension(70, 20));
    saveButton.setPreferredSize(new Dimension(70, 20));
    saveButton.setText("Save");
    gbc = new GridBagConstraints();
    gbc.gridx = 2;
    gbc.gridy = 0;
    gbc.fill = GridBagConstraints.HORIZONTAL;
    panel2.add(saveButton, gbc);
    final JPanel panel3 = new JPanel();
    panel3.setLayout(new GridBagLayout());
    gbc = new GridBagConstraints();
    gbc.gridx = 0;
    gbc.gridy = 1;
    gbc.weightx = 1.0;
    gbc.fill = GridBagConstraints.BOTH;
    mainPanel.add(panel3, gbc);
    final JPanel panel4 = new JPanel();
    panel4.setLayout(new GridBagLayout());
    gbc = new GridBagConstraints();
    gbc.gridx = 0;
    gbc.gridy = 0;
    gbc.weightx = 1.0;
    gbc.weighty = 1.0;
    gbc.fill = GridBagConstraints.BOTH;
    panel3.add(panel4, gbc);
    final JLabel label3 = new JLabel();
    label3.setText("Benefit:");
    gbc = new GridBagConstraints();
    gbc.gridx = 0;
    gbc.gridy = 0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    panel4.add(label3, gbc);
    benefitInput = new JTextField();
    gbc = new GridBagConstraints();
    gbc.gridx = 1;
    gbc.gridy = 0;
    gbc.weightx = 1.0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    gbc.fill = GridBagConstraints.HORIZONTAL;
    panel4.add(benefitInput, gbc);
    final JLabel label4 = new JLabel();
    label4.setText("Cost:");
    gbc = new GridBagConstraints();
    gbc.gridx = 2;
    gbc.gridy = 0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    panel4.add(label4, gbc);
    costInput = new JTextField();
    gbc = new GridBagConstraints();
    gbc.gridx = 3;
    gbc.gridy = 0;
    gbc.weightx = 1.0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    gbc.fill = GridBagConstraints.HORIZONTAL;
    panel4.add(costInput, gbc);
    final JLabel label5 = new JLabel();
    label5.setText("MinUrg:");
    gbc = new GridBagConstraints();
    gbc.gridx = 4;
    gbc.gridy = 0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    panel4.add(label5, gbc);
    minUrgInput = new JTextField();
    minUrgInput.setText("");
    gbc = new GridBagConstraints();
    gbc.gridx = 5;
    gbc.gridy = 0;
    gbc.weightx = 1.0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    gbc.fill = GridBagConstraints.HORIZONTAL;
    panel4.add(minUrgInput, gbc);
    final JLabel label6 = new JLabel();
    label6.setText("MaxUrg:");
    gbc = new GridBagConstraints();
    gbc.gridx = 6;
    gbc.gridy = 0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    panel4.add(label6, gbc);
    maxUrgInput = new JTextField();
    gbc = new GridBagConstraints();
    gbc.gridx = 7;
    gbc.gridy = 0;
    gbc.weightx = 1.0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    gbc.fill = GridBagConstraints.HORIZONTAL;
    panel4.add(maxUrgInput, gbc);
    final JPanel panel5 = new JPanel();
    panel5.setLayout(new GridBagLayout());
    gbc = new GridBagConstraints();
    gbc.gridx = 0;
    gbc.gridy = 1;
    gbc.weightx = 1.0;
    gbc.weighty = 1.0;
    gbc.fill = GridBagConstraints.BOTH;
    panel3.add(panel5, gbc);
    final JLabel label7 = new JLabel();
    label7.setText("Locks:");
    gbc = new GridBagConstraints();
    gbc.gridx = 0;
    gbc.gridy = 0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    panel5.add(label7, gbc);
    locksInput = new JTextField();
    gbc = new GridBagConstraints();
    gbc.gridx = 1;
    gbc.gridy = 0;
    gbc.weightx = 1.0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    gbc.fill = GridBagConstraints.HORIZONTAL;
    panel5.add(locksInput, gbc);
    final JLabel label8 = new JLabel();
    label8.setText("Timeout:");
    gbc = new GridBagConstraints();
    gbc.gridx = 2;
    gbc.gridy = 0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    panel5.add(label8, gbc);
    timeoutInput = new JTextField();
    timeoutInput.setMinimumSize(new Dimension(60, 26));
    timeoutInput.setPreferredSize(new Dimension(60, 26));
    gbc = new GridBagConstraints();
    gbc.gridx = 3;
    gbc.gridy = 0;
    gbc.weighty = 1.0;
    gbc.anchor = GridBagConstraints.WEST;
    gbc.fill = GridBagConstraints.HORIZONTAL;
    panel5.add(timeoutInput, gbc);
    final JSplitPane splitPane1 = new JSplitPane();
    splitPane1.setDividerLocation(102);
    splitPane1.setOrientation(0);
    gbc = new GridBagConstraints();
    gbc.gridx = 0;
    gbc.gridy = 2;
    gbc.weightx = 1.0;
    gbc.weighty = 1.0;
    gbc.fill = GridBagConstraints.BOTH;
    mainPanel.add(splitPane1, gbc);
    final JSplitPane splitPane2 = new JSplitPane();
    splitPane2.setAutoscrolls(true);
    splitPane2.setDividerLocation(85);
    splitPane2.setOrientation(0);
    splitPane1.setRightComponent(splitPane2);
    rolesPanel = new JPanel();
    rolesPanel.setLayout(new BorderLayout(0, 0));
    rolesPanel.setMinimumSize(new Dimension(-1, 85));
    rolesPanel.setPreferredSize(new Dimension(-1, 85));
    splitPane2.setLeftComponent(rolesPanel);
    rolesPanel.setBorder(BorderFactory.createTitledBorder(null, "Roles", TitledBorder.DEFAULT_JUSTIFICATION, TitledBorder.DEFAULT_POSITION, null, null));
    final JScrollPane scrollPane1 = new JScrollPane();
    scrollPane1.setHorizontalScrollBarPolicy(31);
    splitPane2.setRightComponent(scrollPane1);
    editorPane = new JTextPane();
    editorPane.setBackground(new Color(-14540254));
    editorPane.setCaretColor(new Color(-1));
    Font editorPaneFont = this.$$$getFont$$$("Monospaced", -1, 14, editorPane.getFont());
    if (editorPaneFont != null) editorPane.setFont(editorPaneFont);
    editorPane.setForeground(new Color(-5592406));
    editorPane.setMargin(new Insets(3, 0, 3, 3));
    scrollPane1.setViewportView(editorPane);
    final JPanel panel6 = new JPanel();
    panel6.setLayout(new GridBagLayout());
    splitPane1.setLeftComponent(panel6);
    final JSplitPane splitPane3 = new JSplitPane();
    splitPane3.setResizeWeight(0.5);
    gbc = new GridBagConstraints();
    gbc.gridx = 0;
    gbc.gridy = 0;
    gbc.gridwidth = 2;
    gbc.weightx = 1.0;
    gbc.weighty = 1.0;
    gbc.fill = GridBagConstraints.BOTH;
    panel6.add(splitPane3, gbc);
    conditionsPanel = new JPanel();
    conditionsPanel.setLayout(new BorderLayout(0, 0));
    splitPane3.setLeftComponent(conditionsPanel);
    conditionsPanel.setBorder(BorderFactory.createTitledBorder(null, "Conditions", TitledBorder.DEFAULT_JUSTIFICATION, TitledBorder.DEFAULT_POSITION, null, null));
    effectsPanel = new JPanel();
    effectsPanel.setLayout(new BorderLayout(0, 0));
    splitPane3.setRightComponent(effectsPanel);
    effectsPanel.setBorder(BorderFactory.createTitledBorder(null, "Effects", TitledBorder.DEFAULT_JUSTIFICATION, TitledBorder.DEFAULT_POSITION, null, null));
  }

  /**
   * @noinspection ALL
   */
  private Font $$$getFont$$$(String fontName, int style, int size, Font currentFont) {
    if (currentFont == null) return null;
    String resultName;
    if (fontName == null) {
      resultName = currentFont.getName();
    } else {
      Font testFont = new Font(fontName, Font.PLAIN, 10);
      if (testFont.canDisplay('a') && testFont.canDisplay('1')) {
        resultName = fontName;
      } else {
        resultName = currentFont.getName();
      }
    }
    Font font = new Font(resultName, style >= 0 ? style : currentFont.getStyle(), size >= 0 ? size : currentFont.getSize());
    boolean isMac = System.getProperty("os.name", "").toLowerCase(Locale.ENGLISH).startsWith("mac");
    Font fontWithFallback = isMac ? new Font(font.getFamily(), font.getStyle(), font.getSize()) : new StyleContext().getFont(font.getFamily(), font.getStyle(), font.getSize());
    return fontWithFallback instanceof FontUIResource ? fontWithFallback : new FontUIResource(fontWithFallback);
  }

  public JComponent $$$getRootComponent$$$() {
    return mainPanel;
  }

}

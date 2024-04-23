/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.belief.gui;

import edu.tufts.hrilab.belief.BeliefComponent;
import edu.tufts.hrilab.fol.Factory;
import com.intellij.uiDesigner.core.GridConstraints;
import com.intellij.uiDesigner.core.GridLayoutManager;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.swing.*;
import javax.swing.border.TitledBorder;
import java.awt.*;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.WindowEvent;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class BeliefGui extends JFrame {
  private static Logger log = LoggerFactory.getLogger(BeliefGui.class);

  private BeliefComponent component;

  //GUI Variables
  DefaultListModel currentBelsList = new DefaultListModel();
  DefaultListModel timelineBelsList = new DefaultListModel();

  //Data Variables
  private String[] previousBeliefs;
  private String[] unformatedList;
  private boolean unpaused = true;

  // execution loop
  private final ScheduledExecutorService guiUpdateExecutor = Executors.newScheduledThreadPool(1);

  private JPanel beliefPanel;
  private JScrollPane timeline_scroll;
  private JScrollPane current_scroll;
  private JLabel timeline_lbl;
  private JLabel current_lbl;
  private JLabel query_lbl;
  private JTextField query_txtFld;
  private JTextArea queryResults_txtArea;
  private JButton query_btn;
  private JButton pause_btn;
  private JList timeline_list;
  private JList current_list;
  private JPanel queryPanel;
  private JPanel updatePanel;

  public BeliefGui(BeliefComponent component) {
    super("BeliefGUI");
    this.component = component;

    current_list.setModel(currentBelsList);
    timeline_list.setModel(timelineBelsList);
    timeline_list.setCellRenderer(new BeliefGui.MyListRenderer());

    //get initial beliefs
    String allBeliefs = component.getNotificationProver().getTheory();
    previousBeliefs = allBeliefs.split(".\n\n");

    unformatedList = previousBeliefs;
    formatList(currentBelsList);

    query_btn.addActionListener(actionEvent -> {
      String query = query_txtFld.getText();
      String belief = component.getNotificationProver().queryBelief(Factory.createPredicate(query)).toString();
      if (belief.equals("[{}]"))
        belief = "true";
      else if (belief.equals("[]"))
        belief = "false";
      queryResults_txtArea.setText(belief);
    });
    pause_btn.addActionListener(actionEvent -> {
      if (unpaused) {
        pause_btn.setText("Resume");
        unpaused = false;
      } else {
        pause_btn.setText("Pause");
        unpaused = true;
      }
    });
    query_txtFld.addKeyListener(new KeyAdapter() {
      @Override
      public void keyPressed(KeyEvent keyEvent) {
        if (keyEvent.getKeyChar() == KeyEvent.VK_ENTER)
          query_btn.doClick();
      }
    });

    this.add(beliefPanel);
    this.setSize(600, 600);
    this.setVisible(true);

    guiUpdateExecutor.scheduleAtFixedRate(() -> {
      try {
        EventQueue.invokeAndWait(() -> refreshGui());
      } catch (InterruptedException | InvocationTargetException e) {
        log.error("Error updating BeliefGui.", e);
      }
    }, 0, 500, TimeUnit.MILLISECONDS);
  }

  public void shutdown(){
    this.dispatchEvent(new WindowEvent(this, WindowEvent.WINDOW_CLOSING));
  }


  /**
   * Will format the list so that the elements are spaced out correctly.
   *
   * @param list : the list you want to format
   */
  public void formatList(DefaultListModel list) {
    for (int i = 0; i < unformatedList.length; i++) {
      String s = unformatedList[i];
      if (!s.isEmpty()) {
        // Adds to front of list
        // Ensures that a line does not only contain a closing parenthesis
        if (s.replace(" ", "").length() > 1)
          list.add(0, s);
      } else
        // Adds to front of list
        list.add(0, "\n");
    }
  }

  /**
   * Will check the previous list to the current list. Based on the differences
   * it will assign those differences as assertions or retractions.
   * <p>
   * NOTE: The red or green highlighting depends on assert or retract being in the
   * string. Changing assert to something different will require that you change
   * the key word in getListCellRendererComponent method.
   *
   * @param allBeliefsArray
   */
  public void refreshTimeline(String[] allBeliefsArray) {
    List<String> toUpdate = new ArrayList<>();
    if (allBeliefsArray != previousBeliefs) {
      for (String s : allBeliefsArray) {
        if (!Arrays.asList(previousBeliefs).contains(s)) {
          toUpdate.add("assert:" + s);
        }
      }
      for (String s : previousBeliefs) {
        if (!Arrays.asList(allBeliefsArray).contains(s)) {
          toUpdate.add("retract:" + s);
        }
      }
    }

    String[] updatedList = new String[toUpdate.size()];
    for (int i = 0; i < toUpdate.size(); i++) {
      updatedList[i] = toUpdate.get(i);
    }
    unformatedList = updatedList;
    formatList(timelineBelsList);
  }

  public void refreshCurrent(String[] allBeliefsArray, boolean unpaused) {
    if (unpaused) {
      currentBelsList.clear();
      unformatedList = allBeliefsArray;
      formatList(currentBelsList);
    }
  }

  /**
   * Will refresh the current list and the timeline lists
   */
  public void refreshGui() {
    //update belief history
    String allBeliefs = component.getNotificationProver().getTheory();
    String[] allBeliefsArray = allBeliefs.split(".\n\n");

    refreshTimeline(allBeliefsArray);
    refreshCurrent(allBeliefsArray, unpaused);

    previousBeliefs = allBeliefsArray;
  }

  /**
   * Custom Render class to allow different colors for assert or retract statements
   */
  private class MyListRenderer extends DefaultListCellRenderer {
    private HashMap theChosen = new HashMap();

    public Component getListCellRendererComponent(JList list,
                                                  Object value, int index, boolean isSelected,
                                                  boolean cellHasFocus) {
      super.getListCellRendererComponent(list, value, index,
              isSelected, cellHasFocus);
      // Will check if the word assert is in the string. If you wish to change this
      // Be sure to replace assert both here and in the refreshtimeline function
      if (value.toString().startsWith("assert")) {
        theChosen.put(value, "chosen");
      }
      if (theChosen.containsKey(value)) {
        setForeground(Color.green.darker());
      } else {
        setForeground(Color.red);
      }
      return this;
    }
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
    beliefPanel = new JPanel();
    beliefPanel.setLayout(new GridLayoutManager(7, 1, new Insets(5, 5, 5, 5), -1, -1));
    queryPanel = new JPanel();
    queryPanel.setLayout(new GridLayoutManager(4, 1, new Insets(10, 5, 5, 5), -1, -1));
    beliefPanel.add(queryPanel, new GridConstraints(5, 0, 1, 1, GridConstraints.ANCHOR_CENTER, GridConstraints.FILL_BOTH, GridConstraints.SIZEPOLICY_CAN_SHRINK | GridConstraints.SIZEPOLICY_CAN_GROW, GridConstraints.SIZEPOLICY_CAN_SHRINK | GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
    queryPanel.setBorder(BorderFactory.createTitledBorder(BorderFactory.createEtchedBorder(), null, TitledBorder.DEFAULT_JUSTIFICATION, TitledBorder.DEFAULT_POSITION, null, null));
    query_lbl = new JLabel();
    query_lbl.setText("Query");
    queryPanel.add(query_lbl, new GridConstraints(0, 0, 1, 1, GridConstraints.ANCHOR_CENTER, GridConstraints.FILL_NONE, GridConstraints.SIZEPOLICY_FIXED, GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
    query_txtFld = new JTextField();
    queryPanel.add(query_txtFld, new GridConstraints(1, 0, 1, 1, GridConstraints.ANCHOR_WEST, GridConstraints.FILL_HORIZONTAL, GridConstraints.SIZEPOLICY_WANT_GROW, GridConstraints.SIZEPOLICY_FIXED, null, new Dimension(150, -1), null, 0, false));
    queryResults_txtArea = new JTextArea();
    queryResults_txtArea.setEditable(false);
    queryPanel.add(queryResults_txtArea, new GridConstraints(2, 0, 1, 1, GridConstraints.ANCHOR_CENTER, GridConstraints.FILL_BOTH, GridConstraints.SIZEPOLICY_WANT_GROW, GridConstraints.SIZEPOLICY_WANT_GROW, null, new Dimension(150, 50), null, 0, false));
    query_btn = new JButton();
    query_btn.setText("Query Belief");
    queryPanel.add(query_btn, new GridConstraints(3, 0, 1, 1, GridConstraints.ANCHOR_CENTER, GridConstraints.FILL_HORIZONTAL, GridConstraints.SIZEPOLICY_CAN_SHRINK | GridConstraints.SIZEPOLICY_CAN_GROW, GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
    updatePanel = new JPanel();
    updatePanel.setLayout(new GridLayoutManager(5, 1, new Insets(5, 5, 5, 5), -1, -1));
    beliefPanel.add(updatePanel, new GridConstraints(0, 0, 4, 1, GridConstraints.ANCHOR_CENTER, GridConstraints.FILL_BOTH, GridConstraints.SIZEPOLICY_CAN_SHRINK | GridConstraints.SIZEPOLICY_CAN_GROW, GridConstraints.SIZEPOLICY_CAN_SHRINK | GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
    updatePanel.setBorder(BorderFactory.createTitledBorder(BorderFactory.createEtchedBorder(), null, TitledBorder.DEFAULT_JUSTIFICATION, TitledBorder.DEFAULT_POSITION, null, null));
    timeline_scroll = new JScrollPane();
    updatePanel.add(timeline_scroll, new GridConstraints(1, 0, 1, 1, GridConstraints.ANCHOR_CENTER, GridConstraints.FILL_BOTH, GridConstraints.SIZEPOLICY_CAN_SHRINK | GridConstraints.SIZEPOLICY_WANT_GROW, GridConstraints.SIZEPOLICY_CAN_SHRINK | GridConstraints.SIZEPOLICY_WANT_GROW, null, null, null, 0, false));
    timeline_list = new JList();
    timeline_scroll.setViewportView(timeline_list);
    current_scroll = new JScrollPane();
    updatePanel.add(current_scroll, new GridConstraints(3, 0, 1, 1, GridConstraints.ANCHOR_CENTER, GridConstraints.FILL_BOTH, GridConstraints.SIZEPOLICY_CAN_SHRINK | GridConstraints.SIZEPOLICY_WANT_GROW, GridConstraints.SIZEPOLICY_CAN_SHRINK | GridConstraints.SIZEPOLICY_WANT_GROW, null, null, null, 0, false));
    current_list = new JList();
    current_scroll.setViewportView(current_list);
    timeline_lbl = new JLabel();
    timeline_lbl.setText("Belief Timeline");
    updatePanel.add(timeline_lbl, new GridConstraints(0, 0, 1, 1, GridConstraints.ANCHOR_CENTER, GridConstraints.FILL_NONE, GridConstraints.SIZEPOLICY_FIXED, GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
    current_lbl = new JLabel();
    current_lbl.setText("Current Beliefs");
    updatePanel.add(current_lbl, new GridConstraints(2, 0, 1, 1, GridConstraints.ANCHOR_CENTER, GridConstraints.FILL_NONE, GridConstraints.SIZEPOLICY_FIXED, GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
    pause_btn = new JButton();
    pause_btn.setText("Pause");
    updatePanel.add(pause_btn, new GridConstraints(4, 0, 1, 1, GridConstraints.ANCHOR_CENTER, GridConstraints.FILL_NONE, GridConstraints.SIZEPOLICY_CAN_SHRINK | GridConstraints.SIZEPOLICY_CAN_GROW, GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
  }

  public JComponent $$$getRootComponent$$$() {
    return beliefPanel;
  }

}

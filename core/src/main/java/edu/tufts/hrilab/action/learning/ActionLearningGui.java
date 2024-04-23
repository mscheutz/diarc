/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.learning;
import java.util.Vector;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.EventSpec;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import javax.swing.table.DefaultTableModel;
import javax.swing.*;
/**
 *
 * @author tmfrasca
 */
public class ActionLearningGui extends javax.swing.JFrame {

  private static final Logger log = LoggerFactory.getLogger(ActionLearningGui.class);

  private LearningState currentLearningState;
  private UpdateGui updateGui;
  private boolean currentLearningStateChange = false;
  private DefaultTableModel tableModel;
  private JTable table;


  public ActionLearningGui() {
    init();
    updateGui = new UpdateGui(this, log);
    updateGui.start();
    log.trace("Starting action learning gui");

  }

  private void init() {
    setSize(750,300);
    setDefaultCloseOperation(DISPOSE_ON_CLOSE);
    setTitle("Action Learning State");
    initTable();
    //add(table);
    getContentPane().add(table, java.awt.BorderLayout.PAGE_START);
    setVisible(true);
  }

  private void initTable() {
    table = new JTable();
    table.setName("Data Table"); // NOI18N
    table.setAutoResizeMode(javax.swing.JTable.AUTO_RESIZE_LAST_COLUMN); // doesn't work properly
    table.setFillsViewportHeight(true);
    tableModel = new DefaultTableModel();
    resetTableData();
    table.setModel(tableModel);
    //table.setMinimumSize(new java.awt.Dimension(50, 120));
    //table.setPreferredSize(new java.awt.Dimension(300, 200));

  }

  public void setLearningState(LearningState learningState) {
    log.debug("Setting learning state for gui");
    resetTableData();
    currentLearningState = learningState;
    currentLearningStateChange = true;
  }

  private void resetTableData() {
    tableModel.setDataVector(
        new Object [][] {
        {"Name", null},
        {"Arguments", null},
        {"Conditions", null},
        {"Effects", null},
        {"Steps", null}
      },
      new String [] {
        "", ""
      }
    );
  }

  boolean getCurrentLearningStateChange() {
    return currentLearningStateChange;
  }

  void setCurrentLearningStateChange() {
    currentLearningStateChange = false;
  }

  LearningState getLearningState() {
    return currentLearningState;
  }


  void setData() {
    if (currentLearningState != null) {
      int i=0;
      boolean firstElement = true;
      log.debug("setting data");
      setRow(i++, firstElement, currentLearningState.getName(), "Name");
      for (ActionBinding binding : currentLearningState.getRoles()) {
        setRow(i++, firstElement, binding.getName(),"Arguments");
        firstElement = false;
      }
      firstElement = true;
      if (currentLearningState.getConditions().isEmpty()) {
        setRow(i++, firstElement, "","Conditions");
      } else {
        for (Condition condition : currentLearningState.getConditions()) {
          setRow(i++, firstElement, condition.toString(), "Conditions");
          firstElement = false;

        }
      }
      firstElement = true;
      if (currentLearningState.getEffects().isEmpty()) {
        setRow(i++, firstElement, "","Effects");
      } else {
        for (Effect effect : currentLearningState.getEffects()) {
          setRow(i++, firstElement, effect.toString(), "Effects");
          firstElement = false;
        }
      }
      firstElement = true;
      if (currentLearningState.getEventSpecs().isEmpty()) {
        setRow(i++, firstElement, "","Steps");
      } else {
        for (EventSpec event : currentLearningState.getEventSpecs()) {
          setRow(i++, firstElement, event.getType().name() + " " + event.toString(), "Steps");
          firstElement = false;
        }
      }
    }
  }

  void setRow(int row, boolean subRow, String value, String rowName) {
    if (row >= tableModel.getRowCount()) {
      Vector<String> vector = new Vector<>(2);
      if (subRow) {
        vector.add(rowName);
      } else {
        vector.add("");
      }
      vector.add(value);
      tableModel.addRow(vector);
    } else {
      if (subRow) {
        tableModel.setValueAt(rowName, row, 0);
      } else {
        tableModel.setValueAt("", row, 0);
      }
      tableModel.setValueAt(value, row, 1);
    }


  }

  public void clear() {
    resetTableData();
  }
}


class UpdateGui extends Thread {
  private LearningState learningState;
  private int numElements;
  private ActionLearningGui gui;
  private Logger log;
  UpdateGui(ActionLearningGui gui, Logger log) {
    this.log = log;
    this.gui = gui;
    numElements = 0;
  }
  @Override
  public void run() {
    while(true) {

      if (gui.getCurrentLearningStateChange()) {
        learningState = gui.getLearningState();
        gui.setCurrentLearningStateChange();
        numElements = 0;
      }
      if (learningState != null) {
        int newNum =  learningState.getConditions().size();
        newNum += learningState.getRoles().size();
        newNum += learningState.getEffects().size();
        newNum += learningState.getEventSpecs().size();
        if (newNum > numElements) {
          gui.setData();
          numElements = newNum;
        }
      }
      quickNap();
    }
  }
  private void quickNap() {
    try {
      sleep(100);
    } catch (InterruptedException ex) {
    }
  }
}


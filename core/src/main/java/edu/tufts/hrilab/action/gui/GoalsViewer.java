/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.gui;

import edu.tufts.hrilab.action.manager.ExecutionManager;
import edu.tufts.hrilab.action.goal.Goal;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.Future;

public class GoalsViewer extends JPanel {
    private static final Logger log = LoggerFactory.getLogger(GoalsViewer.class);
    private JList currentGoalsList;
    private JPanel goalsViewerPanel;
    private JTabbedPane goalTabPane;
    private JList pastGoalsList;
    private JSplitPane goalsSplitPane;
    private JSplitPane goalsViewSplit;
    private ExecutionManager executionManager;
    private Map<Goal, Future> currentGoalsMap;
    private ArrayList<Goal> currentGoals;
    private Set<Goal> pastGoalsSet;
    private ArrayList<Goal> pastGoals;
    private DefaultListModel currentGoalsModel;
    private DefaultListModel pastGoalsModel;

    GoalsViewer(ExecutionManager em, Map<Goal, Future> currentGoalsMap, Set<Goal> pastGoalsSet) {
        this.currentGoalsMap = currentGoalsMap;
        this.pastGoalsSet = pastGoalsSet;
        executionManager = em;
        currentGoalsModel = new DefaultListModel<>();
        currentGoalsList.setModel(currentGoalsModel);
        for (Goal goal : currentGoalsMap.keySet()) {
            currentGoals.add(goal);
            currentGoalsModel.addElement(goal.toString());
        }
        pastGoalsModel = new DefaultListModel<>();
        pastGoalsList.setModel(pastGoalsModel);
        for (Goal goal : pastGoalsSet) {
            pastGoals.add(goal);
            pastGoalsModel.addElement(goal.toString());
        }


        currentGoals = new ArrayList<>();
        add(goalsViewerPanel);
        currentGoalsList.setVisible(true);
        setVisible(true);
        setupListeners();
    }

    public void updateGoals() {
        currentGoalsModel.clear();
        log.warn("updating goals");
        currentGoals = new ArrayList<>();
        for (Goal goal : currentGoalsMap.keySet()) {
            //log.error(goal.toString());
            currentGoals.add(goal);
            currentGoalsModel.addElement(goal.toString());
        }
        currentGoalsList.setVisible(true);
        pastGoalsModel.clear();
        pastGoals = new ArrayList<>();
        for (Goal goal : pastGoalsSet) {
            //log.error(goal.toString());
            pastGoals.add(goal);
            pastGoalsModel.addElement(goal.toString());
        }
        pastGoalsList.setVisible(true);
    }

    private void setupListeners() {
        currentGoalsList.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                JList list = (JList) mouseEvent.getSource();
                int index = list.locationToIndex(mouseEvent.getPoint());
                GoalInfo goalInfo = new GoalInfo(currentGoals.get(index));
                goalTabPane.add(goalInfo);
            }
        });
        pastGoalsList.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                JList list = (JList) mouseEvent.getSource();
                int index = list.locationToIndex(mouseEvent.getPoint());
                GoalInfo goalInfo = new GoalInfo(pastGoals.get(index));
                goalTabPane.add(goalInfo);
            }
        });
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
        goalsViewerPanel = new JPanel();
        goalsViewerPanel.setLayout(new GridBagLayout());
        goalsViewerPanel.setAlignmentX(0.0f);
        goalsViewerPanel.setAlignmentY(0.0f);
        goalsViewerPanel.setMinimumSize(new Dimension(800, 500));
        goalsViewSplit = new JSplitPane();
        goalsViewSplit.setDividerLocation(200);
        goalsViewSplit.setMinimumSize(new Dimension(800, 500));
        goalsViewSplit.setPreferredSize(new Dimension(800, 500));
        GridBagConstraints gbc;
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.fill = GridBagConstraints.BOTH;
        goalsViewerPanel.add(goalsViewSplit, gbc);
        goalTabPane = new JTabbedPane();
        goalsViewSplit.setRightComponent(goalTabPane);
        goalsSplitPane = new JSplitPane();
        goalsSplitPane.setDividerLocation(128);
        goalsSplitPane.setDoubleBuffered(false);
        goalsSplitPane.setLastDividerLocation(160);
        goalsSplitPane.setName("CurrentGoals");
        goalsSplitPane.setOrientation(0);
        goalsViewSplit.setLeftComponent(goalsSplitPane);
        final JSplitPane splitPane1 = new JSplitPane();
        splitPane1.setDividerSize(0);
        splitPane1.setOrientation(0);
        goalsSplitPane.setRightComponent(splitPane1);
        final JLabel label1 = new JLabel();
        label1.setHorizontalAlignment(0);
        label1.setHorizontalTextPosition(0);
        label1.setText("Current Goals");
        splitPane1.setLeftComponent(label1);
        final JScrollPane scrollPane1 = new JScrollPane();
        splitPane1.setRightComponent(scrollPane1);
        currentGoalsList = new JList();
        scrollPane1.setViewportView(currentGoalsList);
        final JSplitPane splitPane2 = new JSplitPane();
        splitPane2.setDividerSize(0);
        splitPane2.setOrientation(0);
        goalsSplitPane.setLeftComponent(splitPane2);
        final JLabel label2 = new JLabel();
        label2.setHorizontalAlignment(0);
        label2.setHorizontalTextPosition(0);
        label2.setText("Past Goals");
        splitPane2.setLeftComponent(label2);
        final JScrollPane scrollPane2 = new JScrollPane();
        scrollPane2.setAlignmentX(0.0f);
        scrollPane2.setAlignmentY(0.0f);
        splitPane2.setRightComponent(scrollPane2);
        pastGoalsList = new JList();
        pastGoalsList.setAlignmentX(0.0f);
        pastGoalsList.setAlignmentY(0.0f);
        scrollPane2.setViewportView(pastGoalsList);
    }

    public JComponent $$$getRootComponent$$$() {
        return goalsViewerPanel;
    }

}

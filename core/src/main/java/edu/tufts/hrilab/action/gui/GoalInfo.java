/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.gui;

import edu.tufts.hrilab.action.goal.Goal;

import javax.swing.*;
import java.awt.*;

public class GoalInfo extends JPanel {
    private JPanel goalPanel;
    private JLabel predicateLabel;
    private JTextField predicate;
    private JLabel idLabel;
    private JTextField id;
    private JLabel startTimeLabel;
    private JLabel actorLabel;
    private JTextField actor;
    private JTextField status;
    private JLabel statusLabel;
    private JTextField terminated;
    private JTextField endTime;
    private JLabel endTimeLabel;
    private JTextField startTime;
    private JList list1;
    private JList list2;
    private Goal goal;

    GoalInfo(Goal goal) {
        this.goal = goal;
        predicate.setText(goal.getPredicate().toString());
        id.setText(Long.toString(goal.getId()));
        startTime.setText(Long.toString(goal.getStartTime()));
        endTime.setText(Long.toString(goal.getEndTime()));
        actor.setText(goal.getActor().toString());
        status.setText(goal.getStatus().toString());
        add(goalPanel);
        setupListeners();
    }

    private void setupListeners() {
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
        goalPanel = new JPanel();
        goalPanel.setLayout(new GridBagLayout());
        final JPanel panel1 = new JPanel();
        panel1.setLayout(new GridBagLayout());
        GridBagConstraints gbc;
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.BOTH;
        goalPanel.add(panel1, gbc);
        predicateLabel = new JLabel();
        predicateLabel.setText("Goal");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        panel1.add(predicateLabel, gbc);
        predicate = new JTextField();
        predicate.setBackground(new Color(-327937));
        predicate.setEditable(false);
        predicate.setEnabled(false);
        predicate.setFocusable(false);
        predicate.setText("");
        gbc = new GridBagConstraints();
        gbc.gridx = 1;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        panel1.add(predicate, gbc);
        actorLabel = new JLabel();
        actorLabel.setText("Agent");
        gbc = new GridBagConstraints();
        gbc.gridx = 2;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        panel1.add(actorLabel, gbc);
        actor = new JTextField();
        actor.setBackground(new Color(-327937));
        actor.setEditable(false);
        gbc = new GridBagConstraints();
        gbc.gridx = 3;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        panel1.add(actor, gbc);
        final JPanel panel2 = new JPanel();
        panel2.setLayout(new GridBagLayout());
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 1;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.BOTH;
        goalPanel.add(panel2, gbc);
        id = new JTextField();
        id.setBackground(new Color(-327937));
        id.setEditable(false);
        id.setText("");
        gbc = new GridBagConstraints();
        gbc.gridx = 1;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        panel2.add(id, gbc);
        startTimeLabel = new JLabel();
        startTimeLabel.setText("Start Time");
        gbc = new GridBagConstraints();
        gbc.gridx = 2;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        panel2.add(startTimeLabel, gbc);
        startTime = new JTextField();
        startTime.setBackground(new Color(-327937));
        startTime.setEditable(false);
        startTime.setText("");
        gbc = new GridBagConstraints();
        gbc.gridx = 3;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        panel2.add(startTime, gbc);
        idLabel = new JLabel();
        idLabel.setText("Goal ID");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        panel2.add(idLabel, gbc);
        endTimeLabel = new JLabel();
        endTimeLabel.setText("End Time");
        gbc = new GridBagConstraints();
        gbc.gridx = 4;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        panel2.add(endTimeLabel, gbc);
        endTime = new JTextField();
        endTime.setBackground(new Color(-327937));
        endTime.setEditable(false);
        endTime.setText("");
        gbc = new GridBagConstraints();
        gbc.gridx = 5;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        panel2.add(endTime, gbc);
        final JPanel panel3 = new JPanel();
        panel3.setLayout(new GridBagLayout());
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 2;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.BOTH;
        goalPanel.add(panel3, gbc);
        statusLabel = new JLabel();
        statusLabel.setText("Status");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        panel3.add(statusLabel, gbc);
        status = new JTextField();
        status.setBackground(new Color(-327937));
        status.setEditable(false);
        gbc = new GridBagConstraints();
        gbc.gridx = 1;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        panel3.add(status, gbc);
        final JPanel panel4 = new JPanel();
        panel4.setLayout(new GridBagLayout());
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 3;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.BOTH;
        goalPanel.add(panel4, gbc);
        final JLabel label1 = new JLabel();
        label1.setText("Wait Conditions");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        panel4.add(label1, gbc);
        final JLabel label2 = new JLabel();
        label2.setText("Failure Conditions");
        gbc = new GridBagConstraints();
        gbc.gridx = 1;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        panel4.add(label2, gbc);
        list1 = new JList();
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 1;
        gbc.fill = GridBagConstraints.BOTH;
        panel4.add(list1, gbc);
        list2 = new JList();
        gbc = new GridBagConstraints();
        gbc.gridx = 1;
        gbc.gridy = 1;
        gbc.fill = GridBagConstraints.BOTH;
        panel4.add(list2, gbc);
    }

    public JComponent $$$getRootComponent$$$() {
        return goalPanel;
    }

}

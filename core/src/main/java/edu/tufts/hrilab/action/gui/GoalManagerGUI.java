/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.gui;

import edu.tufts.hrilab.action.manager.ExecutionManager;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;

import javax.swing.*;
import javax.swing.border.TitledBorder;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.Future;

public class GoalManagerGUI {
    private JTextField submitGoalField;
    private JButton submitGoalButton;
    private JButton goalsButton;
    private JButton databaseButton;
    private JPanel contentPanel;
    private JPanel submitPanel;
    private JPanel gmPanel;
    private JButton fetch_lgButton;
    private JButton ap_caddy;
    private JButton goToPoseButton;
    private JButton assembleButton;
    private JButton savePerfMeasuresButton;
    private JButton submitEstPerfButton;
    private JTextField assessMod;
    private JRadioButton befRB;
    private JRadioButton aftRB;
    private JRadioButton durRB;
    private JRadioButton comRB;
    private JRadioButton staRB;
    private JRadioButton modRB;
    private ButtonGroup phaseBG;
    private ButtonGroup typeBG;
    private JTextField estGoal;
    private JLabel submitGoalLabel;
    private JButton suspendButton;
    private JButton resumeButton;
    private JPanel randomGoalsPanel;
    private JPanel assessGoalPanel;
    private JPanel phasePanel;
    private JPanel assessModPanel;
    private JPanel goalContentPanel;
    private JPanel submitGoalPanel;
    private JButton setStateButton;
    private JTextField setStateTextField;
    private JPanel utilMethods;
    private JTextField fetchObjField;
    private JButton fetchButton;
    private JRadioButton noneRB;
    private JLabel modification;
    private JLabel assessGoalLabel;
    private JTextField suspendGoalField;
    private JTextField resumeGoalField;
    private JTextField setAgentTxtField;
    private JButton setAgentButton;
    private JLabel setAgentLabel;
    private JTextField perfFileNameTxtField;
    private JPanel submitActionPanel;
    private JTextField submitActionField;
    private JButton submitActionButton;
    private JLabel submitActionLabel;
    private static ExecutionManager executionManager;
    private String dir;
    private DatabaseViewer dbViewer;
    private GoalsViewer goalsViewer;
    private Map<Goal, Future> goals;
    private JFrame gmGUI;
    //FIXME: better way to handle agents for submitting goals, right now hardcoded to andy for fetch stuff
    private Symbol agent;

    public GoalManagerGUI(ExecutionManager em, String path, Map<Goal, Future> goals, Set<Goal> pastGoals) {
        executionManager = em;
        dir = path;
        goalsViewer = new GoalsViewer(em, goals, pastGoals);
        dbViewer = new DatabaseViewer(em, path);
        gmGUI = new JFrame("Goal Manager GUI");
        gmGUI.setContentPane(gmPanel);
        gmGUI.setVisible(true);
        gmGUI.pack();
        phaseBG = new ButtonGroup();
        typeBG = new ButtonGroup();
        setupListeners();
        setupRadioButtonGroups();
        agent = Factory.createSymbol("self");
        if (contentPanel.getComponentCount() > 0) {
            contentPanel.remove(0);
        }
        contentPanel.add(dbViewer);
        contentPanel.updateUI();
    }


    public void updateGoals() {
        goalsViewer.updateGoals();
    }

    private void setupRadioButtonGroups() {
        befRB.setActionCommand("before");
        durRB.setActionCommand("during");
        aftRB.setActionCommand("after");
        comRB.setActionCommand("complete");
        staRB.setActionCommand("state");
        modRB.setActionCommand("modify");
        noneRB.setActionCommand("none");
        phaseBG.add(befRB);
        phaseBG.add(durRB);
        phaseBG.add(aftRB);
        typeBG.add(comRB);
        typeBG.add(staRB);
        typeBG.add(modRB);
        typeBG.add(noneRB);
    }

    private void getAgent() {
        if (!setAgentTxtField.getText().isEmpty()) {
            agent = Factory.createSymbol(setAgentTxtField.getText());
        }
    }

    private void setupListeners() {
        databaseButton.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                if (contentPanel.getComponentCount() > 0) {
                    contentPanel.remove(0);
                }
                contentPanel.add(dbViewer);
                contentPanel.updateUI();
            }
        });

        goalsButton.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                if (contentPanel.getComponentCount() > 0) {
                    contentPanel.remove(0);
                }
                contentPanel.add(goalsViewer);
                contentPanel.updateUI();
            }
        });

        submitGoalButton.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                if (!submitGoalField.getText().isEmpty()) {
                    getAgent();
                    executionManager.submitGoal(new Goal(agent,
                        Factory.createPredicate(submitGoalField.getText())));
                }
            }
        });
        submitGoalField.addActionListener(new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                if (!submitGoalField.getText().isEmpty()) {
                    getAgent();
                    executionManager.submitGoal(new Goal(agent,
                        Factory.createPredicate(submitGoalField.getText())));
                }
            }
        });
        submitActionButton.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                if (!submitActionField.getText().isEmpty()) {
                    getAgent();
                    executionManager.submitGoal(Factory.createPredicate(submitActionField.getText()));
                }
            }
        });
        submitActionField.addActionListener(new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                if (!submitActionField.getText().isEmpty()) {
                    getAgent();
                    executionManager.submitGoal(Factory.createPredicate(submitActionField.getText()));
                }
            }
        });
        savePerfMeasuresButton.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                if (!perfFileNameTxtField.getText().isEmpty()) {
                    getAgent();
                    String fn = perfFileNameTxtField.getText();
                    executionManager.submitGoal(
                        Factory.createPredicate("savePerformanceMeasures", agent.toString(), fn));
                }
            }
        });
        goToPoseButton.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                getAgent();
                executionManager.submitGoal(Factory.createPredicate("goToPose", agent.toString(), "carry"));
            }
        });
        ap_caddy.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                getAgent();
                executionManager.submitGoal(Factory.createPredicate("approach", agent.toString(), "location_0"));
            }
        });
        assembleButton.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                getAgent();
                executionManager.submitGoal(Factory.createPredicate("assemble", agent.toString(), "object_0"));
            }
        });
        submitEstPerfButton.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                String phase = null;
                if (phaseBG.getSelection() != null) {
                    phase = phaseBG.getSelection().getActionCommand();
                }
                if (phase == null) {
                    return;
                }
                String type = null;
                if (typeBG.getSelection() != null) {
                    type = typeBG.getSelection().getActionCommand();
                }
                String modification;
                if (type == null || type.equals("none")) {
                    modification = Factory.createPredicate("if", "none()").toString();
                } else {
                    modification = Factory.createPredicate("if", Factory.createPredicate(type, assessMod.getText())).toString();
                }
                getAgent();
                executionManager.submitGoal(
                    Factory.createPredicate("estimatePerformanceMeasures",
                        agent.toString(),
                        estGoal.getText(),
                        phase,
                        modification)
                );
            }
        });
        suspendButton.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                if (!suspendGoalField.getText().isEmpty()) {
                    getAgent();
                    String goal = suspendGoalField.getText();
                    executionManager.submitGoal(
                        Factory.createPredicate("suspendGoal", agent.toString(), goal));
                }
            }
        });
        resumeButton.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                if (!resumeGoalField.getText().isEmpty()) {
                    getAgent();
                    String goal = suspendGoalField.getText();
                    executionManager.submitGoal(
                        Factory.createPredicate("resumeGoal", agent.toString(), goal));
                }
            }
        });
        setStateButton.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                if (!setStateTextField.getText().isEmpty()) {
                    getAgent();
                    executionManager.submitGoal(
                        Factory.createPredicate("setState", agent.toString(), setStateTextField.getText()));
                }
            }
        });
        fetchButton.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent mouseEvent) {
                super.mouseClicked(mouseEvent);
                if (!fetchObjField.getText().isEmpty()) {
                    getAgent();
                    String obj = fetchObjField.getText();
                    executionManager.submitGoal(
                        Factory.createPredicate("fetch", agent.toString(), "object_" + obj, "location_" + obj));
                }
            }
        });
        fetchObjField.addActionListener(new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                if (!fetchObjField.getText().isEmpty()) {
                    getAgent();
                    String obj = fetchObjField.getText();
                    executionManager.submitGoal(
                        Factory.createPredicate("fetch", agent.toString(), "object_" + obj, "location_" + obj));
                }
            }
        });
        setAgentTxtField.addActionListener(new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                if (!setAgentTxtField.getText().isEmpty()) {
                    agent = Factory.createSymbol(setAgentTxtField.getText());
                }
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
        gmPanel = new JPanel();
        gmPanel.setLayout(new BorderLayout(0, 0));
        gmPanel.setMaximumSize(new Dimension(1920, 1080));
        gmPanel.setMinimumSize(new Dimension(800, 800));
        gmPanel.setPreferredSize(new Dimension(800, 800));
        final JPanel panel1 = new JPanel();
        panel1.setLayout(new GridBagLayout());
        gmPanel.add(panel1, BorderLayout.CENTER);
        final JToolBar toolBar1 = new JToolBar();
        toolBar1.setAlignmentX(0.0f);
        toolBar1.setAlignmentY(0.0f);
        toolBar1.setBorderPainted(true);
        toolBar1.setFloatable(false);
        toolBar1.setMaximumSize(new Dimension(800, 30));
        toolBar1.setMinimumSize(new Dimension(800, 30));
        toolBar1.setPreferredSize(new Dimension(800, 30));
        GridBagConstraints gbc;
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.weightx = 1.0;
        gbc.weighty = 0.1;
        gbc.fill = GridBagConstraints.BOTH;
        panel1.add(toolBar1, gbc);
        goalsButton = new JButton();
        goalsButton.setHorizontalAlignment(0);
        goalsButton.setHorizontalTextPosition(0);
        goalsButton.setText("Goals");
        toolBar1.add(goalsButton);
        databaseButton = new JButton();
        databaseButton.setText("Database");
        toolBar1.add(databaseButton);
        contentPanel = new JPanel();
        contentPanel.setLayout(new BorderLayout(0, 0));
        contentPanel.setMaximumSize(new Dimension(1000, 1000));
        contentPanel.setMinimumSize(new Dimension(800, 700));
        contentPanel.setPreferredSize(new Dimension(800, 700));
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 1;
        gbc.weightx = 1.0;
        gbc.weighty = 4.0;
        gbc.fill = GridBagConstraints.BOTH;
        panel1.add(contentPanel, gbc);
        submitPanel = new JPanel();
        submitPanel.setLayout(new GridBagLayout());
        submitPanel.setMinimumSize(new Dimension(800, 300));
        submitPanel.setPreferredSize(new Dimension(800, 300));
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 2;
        gbc.fill = GridBagConstraints.BOTH;
        panel1.add(submitPanel, gbc);
        submitActionPanel = new JPanel();
        submitActionPanel.setLayout(new GridBagLayout());
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.weighty = 0.05;
        gbc.fill = GridBagConstraints.BOTH;
        submitPanel.add(submitActionPanel, gbc);
        submitActionField = new JTextField();
        gbc = new GridBagConstraints();
        gbc.gridx = 1;
        gbc.gridy = 0;
        gbc.weightx = 0.8;
        gbc.anchor = GridBagConstraints.WEST;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        submitActionPanel.add(submitActionField, gbc);
        submitActionButton = new JButton();
        submitActionButton.setText("Submit Action");
        gbc = new GridBagConstraints();
        gbc.gridx = 2;
        gbc.gridy = 0;
        gbc.weightx = 0.01;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        submitActionPanel.add(submitActionButton, gbc);
        submitActionLabel = new JLabel();
        submitActionLabel.setHorizontalAlignment(0);
        submitActionLabel.setText("Submit Action");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.weightx = 0.1;
        gbc.fill = GridBagConstraints.BOTH;
        submitActionPanel.add(submitActionLabel, gbc);
        submitGoalPanel = new JPanel();
        submitGoalPanel.setLayout(new GridBagLayout());
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 1;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.BOTH;
        submitPanel.add(submitGoalPanel, gbc);
        submitGoalPanel.setBorder(BorderFactory.createTitledBorder(BorderFactory.createRaisedBevelBorder(), null, TitledBorder.DEFAULT_JUSTIFICATION, TitledBorder.DEFAULT_POSITION, null, null));
        submitGoalField = new JTextField();
        submitGoalField.setColumns(50);
        submitGoalField.setMinimumSize(new Dimension(0, 40));
        submitGoalField.setName("submit");
        submitGoalField.setPreferredSize(new Dimension(556, 40));
        gbc = new GridBagConstraints();
        gbc.gridx = 3;
        gbc.gridy = 0;
        gbc.weightx = 3.0;
        gbc.weighty = 1.0;
        gbc.anchor = GridBagConstraints.WEST;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        submitGoalPanel.add(submitGoalField, gbc);
        submitGoalButton = new JButton();
        submitGoalButton.setMinimumSize(new Dimension(100, 40));
        submitGoalButton.setPreferredSize(new Dimension(100, 40));
        submitGoalButton.setText("Submit Goal");
        gbc = new GridBagConstraints();
        gbc.gridx = 4;
        gbc.gridy = 0;
        gbc.weightx = 0.25;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        submitGoalPanel.add(submitGoalButton, gbc);
        setAgentLabel = new JLabel();
        setAgentLabel.setHorizontalAlignment(0);
        setAgentLabel.setHorizontalTextPosition(0);
        setAgentLabel.setMaximumSize(new Dimension(10, 18));
        setAgentLabel.setMinimumSize(new Dimension(10, 40));
        setAgentLabel.setPreferredSize(new Dimension(10, 40));
        setAgentLabel.setText("Agent");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        submitGoalPanel.add(setAgentLabel, gbc);
        setAgentTxtField = new JTextField();
        setAgentTxtField.setColumns(50);
        setAgentTxtField.setMinimumSize(new Dimension(50, 40));
        setAgentTxtField.setPreferredSize(new Dimension(556, 30));
        setAgentTxtField.setText("self");
        setAgentTxtField.setToolTipText("set the agent for goal submission.");
        gbc = new GridBagConstraints();
        gbc.gridx = 1;
        gbc.gridy = 0;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.anchor = GridBagConstraints.WEST;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        submitGoalPanel.add(setAgentTxtField, gbc);
        submitGoalLabel = new JLabel();
        submitGoalLabel.setHorizontalAlignment(0);
        submitGoalLabel.setHorizontalTextPosition(0);
        submitGoalLabel.setMaximumSize(new Dimension(10, 18));
        submitGoalLabel.setMinimumSize(new Dimension(10, 40));
        submitGoalLabel.setText("Goal");
        gbc = new GridBagConstraints();
        gbc.gridx = 2;
        gbc.gridy = 0;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        submitGoalPanel.add(submitGoalLabel, gbc);
        assessGoalPanel = new JPanel();
        assessGoalPanel.setLayout(new GridBagLayout());
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 2;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.BOTH;
        submitPanel.add(assessGoalPanel, gbc);
        assessGoalPanel.setBorder(BorderFactory.createTitledBorder(BorderFactory.createRaisedBevelBorder(), null, TitledBorder.DEFAULT_JUSTIFICATION, TitledBorder.DEFAULT_POSITION, null, null));
        final JLabel label1 = new JLabel();
        label1.setHorizontalAlignment(0);
        label1.setHorizontalTextPosition(0);
        label1.setText("Estimate Performance");
        label1.setVerticalTextPosition(3);
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.gridwidth = 2;
        assessGoalPanel.add(label1, gbc);
        goalContentPanel = new JPanel();
        goalContentPanel.setLayout(new GridBagLayout());
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 1;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.BOTH;
        assessGoalPanel.add(goalContentPanel, gbc);
        phasePanel = new JPanel();
        phasePanel.setLayout(new GridBagLayout());
        gbc = new GridBagConstraints();
        gbc.gridx = 1;
        gbc.gridy = 0;
        gbc.gridheight = 5;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        goalContentPanel.add(phasePanel, gbc);
        befRB = new JRadioButton();
        befRB.setHorizontalAlignment(10);
        befRB.setHorizontalTextPosition(11);
        befRB.setText("before");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        phasePanel.add(befRB, gbc);
        durRB = new JRadioButton();
        durRB.setText("during");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 1;
        gbc.anchor = GridBagConstraints.WEST;
        phasePanel.add(durRB, gbc);
        aftRB = new JRadioButton();
        aftRB.setText("after");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 2;
        gbc.anchor = GridBagConstraints.WEST;
        phasePanel.add(aftRB, gbc);
        assessMod = new JTextField();
        assessMod.setToolTipText("assessment modification");
        gbc = new GridBagConstraints();
        gbc.gridx = 3;
        gbc.gridy = 2;
        gbc.gridheight = 3;
        gbc.weightx = 2.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        goalContentPanel.add(assessMod, gbc);
        assessModPanel = new JPanel();
        assessModPanel.setLayout(new GridBagLayout());
        gbc = new GridBagConstraints();
        gbc.gridx = 2;
        gbc.gridy = 0;
        gbc.gridheight = 5;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        goalContentPanel.add(assessModPanel, gbc);
        comRB = new JRadioButton();
        comRB.setText("complete");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.anchor = GridBagConstraints.WEST;
        assessModPanel.add(comRB, gbc);
        staRB = new JRadioButton();
        staRB.setText("state");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 1;
        gbc.anchor = GridBagConstraints.WEST;
        assessModPanel.add(staRB, gbc);
        modRB = new JRadioButton();
        modRB.setText("modify");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 2;
        gbc.anchor = GridBagConstraints.WEST;
        assessModPanel.add(modRB, gbc);
        noneRB = new JRadioButton();
        noneRB.setText("none");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 3;
        gbc.anchor = GridBagConstraints.WEST;
        assessModPanel.add(noneRB, gbc);
        estGoal = new JTextField();
        estGoal.setColumns(0);
        estGoal.setHorizontalAlignment(10);
        estGoal.setToolTipText("goal to assess");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 1;
        gbc.gridheight = 4;
        gbc.weightx = 2.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        goalContentPanel.add(estGoal, gbc);
        modification = new JLabel();
        modification.setMaximumSize(new Dimension(100, 20));
        modification.setMinimumSize(new Dimension(100, 20));
        modification.setPreferredSize(new Dimension(100, 20));
        modification.setText("assess mod");
        gbc = new GridBagConstraints();
        gbc.gridx = 3;
        gbc.gridy = 0;
        gbc.gridheight = 2;
        goalContentPanel.add(modification, gbc);
        assessGoalLabel = new JLabel();
        assessGoalLabel.setMaximumSize(new Dimension(50, 20));
        assessGoalLabel.setMinimumSize(new Dimension(50, 20));
        assessGoalLabel.setPreferredSize(new Dimension(50, 20));
        assessGoalLabel.setText("goal");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        goalContentPanel.add(assessGoalLabel, gbc);
        submitEstPerfButton = new JButton();
        submitEstPerfButton.setText("Submit");
        gbc = new GridBagConstraints();
        gbc.gridx = 1;
        gbc.gridy = 1;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        assessGoalPanel.add(submitEstPerfButton, gbc);
        utilMethods = new JPanel();
        utilMethods.setLayout(new GridBagLayout());
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 3;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.BOTH;
        submitPanel.add(utilMethods, gbc);
        utilMethods.setBorder(BorderFactory.createTitledBorder(BorderFactory.createRaisedBevelBorder(), null, TitledBorder.DEFAULT_JUSTIFICATION, TitledBorder.DEFAULT_POSITION, null, null));
        setStateButton = new JButton();
        setStateButton.setText("setState");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 1;
        gbc.weightx = 1.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        utilMethods.add(setStateButton, gbc);
        setStateTextField = new JTextField();
        setStateTextField.setColumns(25);
        setStateTextField.setToolTipText("state to hold");
        gbc = new GridBagConstraints();
        gbc.gridx = 1;
        gbc.gridy = 1;
        gbc.weightx = 4.0;
        gbc.weighty = 1.0;
        gbc.anchor = GridBagConstraints.WEST;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        utilMethods.add(setStateTextField, gbc);
        savePerfMeasuresButton = new JButton();
        savePerfMeasuresButton.setText("savePerfMeasures");
        gbc = new GridBagConstraints();
        gbc.gridx = 2;
        gbc.gridy = 1;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        utilMethods.add(savePerfMeasuresButton, gbc);
        perfFileNameTxtField = new JTextField();
        perfFileNameTxtField.setColumns(25);
        perfFileNameTxtField.setText("");
        gbc = new GridBagConstraints();
        gbc.gridx = 3;
        gbc.gridy = 1;
        gbc.weightx = 4.0;
        gbc.anchor = GridBagConstraints.WEST;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        utilMethods.add(perfFileNameTxtField, gbc);
        suspendButton = new JButton();
        suspendButton.setText("suspendGoal");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        utilMethods.add(suspendButton, gbc);
        suspendGoalField = new JTextField();
        suspendGoalField.setColumns(25);
        gbc = new GridBagConstraints();
        gbc.gridx = 1;
        gbc.gridy = 0;
        gbc.weightx = 4.0;
        gbc.weighty = 1.0;
        gbc.anchor = GridBagConstraints.WEST;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        utilMethods.add(suspendGoalField, gbc);
        resumeButton = new JButton();
        resumeButton.setText("resumeGoal");
        gbc = new GridBagConstraints();
        gbc.gridx = 2;
        gbc.gridy = 0;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        utilMethods.add(resumeButton, gbc);
        resumeGoalField = new JTextField();
        resumeGoalField.setColumns(25);
        gbc = new GridBagConstraints();
        gbc.gridx = 3;
        gbc.gridy = 0;
        gbc.weightx = 4.0;
        gbc.weighty = 1.0;
        gbc.anchor = GridBagConstraints.WEST;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        utilMethods.add(resumeGoalField, gbc);
        randomGoalsPanel = new JPanel();
        randomGoalsPanel.setLayout(new GridBagLayout());
        randomGoalsPanel.setEnabled(false);
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 4;
        gbc.weightx = 1.0;
        gbc.weighty = 0.5;
        gbc.fill = GridBagConstraints.BOTH;
        submitPanel.add(randomGoalsPanel, gbc);
        fetchObjField = new JTextField();
        fetchObjField.setColumns(25);
        fetchObjField.setText("");
        fetchObjField.setToolTipText("Object to fetch");
        gbc = new GridBagConstraints();
        gbc.gridx = 1;
        gbc.gridy = 0;
        gbc.weightx = 3.0;
        gbc.weighty = 1.0;
        gbc.anchor = GridBagConstraints.WEST;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        randomGoalsPanel.add(fetchObjField, gbc);
        fetchButton = new JButton();
        fetchButton.setText("fetch");
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.weightx = 2.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        randomGoalsPanel.add(fetchButton, gbc);
        goToPoseButton = new JButton();
        goToPoseButton.setText("goToPose");
        gbc = new GridBagConstraints();
        gbc.gridx = 2;
        gbc.gridy = 0;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        randomGoalsPanel.add(goToPoseButton, gbc);
        assembleButton = new JButton();
        assembleButton.setText("assemble");
        gbc = new GridBagConstraints();
        gbc.gridx = 3;
        gbc.gridy = 0;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        randomGoalsPanel.add(assembleButton, gbc);
        ap_caddy = new JButton();
        ap_caddy.setText("approach caddy");
        gbc = new GridBagConstraints();
        gbc.gridx = 4;
        gbc.gridy = 0;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        randomGoalsPanel.add(ap_caddy, gbc);
    }

    public JComponent $$$getRootComponent$$$() {
        return gmPanel;
    }

    private void createUIComponents() {
        // TODO: place custom component creation code here
    }
}

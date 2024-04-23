/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.gson;

import java.util.List;

public class PromptInfo {

  public ActionInfo[] actions;
  public PropertyInfo[] properties;
  public QuestionInfo[] activeQuestions;
  //Maybe just types? constants?
  public FactInfo[] facts;
  public StateInfo[] states;
  public GoalInfo[] goals;

  public PromptInfo(List<ActionInfo> actions, List<PropertyInfo> properties, List<QuestionInfo> activeQuestions,
                    List<FactInfo> facts, List<StateInfo> states, List<GoalInfo> goals) {
    this.actions = actions.toArray(new ActionInfo[0]);
    this.properties = properties.toArray(new PropertyInfo[0]);
    this.activeQuestions = activeQuestions.toArray(new QuestionInfo[0]);
    this.facts = facts.toArray(new FactInfo[0]);
    this.states = states.toArray(new StateInfo[0]);
    this.goals = goals.toArray(new GoalInfo[0]);
  }
}

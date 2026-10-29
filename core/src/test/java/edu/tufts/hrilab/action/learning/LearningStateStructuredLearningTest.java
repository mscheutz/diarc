/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.learning;

import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageParser;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageWriter;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class LearningStateStructuredLearningTest {
  private static Predicate signature(String name) {
    return Factory.createPredicate(name, "self:agent", "?object");
  }

  private static Predicate step(String name) {
    return Factory.createPredicate(name, "self:agent", "?object");
  }

  @Test
  public void teachesStructuredIfElseWithTailStep() {
    LearningState learningState = new LearningState(signature("structuredLearned"));
    learningState.beginLearnedIf(Factory.createPredicate("true"));
    learningState.addEventSpec(learningState.getEventSpecs().size(), step("carryWithTwoHands"));
    learningState.beginLearnedElse();
    learningState.addEventSpec(learningState.getEventSpecs().size(), step("carryWithOneHand"));
    learningState.endLearnedIf();
    learningState.addEventSpec(learningState.getEventSpecs().size(), step("placeObject"));

    ActionDBEntry learned = learningState.generateActionDBEntry();
    List<String> commands = learned.getEventSpecs().stream().map(EventSpec::getCommand).toList();
    assertTrue(commands.contains("if"));
    assertTrue(commands.contains("then"));
    assertTrue(commands.contains("else"));
    assertTrue(commands.contains("endif"));
    assertEquals("placeObject", commands.get(commands.size() - 1));
  }

  @Test
  public void rejectsPublishingWhenBlockUnclosed() {
    LearningState learningState = new LearningState(signature("unclosedLearned"));
    learningState.beginLearnedIf(Factory.createPredicate("true"));
    learningState.addEventSpec(learningState.getEventSpecs().size(), step("inside"));
    try {
      learningState.generateActionDBEntry();
      fail("Expected generation to fail when an if block is left open.");
    } catch (IllegalStateException expected) {
      assertTrue(expected.getMessage().contains("Unclosed block"));
    }
  }

  @Test
  public void declaresBlockLocalInsideConditionBranch() {
    LearningState learningState = new LearningState(signature("localLearned"));
    learningState.beginLearnedIf(Factory.createPredicate("true"));
    learningState.declareLearnedLocal("!counter", "java.lang.Integer", "", "1");
    learningState.addEventSpec(learningState.getEventSpecs().size(), step("checkCounter"));
    learningState.endLearnedIf();

    ActionDBEntry learned = learningState.generateActionDBEntry();
    List<String> commands = learned.getEventSpecs().stream().map(EventSpec::getCommand).toList();
    assertTrue(commands.contains("declare"));
    assertTrue(commands.contains("assign"));
  }

  @Test
  public void roundTripsNestedStructuredLearningScript() {
    LearningState learningState = new LearningState(signature("nestedStructuredLearned"));
    learningState.beginLearnedIf(Factory.createPredicate("true"));
    learningState.declareLearnedLocal("!counter", "java.lang.Integer", "", "0");
    learningState.beginLearnedWhile(Factory.createPredicate("true"));
    learningState.addEventSpec(learningState.getEventSpecs().size(), step("tick"));
    learningState.endLearnedWhile();
    learningState.beginLearnedElse();
    learningState.beginLearnedForeach("!item", "?object");
    learningState.addEventSpec(learningState.getEventSpecs().size(), step("processItem"));
    learningState.endLearnedForeach();
    learningState.endLearnedIf();
    learningState.addEventSpec(learningState.getEventSpecs().size(), step("finish"));

    ActionDBEntry learned = learningState.generateActionDBEntry(false);
    String serialized = new ActionScriptLanguageWriter().writeAction(learned);
    ActionDBEntry reparsed = new ActionScriptLanguageParser()
        .parseStrict("<nestedStructuredLearned>", List.of(), serialized)
        .get(0);

    List<String> learnedSignature = learned.getEventSpecs().stream()
        .map(es -> es.getType() + "|" + es.getActor() + "|" + es.getCommand() + "|" + es.getInputArgs() + "|" + es.getReturnArgs())
        .toList();
    List<String> reparsedSignature = reparsed.getEventSpecs().stream()
        .map(es -> es.getType() + "|" + es.getActor() + "|" + es.getCommand() + "|" + es.getInputArgs() + "|" + es.getReturnArgs())
        .toList();
    assertEquals(learnedSignature, reparsedSignature);
  }
}

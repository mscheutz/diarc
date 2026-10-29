/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.learning;

import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.manager.ExecutionManager;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class ActionLearningStructuredEditingTest {
  private static Predicate signature(String name) {
    return Factory.createPredicate(name, "self:agent", "?object");
  }

  private static Predicate step(String name) {
    return Factory.createPredicate(name, "self:agent", "?object");
  }

  private static ActionDBEntry seedConditionalAction(String name) {
    LearningState learningState = new LearningState(signature(name));
    learningState.beginLearnedIf(Factory.createPredicate("true"));
    learningState.addEventSpec(learningState.getEventSpecs().size(), step("thenStep"));
    learningState.beginLearnedElse();
    learningState.addEventSpec(learningState.getEventSpecs().size(), step("elseStep"));
    learningState.endLearnedIf();
    learningState.addEventSpec(learningState.getEventSpecs().size(), step("tail"));
    return learningState.generateActionDBEntry(true);
  }

  private static List<String> commandSequence(String name) {
    ActionDBEntry entry = Database.getActionDB().getAction(name);
    assertNotNull(entry);
    return entry.getEventSpecs().stream().map(EventSpec::getCommand).toList();
  }

  @Test
  public void modifyActionSupportsBranchAndBlockEdits() {
    String name = "structuredEdit_" + System.nanoTime();
    ActionDBEntry seeded = seedConditionalAction(name);
    assertNotNull(seeded);
    ActionLearning learning = new ActionLearning(null, false);
    Predicate knownSignature = seeded.getSignature(true);

    learning.modifyAction(
        knownSignature,
        Factory.createPredicate("insertInBranch",
            Factory.createSymbol("0"),
            Factory.createSymbol("1"),
            step("elseExtra")),
        Factory.createPredicate("none"));

    ActionDBEntry afterInsert = Database.getActionDB().getAction(name);
    assertNotNull(afterInsert);
    List<String> insertCommands = afterInsert.getEventSpecs().stream().map(EventSpec::getCommand).toList();
    assertEquals(List.of("if", "true", "then", "thenStep", "else", "elseStep", "elseExtra", "endif", "tail"), insertCommands);

    learning.modifyAction(
        knownSignature,
        Factory.createPredicate("removeBlock", Factory.createSymbol("0")),
        Factory.createPredicate("none"));

    ActionDBEntry afterRemove = Database.getActionDB().getAction(name);
    assertNotNull(afterRemove);
    List<String> removeCommands = afterRemove.getEventSpecs().stream().map(EventSpec::getCommand).toList();
    assertEquals(List.of("tail"), removeCommands);
  }

  @Test
  public void modifyActionSupportsLocalDeclarationRenameAndReplaceAt() {
    String name = "structuredLocalEdit_" + System.nanoTime();
    ActionDBEntry seeded = seedConditionalAction(name);
    assertNotNull(seeded);
    ActionLearning learning = new ActionLearning(null, false);
    Predicate knownSignature = seeded.getSignature(true);

    learning.modifyAction(
        knownSignature,
        Factory.createPredicate("replaceAt",
            Factory.createSymbol("3"),
            step("thenReplacement")),
        Factory.createPredicate("none"));

    learning.modifyAction(
        knownSignature,
        Factory.createPredicate("addBlockLocal",
            Factory.createSymbol("3"),
            Factory.createSymbol("!counter"),
            Factory.createSymbol("java.lang.Integer"),
            Factory.createSymbol("none"),
            Factory.createSymbol("none"),
            Factory.createSymbol("1")),
        Factory.createPredicate("none"));

    learning.modifyAction(
        knownSignature,
        Factory.createPredicate("renameBlockLocal",
            Factory.createSymbol("3"),
            Factory.createSymbol("!n")),
        Factory.createPredicate("none"));

    ActionDBEntry edited = Database.getActionDB().getAction(name);
    assertNotNull(edited);
    List<EventSpec> specs = edited.getEventSpecs();
    List<String> commands = specs.stream().map(EventSpec::getCommand).toList();
    assertTrue(commands.contains("declare"));
    assertTrue(commands.contains("assign"));
    assertTrue(commands.contains("thenReplacement"));
    EventSpec declare = specs.stream().filter(es -> "declare".equals(es.getCommand())).findFirst().orElseThrow();
    assertEquals("!n", declare.getInputArgs().get(0));
    EventSpec assign = specs.stream().filter(es -> "assign".equals(es.getCommand())).findFirst().orElseThrow();
    assertEquals("!n", assign.getReturnArgs().get(0));
  }

  @Test
  public void modifyActionRejectsInvalidBlockEditWithoutMutatingAction() {
    String name = "structuredInvalidBlockEdit_" + System.nanoTime();
    ActionDBEntry seeded = seedConditionalAction(name);
    assertNotNull(seeded);
    ActionLearning learning = new ActionLearning(null, false);
    Predicate knownSignature = seeded.getSignature(true);
    List<String> before = commandSequence(name);

    try {
      learning.modifyAction(
          knownSignature,
          Factory.createPredicate("insertInBranch",
              Factory.createSymbol("0"),
              Factory.createSymbol("9"),
              step("unreachable")),
          Factory.createPredicate("none"));
      fail("Expected invalid branch edit to throw.");
    } catch (IllegalArgumentException expected) {
      assertTrue(expected.getMessage().contains("branch"));
    }

    assertEquals(before, commandSequence(name));
  }

  @Test
  public void modifyActionRejectsControlMarkerReplaceWithoutMutatingAction() {
    String name = "structuredControlReplace_" + System.nanoTime();
    ActionDBEntry seeded = seedConditionalAction(name);
    assertNotNull(seeded);
    ActionLearning learning = new ActionLearning(null, false);
    Predicate knownSignature = seeded.getSignature(true);
    List<String> before = commandSequence(name);

    try {
      learning.modifyAction(
          knownSignature,
          Factory.createPredicate("replaceAt",
              Factory.createSymbol("0"),
              step("replacement")),
          Factory.createPredicate("none"));
      fail("Expected replace-at control marker to throw.");
    } catch (IllegalArgumentException expected) {
      assertTrue(expected.getMessage().contains("control marker"));
    }

    assertEquals(before, commandSequence(name));
  }

  @Test
  public void modifyActionSupportsCanonicalAndLegacyAddBlockLocalInitializerForms() {
    String canonicalName = "structuredCanonicalLocal_" + System.nanoTime();
    ActionDBEntry canonicalSeeded = seedConditionalAction(canonicalName);
    assertNotNull(canonicalSeeded);
    ActionLearning learning = new ActionLearning(null, false);
    Predicate canonicalSignature = canonicalSeeded.getSignature(true);

    learning.modifyAction(
        canonicalSignature,
        Factory.createPredicate("addBlockLocal",
            Factory.createSymbol("3"),
            Factory.createSymbol("!x"),
            Factory.createSymbol("java.lang.Integer"),
            Factory.createSymbol("none"),
            Factory.createSymbol("7")),
        Factory.createPredicate("none"));

    ActionDBEntry canonicalEdited = Database.getActionDB().getAction(canonicalName);
    assertNotNull(canonicalEdited);
    EventSpec canonicalAssign = canonicalEdited.getEventSpecs().stream()
        .filter(es -> "assign".equals(es.getCommand()) && es.getReturnArgs().contains("!x"))
        .findFirst().orElseThrow();
    assertEquals("7", canonicalAssign.getInputArgs().get(0));

    String legacyName = "structuredLegacyLocal_" + System.nanoTime();
    ActionDBEntry legacySeeded = seedConditionalAction(legacyName);
    assertNotNull(legacySeeded);
    Predicate legacySignature = legacySeeded.getSignature(true);
    learning.modifyAction(
        legacySignature,
        Factory.createPredicate("addBlockLocal",
            Factory.createSymbol("3"),
            Factory.createSymbol("!y"),
            Factory.createSymbol("java.lang.Integer"),
            Factory.createSymbol("none"),
            Factory.createSymbol("none"),
            Factory.createSymbol("8")),
        Factory.createPredicate("none"));

    ActionDBEntry legacyEdited = Database.getActionDB().getAction(legacyName);
    assertNotNull(legacyEdited);
    EventSpec legacyAssign = legacyEdited.getEventSpecs().stream()
        .filter(es -> "assign".equals(es.getCommand()) && es.getReturnArgs().contains("!y"))
        .findFirst().orElseThrow();
    assertEquals("8", legacyAssign.getInputArgs().get(0));
  }

  @Test
  public void modifyActionStructuredOpsRequireNoneLocationAndPreserveActionOnFailure() {
    String name = "structuredLocationGuard_" + System.nanoTime();
    ActionDBEntry seeded = seedConditionalAction(name);
    assertNotNull(seeded);
    ActionLearning learning = new ActionLearning(null, false);
    Predicate signature = seeded.getSignature(true);
    List<String> before = commandSequence(name);

    try {
      learning.modifyAction(
          signature,
          Factory.createPredicate("removeAt", Factory.createSymbol("3")),
          Factory.createPredicate("before", step("thenStep")));
      fail("Expected structured edit with non-none location to throw.");
    } catch (IllegalArgumentException expected) {
      assertTrue(expected.getMessage().contains("requires location predicate 'none'"));
    }

    assertEquals(before, commandSequence(name));
  }

  @Test
  public void modifyActionRejectsInvalidLegacyAddBlockLocalPlaceholder() {
    String name = "structuredLegacyPlaceholder_" + System.nanoTime();
    ActionDBEntry seeded = seedConditionalAction(name);
    assertNotNull(seeded);
    ActionLearning learning = new ActionLearning(null, false);
    Predicate signature = seeded.getSignature(true);
    List<String> before = commandSequence(name);

    try {
      learning.modifyAction(
          signature,
          Factory.createPredicate("addBlockLocal",
              Factory.createSymbol("3"),
              Factory.createSymbol("!bad"),
              Factory.createSymbol("java.lang.Integer"),
              Factory.createSymbol("none"),
              Factory.createSymbol("unexpected"),
              Factory.createSymbol("5")),
          Factory.createPredicate("none"));
      fail("Expected invalid legacy placeholder to throw.");
    } catch (IllegalArgumentException expected) {
      assertTrue(expected.getMessage().contains("legacy placeholder"));
    }

    assertEquals(before, commandSequence(name));
  }

  @Test
  public void modifyActionPersistenceFailureRollsBackToOriginalAction() {
    String name = "structuredModifyRollback_" + System.nanoTime();
    ActionDBEntry seeded = seedConditionalAction(name);
    assertNotNull(seeded);
    ExecutionManager manager = new ExecutionManager();
    manager.setLearnedActionsFile("build");
    ActionLearning learning = new ActionLearning(manager, false);
    Predicate signature = seeded.getSignature(true);
    List<String> before = commandSequence(name);

    try {
      learning.modifyAction(
          signature,
          Factory.createPredicate("replaceAt",
              Factory.createSymbol("3"),
              step("replacementStep")),
          Factory.createPredicate("none"));
      fail("Expected publication to fail when learned actions path is not writable as a file.");
    } catch (IllegalStateException expected) {
      assertTrue(expected.getMessage().contains("Failed to publish modified action candidate"));
    }

    assertEquals("Original action must remain available after failed replacement publication.",
        before, commandSequence(name));
  }
}

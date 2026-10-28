/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.asl;

import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.asl.SourceLocation;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

import static org.junit.Assert.*;

public class AslBlockBuilderTest {
  private static EventSpec action(String name) {
    return new EventSpec.Builder(EventSpec.EventType.ACTION).setCommand(name).build();
  }

  private static EventSpec action(String actor, String name, String... inputArgs) {
    EventSpec.Builder builder = new EventSpec.Builder(EventSpec.EventType.ACTION).setCommand(name);
    if (actor != null) {
      builder.setActor(actor);
    }
    for (String inputArg : inputArgs) {
      builder.addInputArg(inputArg);
    }
    return builder.build();
  }

  private static EventSpec operator(String name, String... inputArgs) {
    EventSpec.Builder builder = new EventSpec.Builder(EventSpec.EventType.OPERATOR).setCommand(name);
    for (String inputArg : inputArgs) {
      builder.addInputArg(inputArg);
    }
    return builder.build();
  }

  private static EventSpec assign(String expression, String target) {
    return new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign")
        .addInputArg(expression).addReturnArg(target).build();
  }

  private static EventSpec trueCondition() {
    return AslControlSpecFactory.control(AslControlSpecFactory.Marker.TRUE);
  }

  @Test
  public void buildsNestedIfElseAndWhile() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition()))
        .add(action("a1"))
        .beginElse()
        .beginWhile(List.of(trueCondition()))
        .add(action("a2"))
        .endWhile()
        .endIf()
        .add(action("a3"))
        .validate();

    List<EventSpec> specs = builder.build();
    assertEquals("if", specs.get(0).getCommand());
    assertTrue(specs.stream().anyMatch(s -> "then".equals(s.getCommand())));
    assertTrue(specs.stream().anyMatch(s -> "else".equals(s.getCommand())));
    assertTrue(specs.stream().anyMatch(s -> "while".equals(s.getCommand())));
    assertEquals("endif", specs.get(specs.size() - 2).getCommand());
    assertEquals("a3", specs.get(specs.size() - 1).getCommand());
  }

  @Test
  public void buildsForAndForeachWithBlockMarkers() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginFor("!i", "0", "lt", "10", "++")
        .add(action("step"))
        .endFor()
        .beginForeach("!item", "?items")
        .add(action("consume"))
        .endForeach()
        .validate();

    List<EventSpec> specs = builder.build();
    assertEquals("for", specs.get(0).getCommand());
    assertEquals("block", specs.get(1).getCommand());
    assertEquals("endblock", specs.get(3).getCommand());
    assertEquals("endfor", specs.get(4).getCommand());
    assertEquals("foreach", specs.get(5).getCommand());
    assertEquals("block", specs.get(6).getCommand());
    assertEquals("endblock", specs.get(8).getCommand());
    assertEquals("endfor", specs.get(9).getCommand());
  }

  @Test
  public void validatesAndQueriesStructure() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition()))
        .add(action("left"))
        .beginElseIf(List.of(trueCondition()))
        .add(action("middle"))
        .beginElse()
        .add(action("right"))
        .endIf();

    AslBlockStructure.Snapshot snapshot = builder.snapshot();
    assertEquals(1, snapshot.blocks().size());
    AslBlockStructure.BlockDescriptor ifBlock = snapshot.blocks().get(0);
    assertEquals(AslBlockStructure.BlockType.IF, ifBlock.type());
    assertEquals(3, ifBlock.branches().size());
    assertTrue(snapshot.findMatchingEnd(ifBlock.startIndex()).isPresent());
    assertEquals(ifBlock.endIndex(), snapshot.findMatchingEnd(ifBlock.startIndex()).getAsInt());
    assertTrue(snapshot.findEnclosingBlock(ifBlock.startIndex() + 2).isPresent());
  }

  @Test(expected = IllegalStateException.class)
  public void rejectsElseWithoutIf() {
    new AslBlockBuilder().beginElse();
  }

  @Test(expected = IllegalStateException.class)
  public void rejectsElseifAfterElse() {
    new AslBlockBuilder()
        .beginIf(List.of(trueCondition()))
        .beginElse()
        .beginElseIf(List.of(trueCondition()));
  }

  @Test
  public void transactionalEditsDoNotCorruptOnFailure() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition())).add(action("inside")).endIf();
    List<EventSpec> before = builder.build();

    try {
      builder.removeRange(0, 1); // removes leading if marker and makes structure invalid
      fail("Expected validation failure");
    } catch (IllegalArgumentException expected) {
      // expected
    }

    assertEquals(before, builder.specs());
  }

  @Test
  public void removeAndReplaceCompleteBlock() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition())).add(action("a")).endIf().add(action("tail"));
    int ifIndex = 0;
    builder.replaceCompleteBlock(ifIndex, List.of(action("replacement")));
    assertEquals("replacement", builder.specs().get(0).getCommand());
    assertEquals("tail", builder.specs().get(1).getCommand());
  }

  @Test
  public void appendsAndInsertsUsingBlockApis() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition())).add(action("a")).endIf().add(action("tail"));

    builder.appendToBlockBody(0, action("b"));
    builder.insertIntoBlockBody(0, 0, action("head"));

    List<String> commands = builder.specs().stream().map(EventSpec::getCommand).toList();
    assertTrue(commands.contains("head"));
    assertTrue(commands.contains("a"));
    assertTrue(commands.contains("b"));
  }

  @Test
  public void appendsToIfBranchByIndex() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition()))
        .add(action("thenAction"))
        .beginElse()
        .add(action("elseAction"))
        .endIf();

    builder.appendToBranch(0, 1, action("elseTail"));
    List<EventSpec> specs = builder.specs();
    assertTrue(specs.stream().anyMatch(es -> "elseTail".equals(es.getCommand())));
  }

  @Test(expected = IllegalStateException.class)
  public void staleBlockHandleIsRejected() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition())).add(action("a")).endIf();
    AslBlockBuilder.BlockHandle handle = builder.blockHandle(0);
    builder.appendToBlockBody(0, action("b"));
    builder.appendToBlockBody(handle, action("c"));
  }

  @Test(expected = IllegalArgumentException.class)
  public void removeActionAtRejectsControlMarkers() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition())).add(action("a")).endIf();
    builder.removeActionAt(0);
  }

  @Test
  public void editsBlockLocalDeclarations() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition()))
        .addBlockLocalDeclaration(3, "!x", "java.lang.Integer", "", "1")
        .add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("!x + 1").addReturnArg("!x").build())
        .endIf();

    int declareIndex = -1;
    for (int i = 0; i < builder.specs().size(); i++) {
      if ("declare".equals(builder.specs().get(i).getCommand())) {
        declareIndex = i;
        break;
      }
    }
    assertTrue(declareIndex >= 0);

    builder.renameBlockLocalVariable(declareIndex, "!counter");
    assertTrue(builder.specs().stream().anyMatch(es -> es.getReturnArgs().contains("!counter")));
  }

  @Test
  public void renamingDeclarationRespectsNestedShadowing() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition()))
        .addBlockLocalDeclaration(3, "!x", "java.lang.Integer", "", "1")
        .add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("!x + 1").addReturnArg("!x").build())
        .beginIf(List.of(trueCondition()))
        .addBlockLocalDeclaration(builder.specs().size(), "!x", "java.lang.Integer", "", "2")
        .add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("!x + 1").addReturnArg("!x").build())
        .endIf()
        .add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("!x + 10").addReturnArg("!x").build())
        .endIf();

    int outerDeclare = -1;
    int seen = 0;
    for (int i = 0; i < builder.specs().size(); i++) {
      EventSpec es = builder.specs().get(i);
      if ("declare".equals(es.getCommand()) && "!x".equals(es.getInputArgs().get(0))) {
        if (seen == 0) {
          outerDeclare = i;
        }
        seen++;
      }
    }
    assertEquals(2, seen);
    assertTrue(outerDeclare >= 0);

    builder.renameBlockLocalVariable(outerDeclare, "!outer");

    long outerRefs = builder.specs().stream()
        .filter(es -> es.getInputArgs().stream().anyMatch(a -> a != null && a.contains("!outer"))
            || es.getReturnArgs().contains("!outer")
            || ("declare".equals(es.getCommand()) && es.getInputArgs().size() > 0 && "!outer".equals(es.getInputArgs().get(0))))
        .count();
    long innerRefs = builder.specs().stream()
        .filter(es -> es.getInputArgs().stream().anyMatch(a -> a != null && a.contains("!x"))
            || es.getReturnArgs().contains("!x")
            || ("declare".equals(es.getCommand()) && es.getInputArgs().size() > 0 && "!x".equals(es.getInputArgs().get(0))))
        .count();
    assertTrue("expected renamed outer references", outerRefs > 0);
    assertTrue("expected inner shadow references preserved", innerRefs > 0);
  }

  @Test
  public void rejectRemovingReferencedDeclaration() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition()))
        .addBlockLocalDeclaration(3, "!x", "java.lang.Integer", "", "1")
        .add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("!x + 1").addReturnArg("!x").build())
        .endIf();
    int declareIndex = -1;
    for (int i = 0; i < builder.specs().size(); i++) {
      if ("declare".equals(builder.specs().get(i).getCommand())) {
        declareIndex = i;
        break;
      }
    }
    assertTrue(declareIndex >= 0);
    try {
      builder.removeBlockLocalDeclaration(declareIndex);
      fail("Expected referenced declaration removal to be rejected.");
    } catch (IllegalArgumentException expected) {
      assertTrue(expected.getMessage().contains("!x"));
      assertTrue(expected.getMessage().contains("indices"));
    }
  }

  @Test
  public void removesUnreferencedDeclaration() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition()))
        .addBlockLocalDeclaration(3, "!x", "java.lang.Integer", "", null)
        .add(action("afterDeclare"))
        .endIf();

    int declareIndex = -1;
    for (int i = 0; i < builder.specs().size(); i++) {
      if ("declare".equals(builder.specs().get(i).getCommand())) {
        declareIndex = i;
        break;
      }
    }
    assertTrue(declareIndex >= 0);
    builder.removeBlockLocalDeclaration(declareIndex);
    assertFalse(builder.specs().stream().anyMatch(es -> "declare".equals(es.getCommand())));
  }

  @Test
  public void removesOuterDeclarationWhenOnlyShadowedInnerUsesExist() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition()))
        .addBlockLocalDeclaration(3, "!x", "java.lang.Integer", "", null)
        .beginIf(List.of(trueCondition()))
        .addBlockLocalDeclaration(builder.specs().size(), "!x", "java.lang.Integer", "", "2")
        .add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("!x + 1").addReturnArg("!x").build())
        .endIf()
        .endIf();

    int outerDeclareIndex = -1;
    int seen = 0;
    for (int i = 0; i < builder.specs().size(); i++) {
      EventSpec es = builder.specs().get(i);
      if ("declare".equals(es.getCommand()) && !es.getInputArgs().isEmpty() && "!x".equals(es.getInputArgs().get(0))) {
        if (seen == 0) {
          outerDeclareIndex = i;
        }
        seen++;
      }
    }
    assertEquals(2, seen);
    assertTrue(outerDeclareIndex >= 0);

    builder.removeBlockLocalDeclaration(outerDeclareIndex);
    long remainingDeclarations = builder.specs().stream()
        .filter(es -> "declare".equals(es.getCommand()) && !es.getInputArgs().isEmpty() && "!x".equals(es.getInputArgs().get(0)))
        .count();
    assertEquals(1, remainingDeclarations);
  }

  @Test
  public void rejectsInsertingStatementOutsideDeclarationScope() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginIf(List.of(trueCondition()))
        .addBlockLocalDeclaration(3, "!temp", "java.lang.Integer", "", "1")
        .add(assign("!temp + 1", "!temp"))
        .endIf()
        .add(action("tail"));

    List<EventSpec> before = builder.build();

    int assignIndex = -1;
    for (int i = 0; i < builder.specs().size(); i++) {
      EventSpec es = builder.specs().get(i);
      if ("assign".equals(es.getCommand()) && !es.getReturnArgs().isEmpty() && "!temp".equals(es.getReturnArgs().get(0))) {
        assignIndex = i;
        break;
      }
    }
    assertTrue(assignIndex >= 0);
    EventSpec moved = builder.specs().get(assignIndex);

    try {
      builder.insert(builder.specs().size(), moved);
      fail("Expected out-of-scope insertion to be rejected.");
    } catch (IllegalArgumentException expected) {
      assertTrue(expected.getMessage().contains("lexical scope"));
    }

    assertEquals(before, builder.specs());
  }

  @Test
  public void programmaticBuilderMatchesHandwrittenForeachScopingScript() {
    String source = """
        () = processAll(java.util.List ?items) {
          foreach (!item : ?items) {
            java.lang.String !status = "pending";
            if (true) {
              java.lang.String !status = "valid";
              act:process(!item);
            } else {
              java.lang.String !reason = "skipped";
              op:log("info", !reason);
            }
            op:log("info", !status);
          }
        }
        """;

    ActionDBEntry parsed = new ActionScriptLanguageParser()
        .parseStrict("process-all-test.asl", null, source)
        .get(0);

    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginForeach("!item", "?items")
        .addBlockLocalDeclaration(builder.specs().size(), "!status", "java.lang.String", "", "\"pending\"")
        .beginIf(List.of(trueCondition()))
        .addBlockLocalDeclaration(builder.specs().size(), "!status", "java.lang.String", "", "\"valid\"")
        .add(action(null, "process", "!item"))
        .beginElse()
        .addBlockLocalDeclaration(builder.specs().size(), "!reason", "java.lang.String", "", "\"skipped\"")
        .add(operator("log", "\"info\"", "!reason"))
        .endIf()
        .add(operator("log", "\"info\"", "!status"))
        .endForeach();

    List<String> parsedSignature = toSignature(parsed.getEventSpecs());
    List<String> builtSignature = toSignature(builder.build());
    assertEquals(parsedSignature, builtSignature);
  }

  @Test(expected = AslScopeValidator.ValidationException.class)
  public void rejectsUseOutsideLexicalScope() {
    AslBlockBuilder builder = new AslBlockBuilder();
    EventSpec declare = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare")
        .addInputArg("!x").addInputArg("java.lang.Integer").build();
    EventSpec assign = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign")
        .addInputArg("1").addReturnArg("!x").build();
    builder.beginIf(List.of(trueCondition()))
        .add(declare)
        .add(assign)
        .endIf()
        .add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("2").addReturnArg("!x").build())
        .build();
  }

  private static List<String> toSignature(List<EventSpec> specs) {
    List<String> signature = new ArrayList<>();
    for (EventSpec spec : specs) {
      signature.add(spec.getType() + "|" + spec.getActor() + "|" + spec.getCommand() + "|" + spec.getInputArgs() + "|" + spec.getReturnArgs());
    }
    return signature;
  }

  @Test
  public void scopeValidatorRejectsUndeclaredAndOutOfLifetimeReferences() {
    List<EventSpec> valid = List.of(
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare").addInputArg("!x").addInputArg("java.lang.Integer").build(),
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("!x + 1").addReturnArg("!x").build()
    );
    AslScopeValidator.validate(valid);

    List<EventSpec> undeclared = List.of(
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("!missing + 1").addReturnArg("!x").build()
    );
    expectThrows(AslScopeValidator.ValidationException.class, () -> AslScopeValidator.validate(undeclared));

    List<EventSpec> laterDeclaration = List.of(
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("!x + 1").addReturnArg("!x").build(),
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare").addInputArg("!x").addInputArg("java.lang.Integer").build()
    );
    expectThrows(AslScopeValidator.ValidationException.class, () -> AslScopeValidator.validate(laterDeclaration));

    List<EventSpec> branchLeak = List.of(
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.IF),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.TRUE),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.THEN),
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare").addInputArg("!tmp").addInputArg("java.lang.Integer").build(),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ELSE),
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("!tmp + 1").addReturnArg("!tmp").build(),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDIF)
    );
    expectThrows(AslScopeValidator.ValidationException.class, () -> AslScopeValidator.validate(branchLeak));
  }

  @Test
  public void scopeValidatorHandlesShadowingAndMalformedExpressions() {
    List<EventSpec> shadowed = List.of(
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.IF),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.TRUE),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.THEN),
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare").addInputArg("!v").addInputArg("java.lang.Integer").build(),
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("1").addReturnArg("!v").build(),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.IF),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.TRUE),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.THEN),
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare").addInputArg("!v").addInputArg("java.lang.Integer").build(),
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("!v + 1").addReturnArg("!v").build(),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDIF),
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("!v + 1").addReturnArg("!v").build(),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDIF)
    );
    AslScopeValidator.validate(shadowed);

    List<EventSpec> malformedExpression = List.of(
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare").addInputArg("!x").addInputArg("java.lang.Integer").build(),
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign").addInputArg("(!x +").addReturnArg("!x").build()
    );
    AslScopeValidator.ValidationException ex =
        expectThrows(AslScopeValidator.ValidationException.class, () -> AslScopeValidator.validate(malformedExpression));
    assertTrue(ex.getMessage().contains("malformed expression"));
  }

  @Test
  public void renameDoesNotModifyStringLiteralsAndPreservesSourceLocation() {
    SourceLocation location = new SourceLocation("test.asl", 12, 3, "origin-node");
    EventSpec declare = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare")
        .addInputArg("!item").addInputArg("java.lang.String").setSourceLocation(location).build();
    EventSpec similarDeclaration = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare")
        .addInputArg("!item2").addInputArg("java.lang.String").setSourceLocation(location).build();
    EventSpec assign = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign")
        .addInputArg("!item").addReturnArg("!item").setSourceLocation(location).build();
    EventSpec process = new EventSpec.Builder(EventSpec.EventType.ACTION).setActor("worker").setCommand("process")
        .addInputArg("wrapper(!item)").addInputArg("!item2")
        .addReturnArg("!item")
        .setSourceLocation(location).build();
    EventSpec log = new EventSpec.Builder(EventSpec.EventType.OPERATOR).setCommand("log")
        .addInputArg("\"Processing !item\"").addInputArg("!item").setSourceLocation(location).build();

    AslBlockBuilder builder = new AslBlockBuilder(List.of(
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.BLOCK),
        declare, similarDeclaration, assign, process, log,
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDBLOCK)
    ));
    builder.renameBlockLocalVariable(1, "!object");
    List<EventSpec> renamed = builder.build();

    assertEquals(location, renamed.get(1).getSourceLocation());
    assertEquals("!object", renamed.get(1).getInputArgs().get(0));
    EventSpec renamedProcess = renamed.get(4);
    assertEquals("worker", renamedProcess.getActor());
    assertEquals("process", renamedProcess.getCommand());
    assertEquals("wrapper(!object)", renamedProcess.getInputArgs().get(0));
    assertEquals("!item2", renamedProcess.getInputArgs().get(1));
    assertEquals(List.of("!object"), renamedProcess.getReturnArgs());
    assertEquals(location, renamedProcess.getSourceLocation());
    EventSpec renamedLog = renamed.get(5);
    assertEquals("\"Processing !item\"", renamedLog.getInputArgs().get(0));
    assertEquals("!object", renamedLog.getInputArgs().get(1));
    assertEquals(location, renamedLog.getSourceLocation());
  }

  @Test
  public void failedRenameIsTransactionalForUnsafeArgumentsAndCollisions() {
    EventSpec declare = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare")
        .addInputArg("!item").addInputArg("java.lang.String").build();
    EventSpec malformed = new EventSpec.Builder(EventSpec.EventType.ACTION).setCommand("process")
        .addInputArg("\"unterminated !item").build();
    AslBlockBuilder unsafe = new AslBlockBuilder(List.of(
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.BLOCK),
        declare, malformed,
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDBLOCK)
    ));
    List<EventSpec> beforeUnsafeRename = unsafe.specs();
    expectThrows(IllegalArgumentException.class, () -> unsafe.renameBlockLocalVariable(1, "!object"));
    assertEquals(beforeUnsafeRename, unsafe.specs());

    AslBlockBuilder collision = new AslBlockBuilder();
    collision.beginBlock()
        .addBlockLocalDeclaration(1, "!item", "java.lang.String", "", null)
        .beginBlock()
        .addBlockLocalDeclaration(collision.specs().size(), "!object", "java.lang.String", "", null)
        .endBlock()
        .endBlock();
    List<EventSpec> beforeCollision = collision.specs();
    expectThrows(IllegalArgumentException.class, () -> collision.renameBlockLocalVariable(1, "!object"));
    assertEquals(beforeCollision, collision.specs());
  }

  @Test
  public void forValidationRequiresInitializationForNewCounterAndChecksTypes() {
    List<EventSpec> uninitialized = List.of(
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("for")
            .addInputArg("!i").addInputArg("lt").addInputArg("3").addInputArg("++").build(),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.BLOCK),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDBLOCK),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDFOR)
    );
    expectThrows(AslScopeValidator.ValidationException.class, () -> AslScopeValidator.validate(uninitialized));

    List<EventSpec> initialized = List.of(
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("for")
            .addInputArg("!i").addInputArg("0").addInputArg("lt").addInputArg("3").addInputArg("++").build(),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.BLOCK),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDBLOCK),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDFOR)
    );
    AslScopeValidator.validate(initialized);

    List<EventSpec> reused = List.of(
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("for")
            .addInputArg("!i").addInputArg("lt").addInputArg("3").addInputArg("++").build(),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.BLOCK),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDBLOCK),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDFOR)
    );
    AslScopeValidator.validate(reused, List.of("!i"));

    List<EventSpec> wrongType = List.of(
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare")
            .addInputArg("!i").addInputArg("java.lang.String").build(),
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("for")
            .addInputArg("!i").addInputArg("lt").addInputArg("3").addInputArg("++").build(),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.BLOCK),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDBLOCK),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDFOR)
    );
    expectThrows(AslScopeValidator.ValidationException.class, () -> AslScopeValidator.validate(wrongType));
  }

  @Test
  public void builderSupportsPredeclaredActionLocalsDuringValidation() {
    EventSpec useLocal = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign")
        .addInputArg("!actionLocal + 1").addReturnArg("!actionLocal").build();
    expectThrows(AslScopeValidator.ValidationException.class,
        () -> new AslBlockBuilder(List.of(useLocal)).build());

    AslBlockBuilder builder = new AslBlockBuilder(List.of(useLocal), List.of("!actionLocal"));
    builder.build();
  }

  @Test
  public void rejectsTryCatchFinallyOrderingViolations() {
    AslBlockBuilder builder = new AslBlockBuilder();
    builder.beginTry().beginFinally();
    expectThrows(IllegalStateException.class, () -> builder.beginCatch(List.of("FAIL", "!reason")));

    AslBlockBuilder builder2 = new AslBlockBuilder();
    builder2.beginTry().beginFinally();
    expectThrows(IllegalStateException.class, builder2::beginFinally);
  }

  @Test
  public void validatesGuardsAndDeclarationShape() {
    List<EventSpec> missingIfGuard = List.of(
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.IF),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.THEN),
        AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDIF)
    );
    expectThrows(AslScopeValidator.ValidationException.class, () -> AslScopeValidator.validate(missingIfGuard));

    List<EventSpec> badDeclaration = List.of(
        new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare")
            .addInputArg("!x").addInputArg("not.a.RealType").build()
    );
    expectThrows(AslScopeValidator.ValidationException.class, () -> AslScopeValidator.validate(badDeclaration));
  }

  private static <T extends Throwable> T expectThrows(Class<T> type, Runnable operation) {
    try {
      operation.run();
    } catch (Throwable t) {
      if (type.isInstance(t)) {
        return type.cast(t);
      }
      throw new AssertionError("Expected " + type.getSimpleName() + " but got " + t.getClass().getSimpleName(), t);
    }
    throw new AssertionError("Expected " + type.getSimpleName() + " but nothing was thrown");
  }
}

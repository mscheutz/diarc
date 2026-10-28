/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
* Test for ASL Writer
* @author Evan Krause
*/
package edu.tufts.hrilab.action.asl;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.recovery.PostRecoveryExecutionType;
import edu.tufts.hrilab.action.recovery.RecoveryType;
import edu.tufts.hrilab.fol.Factory;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.junit.Test;

import java.io.File;
import java.util.List;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class ActionASLTest {

    private static final String TEST_DB = "/config/edu/tufts/hrilab/action/asl/roundTrip.asl";

    private ActionDBEntry parseAction(String source) {
        List<ActionDBEntry> actions = new ActionScriptLanguageParser().parseFromString(TEST_DB, null, source);
        assertNotNull("Parsing failed: " + source, actions);
        assertEquals(1, actions.size());
        return actions.get(0);
    }

    private ActionDBEntry assertRoundTrip(String source) {
        ActionDBEntry original = parseAction(source);
        String written = new ActionScriptLanguageWriter().writeAction(original);

        ASLParser syntaxParser = new ASLParser(new CommonTokenStream(new ASLLexer(CharStreams.fromString(written))));
        syntaxParser.actionScript();
        assertEquals("Writer produced invalid ASL: " + written, 0, syntaxParser.getNumberOfSyntaxErrors());

        ActionDBEntry restored = parseAction(written);
        assertEquals(original, restored);
        assertEquals(original.getOnCancelEvent(), restored.getOnCancelEvent());
        assertEquals(original.getOnSuspendEvent(), restored.getOnSuspendEvent());
        assertEquals(original.getOnResumeEvent(), restored.getOnResumeEvent());
        for (int i = 0; i < original.getRoles().size(); ++i) {
            assertEquals(original.getRoles().get(i).getSemanticType(), restored.getRoles().get(i).getSemanticType());
        }
        return restored;
    }

    @Test
    public void testRejectsLexicalErrors() {
        ActionScriptLanguageParser parser = new ActionScriptLanguageParser();
        assertNull(parser.parseFromString(TEST_DB, null, "() = invalid() { @ act:move(); }"));
        assertNull(parser.parseFromString(TEST_DB, null, "() = invalid() {} @"));
    }

    @Test
    public void testRejectsRecoveredSyntaxErrors() {
        ActionScriptLanguageParser parser = new ActionScriptLanguageParser();
        assertNull(parser.parseFromString(TEST_DB, null, "() = valid() {} () = invalid() { act:move() }"));
        assertNull(parser.parseFromString(TEST_DB, null, "() = invalid() { act:move();"));
        assertNull(parser.parseFromString(TEST_DB, null, "() = invalid() {} trailing"));
        assertNotNull(parser.parseFromString(TEST_DB, null, "() = valid() {}"));
    }

    @Test
    public void testRejectsUnknownDeclarationType() {
        ActionScriptLanguageParser parser = new ActionScriptLanguageParser();
        assertNull(parser.parseFromString(TEST_DB, null, "() = invalid() { not.a.Type !x; }"));
        try {
            parser.parseStrict(TEST_DB, null, "() = invalid() { not.a.Type !x; }");
            org.junit.Assert.fail("Expected parseStrict to reject unknown declaration type");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("Class not found for type"));
        }
    }

    @Test
    public void testFileParserRejectsErrors() {
        ActionScriptLanguageParser parser = new ActionScriptLanguageParser();
        for (String filename : List.of("invalid-lexical.asl", "invalid-syntax.asl")) {
            String resource = "/config/edu/tufts/hrilab/action/asl/" + filename;
            assertNotNull(getClass().getResource(resource));
            assertNull(parser.parseFromFile(resource, null));
        }
    }

    @Test
    public void testExistingScriptsStillParse() {
        ActionScriptLanguageParser parser = new ActionScriptLanguageParser();
        for (String filename : List.of("async_tests.asl", "multiple-returns-test.asl", "this_tests.asl",
                "tsc_tests.asl", "for-loop.asl", "for-each.asl", "recovery_tests.asl")) {
            assertNotNull(filename, parser.parseFromFile("/config/edu/tufts/hrilab/action/asl/" + filename, null));
        }
    }

    @Test
    public void testNegatedPredicatesRoundTrip() {
        assertRoundTrip("""
                () = negated() {
                    conditions: {
                        pre: ~ready();
                        or: { pre: ~first(); pre obs: second(); }
                        overall obs: ~blocked(?actor);
                        obligation infer: ~forbidden(?actor);
                    }
                    effects: {
                        always: ~before();
                        success obs: ~after();
                        failure infer: ~failed();
                        nonperf: ~performed();
                    }
                    observes: ~visible(?actor);
                }
                """);
    }

    @Test
    public void testFunctionalNegationRemainsCompatibleWithCanonicalWriter() {
        assertRoundTrip("""
                () = functionalNegation() {
                    conditions: { pre: not(ready()); }
                    effects: { success: not(done()); }
                    observes: not(visible(?actor));
                    recovery: { goals: {not(done())} }
                }
                """);
    }

    @Test
    public void testRecoveryRoundTrip() {
        ActionDBEntry restored = assertRoundTrip("""
                () = recover() {
                    recovery: {
                        actionRecovery: {
                            catchingActions: {move(?actor), ~blocked(?actor)}
                            postRecovery: retry
                        }
                        goals: {arrived(?actor), ~blocked(?actor)}
                        failedActions: {move(?actor)}
                        failureReasons: {obstacle(?actor), ~clear(?actor)}
                        actionStatuses: {FAIL, FAIL_PRECONDITIONS}
                        maximumAttempts: 3
                    }
                    recovery: {
                        actionRecovery: { postRecovery: continue }
                        excludedGoals: {unrelated(?actor)}
                        excludedFailedActions: {waitFor(?actor)}
                        excludedFailureReasons: {~recoverable(?actor)}
                        maximumAttempts: 2
                    }
                    recovery: {
                        goals: {done(?actor)}
                    }
                    recovery: {}
                }
                """);
        assertEquals(4, restored.getPolicyConstraints().size());
        assertEquals(RecoveryType.ACTION, restored.getPolicyConstraints().get(0).recoveryType);
        assertEquals(PostRecoveryExecutionType.RETRY, restored.getPolicyConstraints().get(0).postRecoveryExecutionType);
        assertEquals(PostRecoveryExecutionType.CONTINUE, restored.getPolicyConstraints().get(1).postRecoveryExecutionType);
        assertEquals(RecoveryType.GOAL, restored.getPolicyConstraints().get(2).recoveryType);
        assertEquals(3, restored.getPolicyConstraints().get(0).maximumAttempts);
        assertTrue(restored.getPolicyConstraints().get(0).goals.contains(
                Factory.createPredicate("blocked", "?actor").toNegatedForm()));
        assertTrue(restored.getPolicyConstraints().get(0).catchingActions.contains(
                Factory.createPredicate("blocked", "?actor").toNegatedForm()));
        assertTrue(restored.getPolicyConstraints().get(1).excludedFailureReasons.contains(
                Factory.createPredicate("recoverable", "?actor").toNegatedForm()));
    }

    @Test
    public void testAllSpecTypesAndNoInterruptRoundTrip() {
        assertRoundTrip("""
                () = specTypes() {
                    java.lang.String !command;
                    act:move();
                    op:log("info", "test");
                    obs:visible(?actor);
                    goal:ready(?actor);
                    tsc:notify(?actor);
                    !command = this:getCommand();
                    noInterrupt {
                        tsc:begin();
                        noInterrupt {
                            if (this:isAction() && (tsc:ready() || ~tsc:blocked())) {
                                this:getCommand();
                            } elseif (tsc:available()) {
                                act:move();
                            } else {
                                tsc:waitFor();
                            }
                        }
                        while (~this:isDone()) { tsc:step(); }
                    }
                }
                """);
    }

    @Test
    public void testAsyncAndJoinRoundTrip() {
        ActionDBEntry restored = assertRoundTrip("""
                () = asynchronous() {
                    java.lang.Long !id;
                    edu.tufts.hrilab.action.ActionStatus !status;
                    !id = async {
                        async { tsc:work(); }
                        noInterrupt { act:move(); }
                    }
                    !status = join(!id, 1000);
                    join(!id);
                }
                """);
        assertEquals(List.of("!id"), restored.getEventSpecs().get(0).getReturnArgs());
        EventSpec join = restored.getEventSpecs().stream().filter(e -> e.getCommand().equals("join")).findFirst().get();
        assertEquals(List.of("!status"), join.getReturnArgs());
        assertEquals(List.of("!id", "1000"), join.getInputArgs());
    }

    @Test
    public void testInterruptCallsRoundTrip() {
        assertRoundTrip("""
                () = interruptible() {
                    onInterrupt: {
                        cancel: robot.act:stop();
                        suspend: ?actor.tsc:pause();
                        resume: robot.goal:ready();
                    }
                }
                """);
    }

    @Test
    public void testResumeResetRoundTrip() {
        ActionDBEntry restored = assertRoundTrip("""
                () = restartable() {
                    onInterrupt: { resume: reset; }
                }
                """);
        assertEquals(EventSpec.EventType.CONTROL, restored.getOnResumeEvent().getType());
        assertEquals("reset", restored.getOnResumeEvent().getCommand());
    }

    @Test
    public void testMultipleReturnsRoundTrip() {
        ActionDBEntry restored = assertRoundTrip("""
                () = multipleReturns() {
                    int !first;
                    int !second;
                    (!first, !second) = robot.tsc:values();
                    if ((!first, !second) = act:checkValues()) {
                        (!first, !second) = op:values();
                    }
                }
                """);
        assertEquals(List.of("!first", "!second"), restored.getEventSpecs().get(0).getReturnArgs());
    }

    @Test
    public void testRejectsMismatchedLoopVariables() {
        ActionScriptLanguageParser parser = new ActionScriptLanguageParser();
        assertNull(parser.parseFromString(TEST_DB, null, "() = loop() { for (!i=0; !j lt 10; !i++) {} }"));
        assertNull(parser.parseFromString(TEST_DB, null, "() = loop() { for (!i=0; !i lt 10; !j++) {} }"));
        assertNull(parser.parseFromString(TEST_DB, null, "() = loop(int ?i) { for (?i; ?j lt 10; ?i++) {} }"));
    }

    @Test
    public void testMatchingLoopVariablesRoundTrip() {
        assertRoundTrip("""
                () = loops(int ?limit, int ?step) {
                    int !index = 0;
                    for (!index; !index lt ?limit; !index += ?step) { tsc:step(!index); }
                    for (!i=0; !i lt 10; !i++) {
                        for (!j=5; !j geq 0; !j--) { act:step(!i, !j); }
                    }
                }
                """);
    }

    @Test
    public void testBlockLocalDeclarationsRoundTrip() {
        ActionDBEntry restored = assertRoundTrip("""
                () = scopedLocals() {
                    int !outer = 1;
                    if (true) {
                        int !inner = 2;
                        !outer = !outer + !inner;
                    } else {
                        int !alt = 3;
                        !outer = !outer + !alt;
                    }
                }
                """);

        assertTrue(restored.getEventSpecs().stream().anyMatch(es -> "declare".equals(es.getCommand())));
        assertTrue(restored.getRoles().stream().noneMatch(role -> "!inner".equals(role.getName())));
        assertTrue(restored.getRoles().stream().noneMatch(role -> "!alt".equals(role.getName())));
    }

    @Test
    public void testStandaloneLexicalBlocksRoundTrip() {
        ActionDBEntry restored = assertRoundTrip("""
                () = lexicalBlocks() {
                    int !outer = 1;
                    block {
                        int !outer = 2;
                        block {
                            int !inner = !outer + 1;
                            op:log("info", !inner);
                        }
                        op:log("info", !outer);
                    }
                    op:log("info", !outer);
                }
                """);
        assertTrue(restored.getEventSpecs().stream().anyMatch(es -> "block".equals(es.getCommand())));
        assertTrue(restored.getEventSpecs().stream().anyMatch(es -> "endblock".equals(es.getCommand())));
    }

    @Test
    public void testStandaloneBlockInConditionalAndLoopRoundTrip() {
        assertRoundTrip("""
                () = nestedStandaloneBlocks(java.util.List ?items) {
                    if (true) {
                        block {
                            int !a = 1;
                            op:log("info", !a);
                        }
                    } else {
                        op:log("info", "noop");
                    }
                    foreach (!item : ?items) {
                        block {
                            java.lang.String !status = "ok";
                            op:log("info", !status);
                        }
                    }
                    while (true) {
                        block { op:log("info", "never"); }
                    }
                }
                """);
    }

    @Test
    public void testRejectsOutOfScopeBlockLocalReference() {
        ActionScriptLanguageParser parser = new ActionScriptLanguageParser();
        assertNull(parser.parseFromString(TEST_DB, null, """
                () = invalidScope() {
                    if (true) { int !tmp = 1; }
                    !tmp = 2;
                }
                """));
    }

    @Test
    public void testSemanticTypesRoundTrip() {
        ActionDBEntry restored = assertRoundTrip("""
                (edu.tufts.hrilab.fol.Symbol ?result:physical) = typed(
                        edu.tufts.hrilab.fol.Symbol ?actor:robot,
                        edu.tufts.hrilab.fol.Symbol ?object:physical) {
                    edu.tufts.hrilab.fol.Symbol !place:location;
                    conditions: { pre: available(?object); }
                    effects: { success: found(?result); }
                }
                """);
        assertEquals("robot", restored.getRole("?actor").getSemanticType());
        assertEquals("physical", restored.getRole("?object").getSemanticType());
        assertEquals("physical", restored.getRole("?result").getSemanticType());
        assertEquals("location", restored.getRole("!place").getSemanticType());
    }

    /**
     * TODO: EAK: the ASL writer and reader should be separated from the Database entirely so that
     *       there's no weird dependency on it
     */
    @Test
    public void testWriter() {
        // Build an Action Script.
        ActionDBEntry.Builder subBuilder = new ActionDBEntry.Builder("action1");
        subBuilder.setDescription("description");
        subBuilder.addRole(new ActionBinding.Builder("?var1", int.class).build());
        subBuilder.addRole(new ActionBinding.Builder("?var2", String.class).setDefaultValue("def").setIsReturn(true).build());
        subBuilder.addCondition(new Condition(Factory.createPredicate("precond"), ConditionType.PRE));
        Condition.Disjunction d = new Condition.Disjunction(ConditionType.PRE);
        d.or(Factory.createPredicate("notObservable"), Observable.FALSE);
        d.or(Factory.createPredicate("observable"), Observable.TRUE);
        d.or(Factory.createPredicate("default"), Observable.DEFAULT);
        subBuilder.addCondition(new Condition(d));
        subBuilder.addCondition(new Condition(Factory.createPredicate("overall"), ConditionType.OVERALL));
        subBuilder.addEffect(new Effect(Factory.createPredicate("alwayseffect"), EffectType.ALWAYS));
        subBuilder.addEffect(new Effect(Factory.createPredicate("alwayssideeffect"), EffectType.ALWAYS, Observable.TRUE));
        subBuilder.addEffect(new Effect(Factory.createPredicate("successeffect"), EffectType.SUCCESS));
        subBuilder.addEffect(new Effect(Factory.createPredicate("successobservable"), EffectType.SUCCESS, Observable.TRUE));
        subBuilder.addEffect(new Effect(Factory.createPredicate("failureeffect"), EffectType.FAILURE));
        subBuilder.addEffect(new Effect(Factory.createPredicate("nonperformance"), EffectType.NONPERF));
        subBuilder.setCost("3");
        subBuilder.setBenefit("4");
        subBuilder.setTimeout("5");
        subBuilder.setMinUrg(".6");
        subBuilder.setMaxUrg(".7");
        subBuilder.addEventSpec(new EventSpec(EventSpec.EventType.CONTROL, "if"));
        subBuilder.addEventSpec(new EventSpec(EventSpec.EventType.OPERATOR, "gt 1 0"));
        subBuilder.addEventSpec(new EventSpec(EventSpec.EventType.CONTROL, "then"));
        subBuilder.addEventSpec(new EventSpec(EventSpec.EventType.ACTION, "?actor.action arg1 arg2 3.1415"));
        subBuilder.addEventSpec(new EventSpec(EventSpec.EventType.CONTROL, "endif"));
        //subBuilder.addEventSpec("obspec", "?actor.touching arg1 arg3");
        subBuilder.setDBFile("/config/edu/tufts/hrilab/action/asl/actionASLTest.asl"); // just so the ActionDBEntry.equals will work

        // Build script
        ActionDBEntry action1 = subBuilder.build(false);

        // Write entity to file, then read.
        ActionScriptLanguageWriter aslWriter = new ActionScriptLanguageWriter();
        String writeFilename = "src/test/resources" + action1.getDBFile();
        aslWriter.writeToFile(action1, writeFilename);
        ActionScriptLanguageParser parser = new ActionScriptLanguageParser();
        List<ActionDBEntry> parsedActions = parser.parseFromFile(action1.getDBFile(), null);
        assertNotNull(parsedActions);
        ActionDBEntry action1FromFile = parsedActions.get(0);

        // delete tmp file
        File file = new File(writeFilename);
        if (file.exists()) {
            file.delete();
        }

        // Compare to the original.
        assertEquals(action1, action1FromFile);
    }
}

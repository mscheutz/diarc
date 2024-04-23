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
import edu.tufts.hrilab.action.lock.ActionResourceLockLinear;
import edu.tufts.hrilab.fol.Factory;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.File;
import java.util.List;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertEquals;

public class ActionASLTest {

    @BeforeClass
    public static void setUp() {
        ActionResourceLockLinear.initResources();
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
        subBuilder.addResourceLock("motionLock");
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

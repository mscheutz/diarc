/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action;

import java.util.Arrays;
import java.util.List;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 *
 * @author willie
 */
public class ScriptParserTest {
  
  public ScriptParserTest() {
  }

  @Test
  public void testParseAct() {
    EventSpec spec1 = new EventSpec(EventSpec.EventType.ACTION, "test");
    List<EventSpec> specs = Arrays.asList(spec1);
    ScriptParser parser = new ScriptParser(specs);
    String cmd = parser.getCommand();
    EventSpec action = parser.getEventSpec();
    assertEquals("test", cmd);
    assertNotNull(action);
    assertEquals(action.getType(), EventSpec.EventType.ACTION);
  }

  @Test
  public void testParseObs() {
    EventSpec spec1 = new EventSpec(EventSpec.EventType.OBSERVATION, "test ob");
    List<EventSpec> specs = Arrays.asList(spec1);
    ScriptParser parser = new ScriptParser(specs);
    String cmd = parser.getCommand();
    EventSpec obs = parser.getEventSpec();
    assertEquals("test", cmd);
    assertNotNull(obs);
    assertEquals(obs.getType(), EventSpec.EventType.OBSERVATION);
  }

  @Test
  public void testIFParse() {
    EventSpec spec1 = new EventSpec(EventSpec.EventType.CONTROL, "if");
    EventSpec spec2 = new EventSpec(EventSpec.EventType.CONTROL, "true");
    EventSpec spec3 = new EventSpec(EventSpec.EventType.CONTROL, "then");
    EventSpec spec4 = new EventSpec(EventSpec.EventType.ACTION, "spin");
    EventSpec spec5 = new EventSpec(EventSpec.EventType.CONTROL, "endif");
    List<EventSpec> specs = Arrays.asList(spec1, spec2, spec3, spec4, spec5);
    ScriptParser parser = new ScriptParser(specs);
    String ifcmd = parser.getCommand();
    ScriptParser iftest = parser.getBody();
    String testcmd = iftest.getCommand();
    ScriptParser ifthen = iftest.getRest();
    String blockcmd = ifthen.getCommand();
    ScriptParser block = ifthen.getBody();
    String blockaction = block.getCommand();
    EventSpec spin = block.getEventSpec();
    ScriptParser rest = parser.getRest();
    assertEquals("if", ifcmd);
    assertEquals("true", testcmd);
    assertEquals("then", blockcmd);
    assertEquals("spin", blockaction);
    assertNotNull(spin);
    assertEquals(spin.getType(), EventSpec.EventType.ACTION);
    assertNull(rest);
  }
  
  @Test
  public void testWHILEParse() {
    EventSpec spec1 = new EventSpec(EventSpec.EventType.CONTROL, "while");
    EventSpec spec2 = new EventSpec(EventSpec.EventType.CONTROL, "true");
    EventSpec spec3 = new EventSpec(EventSpec.EventType.CONTROL, "do");
    //EventSpec spec4 = new EventSpec("spin");
    EventSpec spec5 = new EventSpec(EventSpec.EventType.CONTROL, "endwhile");
    List<EventSpec> specs = Arrays.asList(spec1, spec2, spec3, spec5);
    ScriptParser parser = new ScriptParser(specs);
    String whilecmd = parser.getCommand();
    ScriptParser whiletest = parser.getBody();
    String testcmd = whiletest.getCommand();
    ScriptParser whiledo = whiletest.getRest();
    String blockcmd = whiledo.getCommand();
    ScriptParser block = whiledo.getBody();
    String blockaction = block.getCommand();
    //String spin = block.getArgs().getCommand();
    ScriptParser rest = parser.getRest();
    assertEquals("while", whilecmd);
    assertEquals("true", testcmd);
    assertEquals("do", blockcmd);
    assertNull(blockaction);
    assertNull(rest);
  }
  
}

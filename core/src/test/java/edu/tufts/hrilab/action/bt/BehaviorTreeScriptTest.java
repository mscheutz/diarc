/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.bt;

import edu.tufts.hrilab.action.asl.ActionScriptLanguageParser;
import org.junit.Test;

import java.nio.charset.StandardCharsets;
import java.util.Base64;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static org.junit.Assert.*;

public class BehaviorTreeScriptTest {
  private static String compile(String source) {
    return BehaviorTreeScript.compile(source, "s", "!scriptResult", false);
  }

  private static String encoded(String value) {
    return Base64.getEncoder().encodeToString(value.getBytes(StandardCharsets.UTF_8));
  }

  private static int occurrences(String source, String value) {
    return source.split(Pattern.quote(value), -1).length - 1;
  }

  private static int operation(String source, String operator) {
    Matcher decoded = Pattern.compile("(!s_\\d+) = op:invokeMethod\\(\\?execution, \"decoded\", \""
        + Pattern.quote(encoded(operator)) + "\"\\);").matcher(source);
    assertTrue("Missing operator " + operator, decoded.find());
    int step = source.indexOf("\"scriptBinary\", " + decoded.group(1) + ",");
    assertTrue("Missing binary step for " + operator, step >= 0);
    return step;
  }

  private static void rejects(String source) {
    rejects(() -> compile(source));
  }

  private static void rejects(Runnable operation) {
    try {
      operation.run();
    } catch (IllegalArgumentException expected) {
      return;
    }
    fail("Expected invalid script or compiler arguments");
  }

  @Test
  public void emitsAllAssignmentsAndScalarFamilies() {
    String asl = compile("x:=0x10; x=2; x+=1; x-=1; x*=2; x/=2;"
        + "a:=1.25e-2; b:=true; c:=false; name:='hello';"
        + "name+=' world'; x+y*3/2-1; x&y|3^2; ~x; -y; !b; name..x;");
    for (String helper : List.of("scriptNumber", "scriptAssign", "scriptRead",
        "scriptBinary", "scriptUnary", "decoded")) {
      assertTrue(asl.contains("\"" + helper + "\""));
    }
    for (String operator : List.of(":=", "=", "+=", "-=", "*=", "/=", "..",
        "&", "|", "^", "~", "!", "-")) {
      assertTrue("Missing " + operator, asl.contains("\"" + encoded(operator) + "\""));
    }
    assertTrue(asl.contains("\"scriptNumber\", \"0x10\""));
    assertTrue(asl.contains("\"scriptNumber\", \"1.25e-2\""));
    assertTrue(asl.contains("\"scriptNumber\", \"1.0\""));
    assertTrue(asl.contains("\"scriptNumber\", \"0.0\""));
    assertTrue(asl.contains("!scriptResult = op:invokeMethod(?execution, \"scriptIdentity\", !s_"));
    assertFalse(asl.contains("java.lang.Object !scriptResult;"));
  }

  @Test
  public void respectsUpstreamPrecedenceAndLeftAssociativity() {
    String asl = compile("a+b*c&d|e^f");
    assertTrue(operation(asl, "*") < operation(asl, "+"));
    assertTrue(operation(asl, "+") < operation(asl, "&"));
    assertTrue(operation(asl, "&") < operation(asl, "|"));
    assertTrue(operation(asl, "|") < operation(asl, "^"));
    asl = compile("a-b-c");
    assertEquals(2, occurrences(asl, "\"scriptBinary\""));
    assertTrue(asl.contains("\"scriptRead\""));
    asl = compile("(a+b)*c");
    assertTrue(operation(asl, "+") < operation(asl, "*"));
  }

  @Test
  public void sequencesStatementsAndReturnsOnlyLastValue() {
    String asl = compile("a:=1;;; b:=2;; a+b;;");
    assertEquals(2, occurrences(asl, "\"scriptAssign\""));
    assertEquals(1, occurrences(asl, "!scriptResult ="));
    assertTrue(asl.lastIndexOf("\"scriptAssign\"") < asl.lastIndexOf("\"scriptBinary\""));
    assertTrue(asl.endsWith(";\n"));
    // Upstream also allows adjacent expressions without a semicolon.
    assertEquals(2, occurrences(compile("1 2"), "\"scriptNumber\""));
  }

  @Test
  public void logicalOperatorsAreEagerAndOnlyTernariesBecomeBranches() {
    String asl = compile("false && (a:=1); true || (b:=2); flag ? (c:=3) : (d:=4)");
    assertEquals(1, occurrences(asl, "if ("));
    assertEquals(1, occurrences(asl, "} else {"));
    assertEquals(2, occurrences(asl, "\"scriptBinary\""));
    assertFalse(asl.contains(" && "));
    assertFalse(asl.contains(" || "));
    assertTrue(asl.indexOf("\"scriptAssign\"") < operation(asl, "&&"));
    assertTrue(asl.indexOf("\"" + encoded("b") + "\"") < operation(asl, "||"));
    assertTrue(operation(asl, "||") < asl.indexOf("if ("));
    assertTrue(asl.contains("\"scriptConditionalTruthy\""));
    assertFalse(asl.contains("\"scriptTruthy\""));
    assertFalse(asl.contains("evaluateScript"));
    assertFalse(asl.contains("executeScript"));
  }

  @Test
  public void logicalRightHandReadsCannotSkipMissingVariableErrors() {
    for (String script : List.of("false && missing", "true || missing")) {
      String asl = compile(script);
      assertFalse(asl.contains("if ("));
      assertEquals(1, occurrences(asl, "\"scriptRead\""));
      assertEquals(1, occurrences(asl, "\"scriptBinary\""));
      assertTrue(asl.indexOf("\"scriptRead\"") < asl.indexOf("\"scriptBinary\""));
    }
    String asl = compile("(left+=1) && (right+=1)");
    int firstAssignment = asl.indexOf("\"scriptAssign\"");
    int secondAssignment = asl.indexOf("\"scriptAssign\"", firstAssignment + 1);
    assertTrue(firstAssignment < secondAssignment);
    assertTrue(secondAssignment < operation(asl, "&&"));
    assertFalse(asl.contains("if ("));
  }

  @Test
  public void preparesAssignmentTargetsBeforeAnyRightHandSideEvaluation() {
    String asl = compile("x=(x:=1)");
    int firstPreparation = asl.indexOf("\"scriptPrepareAssignment\"");
    int secondPreparation = asl.indexOf("\"scriptPrepareAssignment\"", firstPreparation + 1);
    int literal = asl.indexOf("\"scriptNumber\"");
    int firstAssignment = asl.indexOf("\"scriptAssign\"");
    int secondAssignment = asl.indexOf("\"scriptAssign\"", firstAssignment + 1);
    assertTrue(firstPreparation >= 0);
    assertTrue(firstPreparation < secondPreparation);
    assertTrue(secondPreparation < literal);
    assertTrue(literal < firstAssignment);
    assertTrue(firstAssignment < secondAssignment);
    assertEquals(2, occurrences(asl, "\"scriptPrepareAssignment\""));
    assertFalse(asl.contains("if ("));
    asl = compile("x:=x");
    assertTrue(asl.indexOf("\"scriptPrepareAssignment\"") < asl.indexOf("\"scriptRead\""));
    assertTrue(asl.indexOf("\"scriptRead\"") < asl.indexOf("\"scriptAssign\""));
  }

  @Test
  public void separatesConditionalStringTruthFromStrictPredicateTruth() {
    String asl = BehaviorTreeScript.compile("'text' ? 1 : 0", "s", "!scriptResult", true);
    assertEquals(1, occurrences(asl, "\"scriptConditionalTruthy\""));
    assertEquals(1, occurrences(asl, "\"scriptTruthy\""));
    assertTrue(asl.indexOf("\"scriptConditionalTruthy\"") < asl.indexOf("\"scriptTruthy\""));
    assertTrue(asl.contains("!scriptResult = op:invokeMethod(?execution, \"scriptTruthy\", "));
    assertFalse(compile("'text'").contains("\"scriptConditionalTruthy\""));
  }

  @Test
  public void chainedComparisonsReuseMiddleOperandsAndShortCircuit() {
    String asl = compile("a < (b+=1) <= c != d");
    assertEquals(1, occurrences(asl, "\"scriptAssign\""));
    assertEquals(3, occurrences(asl, "\"scriptBinary\""));
    assertEquals(2, occurrences(asl, "if ("));
    assertTrue(operation(asl, "<") < asl.indexOf("if ("));
    assertTrue(asl.indexOf("if (") < asl.indexOf("\"" + encoded("c") + "\""));
    for (String operator : List.of("==", "!=", "<", ">", "<=", ">=")) {
      assertEquals(1, occurrences(compile("a" + operator + "b"), "\"scriptBinary\""));
    }
    asl = compile("3 < 2 < (sideEffect+=1) < missing");
    assertTrue(operation(asl, "<") < asl.indexOf("if ("));
    assertTrue(asl.indexOf("if (") < asl.indexOf("\"scriptPrepareAssignment\""));
    assertTrue(asl.lastIndexOf("if (") < asl.indexOf("\"scriptRead\"",
        asl.indexOf("\"" + encoded("missing") + "\"")));
    assertEquals(1, occurrences(asl, "\"scriptAssign\""));
  }

  @Test
  public void hoistsUniqueLocalsAndDoesNotRedeclareResult() {
    String asl = BehaviorTreeScript.compile("a ? (b && c) : (d || e)",
        "!local", "!local_0", true);
    Set<String> names = new HashSet<>();
    boolean instructions = false;
    for (String line : asl.split("\n")) {
      if (line.startsWith("java.lang.Object ")) {
        assertFalse("Declaration after executable instruction", instructions);
        String variable = line.substring("java.lang.Object ".length(), line.length() - 1);
        assertTrue(variable.matches("!local_\\d+"));
        assertFalse("!local_0".equals(variable));
        assertTrue("Duplicate temporary", names.add(variable));
      } else {
        instructions = true;
      }
    }
    assertTrue(names.size() > 5);
    assertTrue(asl.contains("!local_0 = op:invokeMethod(?execution, \"scriptTruthy\", "));
    assertTrue(compile("a").contains("!s_0"));
    assertTrue(BehaviorTreeScript.compile("a", "other", "?result", false).contains("!other_0"));
  }

  @Test
  public void arbitraryStringsAndKeysNeverAppearAsAslLiterals() {
    String value = "!scriptResult ?execution \"quoted\" \\slash\n雪";
    String asl = compile("plain_key := '" + value + "'; plain_key");
    assertFalse(asl.contains(value));
    assertFalse(asl.contains("plain_key"));
    assertTrue(asl.contains("\"" + encoded(value) + "\""));
    assertTrue(asl.contains("\"" + encoded("plain_key") + "\""));
    for (String text : List.of("", "(", ")", ";", "+", "true", ":=")) {
      assertTrue(compile("'" + text + "'").contains("\"" + encoded(text) + "\""));
    }
  }

  @Test
  public void generatedProgramsParseAsNativeAsl() {
    for (String script : List.of("x:=0xAB; y:=1e-3; x+y", "a ? b ? c : d : e",
        "a && (b:=2) || c", "a < b <= c != d", "x:=y:=3", "x='!unsafe ?text \"quoted\"'")) {
      String asl = "(java.lang.Object ?result) = compiledScript("
          + "edu.tufts.hrilab.action.bt.BehaviorTreeExecution ?execution) {\n"
          + "java.lang.Object !scriptResult;\n" + compile(script)
          + "?result = op:invokeMethod(?execution, \"scriptIdentity\", !scriptResult);\n}\n";
      var actions = new ActionScriptLanguageParser().parseFromString("compiled-script.asl", null, asl);
      assertNotNull(script, actions);
      assertEquals(script, 1, actions.size());
    }
  }

  @Test
  public void rejectsMalformedAndUnsupportedSyntax() {
    for (String source : List.of("", " ", "\t\n\r", ";", ";;1", "'unterminated",
        "\"double quoted\"", "0x", "0xG", "0x1.2", "3e", "3e+", "1.", ".5", "3foo",
        "1 +", "(1", "1)", "a ? b", "a ? : c", "a ? b :", "1=2", "'a':=2", "(a+b)=3",
        "x%=2", "1%2", "a[0]", "f(1,2)", "a.b", "{a}", "1<<2", "+1", "1 //comment",
        "1 /*comment*/", "@", "@1", "9223372036854775808",
        "0xFFFFFFFFFFFFFFFF", "1e9999")) {
      rejects(source);
    }
  }

  @Test
  public void enforcesBoundedCompilationAndLegalVariableNames() {
    rejects("(".repeat(150) + "1" + ")".repeat(150));
    rejects("a+".repeat(150) + "a");
    rejects("1;".repeat(1100));
    rejects(" ".repeat(65537));
    for (String prefix : List.of("", "_x", "Upper", "a-b", "a;")) {
      rejects(() -> BehaviorTreeScript.compile("1", prefix, "!result", false));
    }
    for (String result : List.of("result", "!Upper", "!result; exit(FAIL)", "")) {
      rejects(() -> BehaviorTreeScript.compile("1", "s", result, false));
    }
    rejects(() -> BehaviorTreeScript.compile(null, "s", "!result", false));
  }
}

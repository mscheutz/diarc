/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * @author Zhouqi
 */

package edu.tufts.hrilab.action.asl;

import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.execution.control.ControlFactory.Control;
import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.StringJoiner;
import java.util.stream.Collectors;

public class ActionScriptLanguageWriter {

  private final static Logger log = LoggerFactory.getLogger(ActionScriptLanguageWriter.class);

  private StringBuilder sb;

  public void writeToFile(List<ActionDBEntry> actionScripts, String filename) {
    StringBuilder localSB = new StringBuilder();

    // write each action
    actionScripts.forEach(action -> localSB.append(writeAction(action)).append("\n"));

    try {
      FileWriter fw = new FileWriter(filename);
      fw.write(localSB.toString());
      fw.close();
    } catch (IOException e) {
      log.error("Failed to write script to ASL file.", e);
    }
  }

  /**
   * Convenience method to write a single action script to file.
   * @param actionScript
   * @param filename
   */
  public void writeToFile(ActionDBEntry actionScript, String filename) {
    writeToFile(new ArrayList<>(Arrays.asList(actionScript)), filename);
  }

  public String writeAction(ActionDBEntry entry) {
    sb = new StringBuilder();

    openBlock(generateMethodSignature(entry));
    writeLocalParameters(entry);
    writeSettings(entry);
    writeConditions(entry);
    writeEffects(entry);
    writeInterrupts(entry);
    writeObserves(entry);
    writeLocks(entry);
    writeMainSpecs(entry);
    closeBlock();

    return sb.toString();
  }

  private String generateMethodSignature(ActionDBEntry entry) {
    // write return parameters
    StringJoiner sj = new StringJoiner(", ", "(", ")");
    entry.getRoles().stream().filter(x -> x.isReturn).forEach(rt -> sj.add(actionBindingString(rt)));
    String returnVars = sj.toString();

    // write input parameters
    StringJoiner sj2 = new StringJoiner(", ", "(", ")");
    entry.getRoles().stream().filter(x -> !x.isLocal && !x.isReturn).forEach(in -> sj2.add(actionBindingString(in)));
    String inputVars = sj2.toString();

    // action name
    String name = entry.getName();

    // description
    String desc = entry.getDescription();

    // generate method signature
    if (desc != null && !desc.isEmpty()) {
      return returnVars + " = " + name + "[\"" + desc + "\"]" + inputVars;
    } else {
      return returnVars + " = " + name + inputVars;
    }
  }

  private void writeLocalParameters(ActionDBEntry entry) {
    if (entry.getRoles().stream().anyMatch(x -> x.isLocal)) {
      entry.getRoles().stream().filter(x -> x.isLocal).forEach(lc -> cat(actionBindingString(lc) + ";"));
    }
  }

  private String actionBindingString(ActionBinding ab) {
    String varNamePlusType = ab.javaType.getCanonicalName() + " " + ab.name;
    String assign = "";
    if (ab.hasDefaultValue()) {
      if (ab.javaType.equals(String.class)) {
        assign = " = \"" + ab.defaultValue.toString() + "\"";
      } else {
        assign = " = " + ab.defaultValue.toString();
      }
    }
    return varNamePlusType + assign;
  }

  private void writeSettings(ActionDBEntry entry) {
    if (entry.getBenefit() != 0.0) {
      cat("benefit = " + entry.getBenefit() + ";");
    }
    if (entry.getCost() != 0.0) {
      cat("cost = " + entry.getCost() + ";");
    }
    if (entry.getMinUrg() != 0.0) {
      cat("minurg = " + entry.getMinUrg() + ";");
    }
    if (entry.getMaxUrg() != 1.0) {
      cat("maxurg = " + entry.getMaxUrg() + ";");
    }
    if (entry.hasTimeout()) {
      cat("timeout = " + entry.getTimeout() + ";");
    }
  }

  private void writeConditions(ActionDBEntry entry) {
    if (!entry.getPreConditions().isEmpty() || !entry.getOverallConditions().isEmpty() || !entry.getObligationConditions().isEmpty()) {
      openBlock("conditions :");

      for (Condition c : entry.getPreConditions()) {
        if (c.isDisjunction()) {
          openBlock("or :");
        }
        c.getPredicates().forEach(((predicate, observable) -> writePredicateString("pre", predicate, observable)));
        if (c.isDisjunction()) {
          closeBlock();
        }
      }

      for (Condition c : entry.getOverallConditions()) {
        if (c.isDisjunction()) {
          openBlock("or :");
        }
        c.getPredicates().forEach(((predicate, observable) -> writePredicateString("overall", predicate, observable)));
        if (c.isDisjunction()) {
          closeBlock();
        }
      }

      for (Condition c : entry.getObligationConditions()) {
        if (c.isDisjunction()) {
          openBlock("or :");
        }
        c.getPredicates().forEach(((predicate, observable) -> writePredicateString("obligation", predicate, observable)));
        if (c.isDisjunction()) {
          closeBlock();
        }
      }

      closeBlock();
    }
  }

  private void writeEffects(ActionDBEntry entry) {
    List<Effect> effects = entry.getEffects().stream().filter(e -> !e.isAutoGenerated()).collect(Collectors.toList());
    if (!effects.isEmpty()) {
      openBlock("effects :");
      effects.forEach(e -> writePredicateString(e.getType().name().toLowerCase(), e.getPredicate(), e.getObservable()));
      closeBlock();
    }
  }

  private void writeObserves(ActionDBEntry entry) {
    entry.getObservations().forEach(e -> writePredicateString("observes", e, Observable.DEFAULT));
  }

  private void writePredicateString(String prefix, Predicate predicate, Observable observable) {
    String obs;
    switch (observable) {
      case TRUE:
        obs = " obs";
        break;
      case FALSE:
        obs = " infer";
        break;
      default:
        obs = "";
        break;
    }
    StringJoiner sj = new StringJoiner(",", "(", ")");
    predicate.getArgs().forEach(s -> sj.add(s.toString()));
    String predicateString = predicate.getName() + sj;
    if (predicate.isNegated()) {
      predicateString = "~" + predicateString;
    }
    cat(prefix + obs + " : " + predicateString +";");
  }

  private void writeInterrupts(ActionDBEntry entry) {
    if (entry.hasOnCancelEvent() || entry.hasOnResumeEvent() || entry.hasOnSuspendEvent()) {
      openBlock("onInterrupt :");
      if (entry.hasOnCancelEvent()) {
        cat("cancel: " + generateSpec(entry.getOnCancelEvent()));
      }
      if (entry.hasOnSuspendEvent()) {
        cat("suspend: " + generateSpec(entry.getOnSuspendEvent()));
      }
      if (entry.hasOnResumeEvent()) {
        cat("resume: " + generateSpec(entry.getOnResumeEvent()));
      }
      closeBlock();
    }
  }
  private void writeLocks(ActionDBEntry entry) {
    entry.getResourceLockNames().forEach(s -> cat("locks : " + s + ";"));
  }

  private void writeMainSpecs(ActionDBEntry entry) {
    if (!entry.getEventSpecs().isEmpty()) {
      boolean inConditioning = false;
      StringBuilder conditionString = new StringBuilder();
      ArrayDeque<String> conditionNameStack = new ArrayDeque<>();
      ArrayDeque<StringJoiner> conditionStack = new ArrayDeque<>();
      for (EventSpec es : entry.getEventSpecs()) {
        if (inConditioning) {
          switch (es.getType()) {
            case ACTION:
            case OPERATOR:
            case OBSERVATION:
            case GOAL:
              if (!conditionStack.isEmpty()) {
                conditionStack.peek().add(generateSpec(es));
              } else {
                conditionString.append("(").append(generateSpec(es)).append(")");
              }
              break;
            case CONTROL:
              switch (Control.fromString(es.getCommand())) {
                case OR:
                  conditionNameStack.push("or");
                  conditionStack.push(new StringJoiner(" || ", "(", ")"));
                  break;
                case AND:
                  if (!conditionNameStack.isEmpty()){
                    if (conditionNameStack.peek().equals("and")) {
                      conditionNameStack.push("and");
                      conditionStack.push(new StringJoiner(" && ", "(", ")"));
                    } else if (conditionNameStack.peek().equals("or")) {
                      conditionNameStack.push("and");
                      conditionStack.push(new StringJoiner(" && "));
                    } else {
                      conditionNameStack.push("and");
                      conditionStack.push(new StringJoiner(" && ", "(", ")"));
                    }
                  } else {
                    conditionNameStack.push("and");
                    conditionStack.push(new StringJoiner(" && ", "(", ")"));
                  }
                  break;
                case NOT:
                  if (conditionStack.isEmpty()) {
                    conditionStack.push(new StringJoiner("", "(~", ")"));
                  } else {
                    conditionStack.push(new StringJoiner("", "~", ""));
                  }
                  conditionNameStack.push("not");
                  break;
                case ENDOR:
                case ENDAND:
                case ENDNOT:
                  if (!conditionNameStack.isEmpty()) {
                    conditionNameStack.pop();
                    if (conditionNameStack.isEmpty()) {
                      conditionString.append(conditionStack.pop().toString());
                    } else {
                      String temp = conditionStack.pop().toString();
                      if (!conditionStack.isEmpty()) {
                        conditionStack.peek().add(temp);
                      } else {
                        log.error("Control conditioning goes wrong in the ActionDBEntry.");
                      }
                    }
                  } else {
                    log.error("Control conditioning goes wrong in the ActionDBEntry.");
                  }
                  break;
                case THEN:
                case DO:
                  if (conditionNameStack.isEmpty() && conditionStack.isEmpty()) {
                    openBlock(conditionString.toString());
                    inConditioning = false;
                  } else {
                    inConditioning = false;
                    conditionNameStack.clear();
                    conditionStack.clear();
                    log.error("Control conditioning goes wrong in the ActionDBEntry.");
                  }
                  break;
                case TRUE:
                  if (!conditionStack.isEmpty()) {
                    conditionStack.peek().add("true");
                  } else {
                    conditionString.append("(true)");
                  }
                  break;
                case BLOCK:
                case END:
                case ASYNC:
                case ENDASYNC:
                case JOIN:
                case RETURN:
                case IF:
                case ELSEIF:
                case ELSE:
                case ENDIF:
                case WHILE:
                case ENDWHILE:
                case FOR:
                case TRY:
                case EXIT:
                case CATCH:
                case ENDFOR:
                case ENDTRY:
                case FINALLY:
                case FOREACH:
                case OTHER:
                default:
                  log.error(String.format("Control %s should not exist inside conditioning.", es.toString()));
                  break;
              }
              break;
            default:
              break;
          }
        } else {
          switch (es.getType()) {
            case ACTION:
            case OPERATOR:
            case OBSERVATION:
            case GOAL:
              cat(generateSpec(es) + ";");
              break;
            case CONTROL:
              switch (Control.fromString(es.getCommand())) {
                case IF:
                  inConditioning = true;
                  conditionString = new StringBuilder("if ");
                  break;
                case ELSEIF:
                  closeBlock();
                  inConditioning = true;
                  conditionString = new StringBuilder("elseif ");
                  break;
                case ELSE:
                  closeBlock();
                  openBlock("else");
                  break;
                case WHILE:
                  inConditioning = true;
                  conditionString = new StringBuilder("while ");
                  break;
                case FOR:
                  String name = es.getInputArgs().get(0);
                  if (es.getInputArgs().size() == 4) {
                    openBlock("for (" + name + "; "
                        + name + " " + es.getInputArgs().get(1) + " " + es.getInputArgs().get(2) + "; "
                        + name + " " + es.getInputArgs().get(3) + ")");
                  } else {
                    openBlock("for (" + name + "=" + es.getInputArgs().get(1) + "; "
                        + name + " " + es.getInputArgs().get(2) + " " + es.getInputArgs().get(3) + "; "
                        + name + " " + es.getInputArgs().get(4) + ")");
                  }
                  break;
                case FOREACH:
                  String[] tokens = es.toString().split(" ");
                  openBlock(tokens[0] + "(" + tokens[1] + " : " + tokens[2] +")");
                  break;
                case TRY:
                  openBlock("try");
                  break;
                case CATCH:
                  closeBlock();
                  StringJoiner catchArgs = new StringJoiner(", ", "(", ")");
                  es.getInputArgs().forEach(catchArgs::add);
                  openBlock("catch" + catchArgs.toString());
                  break;
                case FINALLY:
                  closeBlock();
                  openBlock("finally");
                  break;
                case ENDIF:
                case ENDWHILE:
                case ENDTRY:
                case ENDFOR:
                case ENDASYNC:
                  closeBlock();
                  break;
                case EXIT:
                  StringJoiner exitArgs = new StringJoiner(", ", "(", ")");
                  es.getInputArgs().forEach(exitArgs::add);
                  cat("exit" + exitArgs.toString() + ";");
                  break;
                case ASYNC:
                  if (es.getInputArgs().size() == 0) {
                    openBlock("async ()");
                  } else {
                    openBlock("async (" + es.getInputArgs().get(0) + ")");
                  }
                  break;
                case JOIN:
                  StringJoiner joinArgs = new StringJoiner(", ", "(", ")");
                  es.getInputArgs().forEach(joinArgs::add);
                  cat("join" + joinArgs.toString() + ";");
                  break;
                case RETURN:
                  cat("return;");
                  break;
                case BLOCK:
                case END:
                case DO:
                case AND:
                case OR:
                case NOT:
                case ENDOR:
                case ENDAND:
                case ENDNOT:
                case THEN:
                case TRUE:
                case OTHER:
                default:
                  log.error(String.format("Control %s should not exist outside conditioning. Control type: %s", es.toString(), es.getType().toString()));
                  break;
              }
              break;
            default:
              break;
          }
        }
      }
    }
  }

  private String generateSpec(EventSpec spec) {
    String command = spec.getCommand();
    StringJoiner sj;
    if (Utilities.isScriptVariable(command)) {
      sj = new StringJoiner(", ");
    } else {
      sj = new StringJoiner(", ", "(", ")");
    }
    spec.getInputArgs().forEach(sj::add);

    String type;
    switch (spec.getType()) {
      case ACTION:
        type = "act";
        break;
      case OBSERVATION:
        type = "obs";
        break;
      case GOAL:
        type = "goal";
        break;
      case OPERATOR:
        type = "op";
        break;
      case TSC:
        type = "tsc";
        break;
      default:
        log.error("Unexpected type: " + spec);
        type = spec.getType().toString().toLowerCase();
    }

    if(!spec.getActor().equals("?actor") && spec.getActor().length()>0) {
      type = spec.getActor() + "." + type;
    }
    String returnValue="";

    if(spec.getReturnArgs().size() == 1) {
      returnValue = spec.getReturnArgs().get(0) +" = ";
    } else if(spec.getReturnArgs().size()>1){
      log.warn("[generateSpec] printing multiple return args not supported: "+spec.getReturnArgs());
    }

    return returnValue + type + ":" + command + sj;
  }

  private int indent = 0;

  private void openBlock(String str) {
    makeIndent(indent++);
    sb.append(str).append(" {\n");
  }

  private void closeBlock() {
    makeIndent(--indent);
    sb.append("}\n");
  }

  private void cat(String str) {
    makeIndent(indent);
    sb.append(str).append("\n");
  }

  private void makeIndent(int i) {
    if (i > 0) {
      while(i-- > 0) {
        sb.append("    ");
      }
    }
  }
}

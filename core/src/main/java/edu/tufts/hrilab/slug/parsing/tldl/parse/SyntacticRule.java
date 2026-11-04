package edu.tufts.hrilab.slug.parsing.tldl.parse;


import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

//Used to be an inner class of Entry, and probably should be, but isn't right now
//Implementation of CCG, used to represent the syntactic state of a node in the Parse space
//and to determine if node combinations are valid
public class SyntacticRule implements Cloneable {
  private static Logger log = LoggerFactory.getLogger(SyntacticRule.class);

  //String form of entire CCG rule
  private String type = "";

  private SyntacticRule returnType;

  private SyntacticRule argType;
  //  boolean isWildcard=false;
  boolean isLiteral = false;
  //number of apply statements in the rule and it's return types
  int depth;

  //default order of application is left to right (parens can change this)
  boolean forwardApply = false;
  boolean backwardApply = false;

  public String getType() {
    return type;
  }

  public SyntacticRule getReturnType() {
    return returnType;
  }

  public SyntacticRule getArgType() {
    return argType;
  }
  public boolean isLiteral() {
    return isLiteral;
  }

  /**
   * Construct new instance from string form specified in dictionary
   */
  public SyntacticRule(String definition) {
    type = definition.trim();
    List<String> parts = parseDef(definition.trim());

    if (parts.size() == 1) {
      returnType = null;
      depth = 0;
      if (type.equals("LIT*")) {
        isLiteral = true;
        type = type.substring(0, type.length() - 1);
      }
//      else if (type.endsWith("*")) {
////       isWildcard=true;
//       type=type.substring(0,type.length()-1);
//      }
    } else {
      if (parts.get(1).equals("/")) {
        forwardApply = true;
      } else if (parts.get(1).equals("\\")) {
        backwardApply = true;
      }
      returnType = new SyntacticRule(parts.get(0).trim());
      depth = 1 + returnType.depth;
      argType = new SyntacticRule(parts.get(2));
      log.debug("type: " + type);
      log.debug("return type: " + returnType);
      log.debug("argtype: " + argType);
    }
  }

  //Helper method used in dictionary file constructor
  //parses definition ito highest level apply statement
  private List<String> parseDef(String definition) {
    List<String> pieces = new ArrayList<>(3);
    int parenDepth = 0;
    for (int i = 0; i < definition.length(); i++) {
      char c = definition.charAt(i);

      if (c == '(') {
        parenDepth++;
      } else if (c == ')') {
        parenDepth--;
      }

      if (parenDepth == 0 && (c == '/' || c == '\\')) {
        String LHS = definition.substring(0, i).trim();
        String RHS = definition.substring(i + 1).trim();
        //remove parens
        if (LHS.startsWith("(") && LHS.endsWith(")")) {
          LHS = LHS.substring(1, LHS.length() - 1);
        }
        if (RHS.startsWith("(") && RHS.endsWith(")")) {
          RHS = RHS.substring(1, RHS.length() - 1);
        }
        log.debug("parsed def- LHS: " + LHS + " opp: " + c + " RHS: " + RHS);
        pieces.add(LHS);
        pieces.add(String.valueOf(c));
        pieces.add(RHS);
        break;
      }
    }
    //if no apply statement then just add
    if (pieces.size() == 0) {
      pieces.add(definition);
    }
    return pieces;
  }

  /**
   * Construct a new syntatic rule from the syntaxes present in a prase context
   * TODO:Does this work with the template as it is recursively defines syntactic rules to create a new rule from a nested set of args?
   */
  SyntacticRule(SyntacticRule ret, List<SyntacticRule> preceding, List<SyntacticRule> following) {
    if (preceding.isEmpty() && following.size() == 1) {
      returnType = ret;
      depth = 1;
      argType = following.get(0);
      forwardApply = true;
      type = ret.type + "/" + argType.type;
    } else if (preceding.size() == 1 && following.isEmpty()) {
      returnType = ret;
      depth = 1;
      argType = preceding.get(0);
      //Brad: should this be true or false
      backwardApply = true;
      type = ret.type + "\\" + argType.type;
    }
    //do preceding first i guess?
    else if (!preceding.isEmpty()) {
      argType = preceding.remove(preceding.size() - 1);
      backwardApply = true;
      returnType = new SyntacticRule(ret, preceding, following);
      depth = returnType.depth + 1;
      if (returnType.argType != null) {
        type += "(";
      }
      type += returnType.type;
      if (returnType.argType != null) {
        type += ")";
      }
      type += "\\" + argType.type;
    } else if (!following.isEmpty()) {
      argType = following.remove(following.size() - 1);
      forwardApply = true;
      returnType = new SyntacticRule(ret, preceding, following);
      depth = returnType.depth + 1;
      if (returnType.argType != null) {
        type += "(";
      }
      type += returnType.type;
      if (returnType.argType != null) {
        type += ")";
      }
      type += "/" + argType.type;
    } else {
      returnType = ret;
      type = ret.type;
      depth = 0;
      argType = null;
    }

  }

  public SyntacticRule clone() throws CloneNotSupportedException {
    SyntacticRule clonedSyntacticRule = (SyntacticRule) super.clone();
    clonedSyntacticRule.type = this.type;
    if (this.returnType == null) {
      clonedSyntacticRule.returnType = null;
    } else {
      clonedSyntacticRule.returnType = this.returnType.clone();
    }
    if (this.argType == null) {
      clonedSyntacticRule.argType = null;
    } else {
      clonedSyntacticRule.argType = this.argType.clone();
    }
    clonedSyntacticRule.depth = this.depth;
//    clonedSyntacticRule.isWildcard= this.isWildcard;
    return clonedSyntacticRule;
  }

  boolean checkArgsApply(SyntacticRule possibleArg) {
    if (argType != null) {
      log.debug("this.type: " + this.type + " this.argType.type " + argType + " returnType.type" + returnType);
      log.debug("Checking for apply, arg type " + argType.type + " possibleArg type " + possibleArg.type);
      return possibleArg.type.equals(argType.type);
    }
    return false;
  }

  //TODO:brad: this should be implemented
//  public boolean checkArgsCompose(SyntacticRule possibleArg) {
//    log.debug("Checking for compose:");
//    if (possibleArg.returnType != null) {
//      log.debug("arg type " + argType.type + " possibleArg type " + possibleArg.returnType.type);
//      if (possibleArg.returnType.type.equals(argType.type) || possibleArg.type.equals(argType.type)) return true;
//    }
//    return false;
//  }

  //performs rule application
  SyntacticRule apply(SyntacticRule other) {
    if (checkArgsApply(other)) {
      if (returnType == null) {
        log.error("invalid apply call " + type);
        return this;
      } else if (returnType.returnType == null) {
        try {
          return returnType.clone();
        } catch (Exception e) {
          log.error("[apply] ", e);
        }
      } else {
        try {
          return returnType.clone();
        } catch (Exception e) {
          log.error("[apply] ", e);
        }
      }
    }
    return null;
  }

  //TODO:brad: this should be implemented
//perform rule composition
//  SyntacticRule compose(SyntacticRule arg) {
//    SyntacticRule ret = new SyntacticRule(returnType, arg.argType, arg.forwardApply);
//    return ret;
//  }

  @Override
  public String toString() {
    return type;
  }

}
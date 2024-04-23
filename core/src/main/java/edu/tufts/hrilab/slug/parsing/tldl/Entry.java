/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.tldl;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

//Used to be an inner class of Entry, and probably should be, but isn't right now
//Implementation of CCG, used to represent the syntactic state of a node in the Parse space
//and to determine if node combinations are valid
class SyntacticRule implements Cloneable {
  private static Logger log = LoggerFactory.getLogger(SyntacticRule.class);

  //String form of entire CCG rule
  String type = "";

  private SyntacticRule returnType;
  SyntacticRule argType;
//  boolean isWildcard=false;
  boolean isLiteral=false;
  //number of apply statements in the rule and it's return types
  int depth;

  //default order of application is left to right (parens can change this)
  boolean forwardApply = false;
  boolean backwardApply = false;

  public SyntacticRule getReturnType() {
    return returnType;
  }
  /**
   * Construct new instance from string form specified in dictionary
   */
  SyntacticRule(String definition) {
    type = definition.trim();
    List<String> parts = parseDef(definition.trim());

    if (parts.size() == 1) {
      returnType = null;
      depth = 0;
      if(type.equals("LIT*")){
        isLiteral=true;
        type=type.substring(0,type.length()-1);
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

      if (c == '(') parenDepth++;
      else if (c == ')') parenDepth--;

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
    if (pieces.size() == 0) pieces.add(definition);
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
      if (returnType.argType != null) type += "(";
      type += returnType.type;
      if (returnType.argType != null) type += ")";
      type += "\\" + argType.type;
    } else if (!following.isEmpty()) {
      argType = following.remove(following.size() - 1);
      forwardApply = true;
      returnType = new SyntacticRule(ret, preceding, following);
      depth = returnType.depth + 1;
      if (returnType.argType != null) type += "(";
      type += returnType.type;
      if (returnType.argType != null) type += ")";
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
    if(this.returnType == null){
      clonedSyntacticRule.returnType =  null;
    }else{
      clonedSyntacticRule.returnType =  this.returnType.clone();
    }
    if(this.argType == null) {
      clonedSyntacticRule.argType = null;
    } else{
        clonedSyntacticRule.argType =  this.argType.clone();
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

//Used to be an inner class of Entry and probably should be
//implementation of lambda calculus used to define semantics of parse rules.
class SemanticRule implements Cloneable {

  private static Logger log = LoggerFactory.getLogger(SemanticRule.class);
  private String definition;
  String body;
  private boolean argsSet = false;
  //  boolean childrenSet = false;
  private SortedMap<String, SemanticRule> args;
  LinkedList<String> unsetArgs;
  String ghTier = "";

  public SemanticRule clone() throws CloneNotSupportedException {
    SemanticRule clonedSemanticRule = (SemanticRule) super.clone();
    clonedSemanticRule.body = this.body;
    clonedSemanticRule.argsSet = this.argsSet;
    clonedSemanticRule.args = new TreeMap<>();
    for (Map.Entry<String, SemanticRule> e : this.args.entrySet()) {
      if(e.getValue() == null){
        clonedSemanticRule.args.put(e.getKey(), null);
      }else{
        clonedSemanticRule.args.put(e.getKey(), e.getValue().clone());
      }
    }
    clonedSemanticRule.unsetArgs = new LinkedList<>(this.unsetArgs);
    clonedSemanticRule.ghTier = this.ghTier;
    return clonedSemanticRule;
  }

  //TODO: brad:I don't think this is valid any more
  @Deprecated
  public int getBodyArgs() {
    String[] args = body.split("\\,");
    return args.length;
  }
  //Construct new semantic rule from string from in .dict file
  SemanticRule(String definition) {
    log.trace("new semantic rule: " + definition);
    this.definition=definition.trim();
    args = new TreeMap<>();
    unsetArgs = new LinkedList<>();

    //the 2 here allows for there to be .'s in the body of the wildcard
    String[] pieces = definition.split("\\.",2);


    if (definition.startsWith("\"") && definition.endsWith("\"") ||  pieces.length == 1) {
      body = definition.trim();
      argsSet = true;
    } else if (pieces.length == 2) {

      body = pieces[1];//.replaceAll("\\s+", "");
      log.debug("pieces on now: " + body);
      String[] arguments = pieces[0].split("#");

      for (String argument : arguments) {
        String curArg = argument.trim();
        if (!curArg.equals("")) {

          String uniqueArg = "$" + curArg + UUID.randomUUID();
          unsetArgs.add(uniqueArg);
          log.trace("adding arg: "+uniqueArg);
          body = body.replaceAll("\\$" + curArg, Matcher.quoteReplacement(uniqueArg));
          log.trace("updated body: " + body);
        }
      }
    } else {
      log.error("badly formed definition: " + definition);
    }
  }

  private SemanticRule() {
    definition = "";
    body = "";
    args = new TreeMap<>();
    unsetArgs = new LinkedList<>();
    ghTier = "";
    argsSet = true;
  }

  public static SemanticRule generateHardcodedSemanticRule(String body) {
    SemanticRule s = new SemanticRule(body);
    s.argsSet = false;
    return s;
  }

  //TODO:brad: should this be public?
  public static SemanticRule generateLiteralSemanticRule(String body, String semanticType) {
    SemanticRule s = new SemanticRule();
    body=body.trim();

    //TODO:brad: what if we put everything in quotes? would that affect anything?
    Pattern alphaNumericPattern = Pattern.compile("\\w+");
    Matcher matcher = alphaNumericPattern.matcher(body);
    if (!matcher.matches()) {
      body="\"" + body + "\"";
    }
   if(!semanticType.isEmpty()) {
     s.body = body + ":" + semanticType;
   }else{
     s.body=body;
   }
    return s;
  }

  SemanticRule bindWildcard(String wildcardValue){

    Pattern alphaNumericPattern = Pattern.compile("\\w+");
    Matcher matcher = alphaNumericPattern.matcher(wildcardValue);
    if (!matcher.matches()) {
      wildcardValue="\"" + wildcardValue + "\"";
    }

    String newDefinition= this.definition;
    if(newDefinition.contains("*")){
      newDefinition=newDefinition.replace("*",wildcardValue);
      return new SemanticRule(newDefinition);
    }else{
      log.error("trying to bind a wildcard to a semantic rule that doesn't have one? definition:"+definition+" wildcard: "+wildcardValue);
      //TODO:brad: should this be null instead?
      return this;
    }
  }

  //constructs a new semantic rule with given functor name and number of arguments,
  //TODO: make the learning constructors appropriately general
  //used during definition learning/inference process
  SemanticRule(String functor, int airty, String rr) {
    ghTier = rr;
    args = new TreeMap<>();
    unsetArgs = new LinkedList<>();
    body = functor + "(";
    //create unique variables for number of arguments
    //65 is index of 'A'
    for (int key = 65; key < 65 + airty; key++) {
      String varName = "$" + (char) key + this.hashCode();
      unsetArgs.add(varName);
    }
    if (airty > 26) log.warn("more arguments than there are characters, weird stuff might happen");
    for (String arg : unsetArgs) {
      body += arg + ",";
    }
    if (unsetArgs.size() > 0) {
      body = body.substring(0, body.length() - 1);
    }
    body += ")";
  }

  //for implied subject cases, also used in learning
  SemanticRule(String functor, String type, String rr) {
    log.trace("new semantic rule: " + functor);
    args = new TreeMap<>();
    unsetArgs = new LinkedList<>();
    argsSet = true;

    body = functor.trim();
    //I'm pretty sure you can only have an implied subject in imperativess,
    //but I haven't thought about all of the question cases...
    if (type.equals("C")) {
      body += "()";
    }
    ghTier = rr;
  }

  void setGHTier(String cognitiveStatus) {
    ghTier = cognitiveStatus;
  }

  //perform lambda calculus application, used to generate structured semantic representation
  public SemanticRule apply(SemanticRule arg) {

    SemanticRule result = null;
    SemanticRule argClone = null;
    try {
      result = this.clone();
      argClone = arg.clone();
    } catch (Exception e) {
      log.error("exception cloning in apply.", e);
    }
    if ( result ==null || argClone == null) {
      log.error("semantic rule couldn't be cloned, returning null");
    } else if (result.unsetArgs.isEmpty()) {
      log.error("no remaining args cannot apply: " + argClone.getSemantics() + " to: " + result.getSemantics());
      result = null;
    } else {
      log.trace("setting arg " + result.unsetArgs.size());
      String toBeApplied = result.unsetArgs.remove();
      log.trace("to be applied " + toBeApplied);

      result.updateArgs(toBeApplied, argClone);
      result.unsetArgs.addAll(argClone.unsetArgs);
      result.argsSet = (result.unsetArgs.size() == 0) && (argClone.unsetArgs.size() == 0);
      log.trace(" Semantics after apply: " + result.getSemantics() + " unset args: " + result.unsetArgs);
    }

    return result;
  }

  //helper method to update argument values in lambda expressions
  private void updateArgs(String argName, SemanticRule rule) {
    if (!updateArgsInChildren(this,argName,rule)) {
      args.put(argName, rule);
    }
  }

  private boolean updateArgsInChildren(SemanticRule parent, String argName, SemanticRule rule){
    log.trace("[updateArgsHelper] "+argName+", "+rule);
    boolean found= false;
    for(Map.Entry<String,SemanticRule> arg: parent.args.entrySet()){
      SemanticRule argRule= arg.getValue();
      if(!argRule.args.isEmpty()){
        found = found || updateArgsInChildren(argRule,argName,rule);
      }
      if(!found && argRule.unsetArgs.contains(argName)){
        log.trace("[updateArgsInChildren] found arg in child! "+argName+" "+argRule);
        argRule.args.put(argName,rule);
        argRule.unsetArgs.remove(argName);
        found= true;
      }
    }
    return found;
  }

  //produces string form of nested/lambda calculus semantics
  //pretty much only used in logging
  public String getSemantics() {
    String returnValue = body;
    for (String arg : args.keySet()) {
      returnValue = returnValue.replace(arg, args.get(arg).getSemantics());
    }
    return returnValue;
  }

  public String getDefinition() {
    return definition;
  }

  /**
   * Generate semantic form that is actually used by the rest of the system
   * This is pretty messy, because the semantics we use are pretty messy...
   *
   * @return Returns list of Symbols whose first entry is in the form of the semantics field of an Utterance,
   * and the remaining members of the list are in the form of the supplementalSemantics field of an Utterance
   * as well as a map of Variables to GH tiers which are put into the tierAssignments field of an Utterance
   * basically this recursively traverses the parse tree and gets the values of the lambda expressions,
   * but because of how we handle semantics there is a lot of condition checking...
   */
  Pair<List<Symbol>, Map<Variable, Symbol>> getDIARCSemantics() {
    log.trace("getting DIARC Semantics for: " + body + " args: " + args);

    List<Symbol> semBindings = new ArrayList<>();
    Map<Variable, Symbol> ghBindings = new LinkedHashMap<>();

    //base cases
    if (args.isEmpty()) {
      if (ghTier.equals("VAR")) {

        String functorName=body;
        String type="";
        //TODO:brad:revisit this, this is a mechanism to allow parse rules for descriptors with arguments that have semantic types, as well as descriptors whose names contain ":"
        if(body.contains(":") && !(body.startsWith("\"") && body.endsWith("\""))){
          int i = body.lastIndexOf(":");
          functorName=body.substring(0, i);
          type=body.substring(i+1);
        }
        //make new term and free variable if its a new VAR
        Variable newThing = Parse.getNextVar(type);
        Term newSemBinding = new Term(functorName, newThing);
        log.trace("new var and bindings created: " + newThing + " " + newSemBinding);
        semBindings.add(newThing);
        semBindings.add(newSemBinding);
      } else if (ghTier.equals("MOD")) {
        //Catch badly formed definitions I guess, it seems like we would probably want to do this earlier...
        log.error("something that doesn't have arguments can't be MOD: " + body);
        return null;
      } else if (!ghTier.isEmpty()) {
        //for cases where there is a ghTier we need a new free variable and to pass on the ghTier
        //eg: "give me that"
        String functorName=body;
        String type="";
        //TODO:brad:is this case a hack? should this only happen in the VAR case?
        if(body.contains(":") && !(body.startsWith("\"") && body.endsWith("\""))){
          int i = body.lastIndexOf(":");
          functorName=body.substring(0, i);
          type=body.substring(i+1);
        }
        Variable newVar = Parse.getNextVar(type);
        Term newSemBinding = new Term(functorName, newVar);
//        Term newSemBinding = new Term(body, newVar);
        semBindings.add(newVar);
        semBindings.add(newSemBinding);
        ghBindings.put(newVar, Factory.createSymbol(ghTier));
        log.trace("new gh bindings for var: " + newVar + " " + newSemBinding + " " + ghBindings.get(newVar));
      } else if (body.endsWith("()") || body.endsWith("(?ADDRESSEE)")) {
        //special case for nullary predicates
        semBindings.add(Factory.createPredicate(body));
        log.trace("new empty predicate created: " + semBindings.get(semBindings.size() - 1));
      } else if (!argsSet) {
        semBindings.add(Factory.createPredicate(body));
        log.trace("created predicate with hardcoded args: " + semBindings.get(semBindings.size() - 1));
      }
      //otherwise, just pass on a symbol
      else {
        log.trace("body "+body);
        //TODO:brad: are there cases where making a predicate is bad?
        Symbol newSym = Factory.createFOL(body);
        semBindings.add(newSym);
        log.trace("new constant symbol created: " + newSym);
      }
    }
      //recursive cases
    else {
      String returnValue = body;
      SortedMap<String, Symbol> predArgs = new TreeMap<>();
      List<Symbol> additionalBindings = new ArrayList<>();
      log.trace("getting semantics for args");

      //update bindings for each argument
      for (String arg : args.keySet()) {
        log.trace("arg: " + arg + " semantics " + args.get(arg).getSemantics());
        Pair<List<Symbol>, Map<Variable, Symbol>> argBindings = args.get(arg).getDIARCSemantics();
        predArgs.put(arg, argBindings.getLeft().remove(0));
        additionalBindings.addAll(argBindings.getLeft());
        ghBindings.putAll(argBindings.getRight());
      }

      //this is like this so that the lambda function args don't need to align with predicate args
      for (Map.Entry<String,Symbol> a : predArgs.entrySet()) {
        returnValue = returnValue.replace(a.getKey(), a.getValue().toString());
      }
      log.debug("[getDIARCsemantics] return value: "+returnValue);

      //in the case of adjectives/ adjectival prepositions with multiple args
      //we need to do some shenanigans with the form that is stored in the main verb
      //ultimately we don't really have a good solution for this, without a richer semantic representation
      if (ghTier.equals("MOD")) {
        additionalBindings.add(Factory.createPredicate(returnValue));
        returnValue = predArgs.get(predArgs.firstKey()).toString();
      }

      //appropriately generate predicate for main verb/ main semantic form
      log.trace("returnValue: "+returnValue);
      Symbol mainSemantics=Factory.createFOL(returnValue);
      semBindings.add(mainSemantics);
      semBindings.addAll(additionalBindings);

      //pass along GH tier info
      if (!ghTier.isEmpty() && !ghTier.equals("MOD")) {
        //TODO:Brad: confirm that this is general enough
        if(mainSemantics.isTerm()){
          ghBindings.put(new Variable(predArgs.get(predArgs.firstKey()).getName()), Factory.createSymbol(ghTier));
        }else {
          ghBindings.put(new Variable(returnValue), Factory.createSymbol(ghTier));
        }
      }

    }
    log.trace("semBindings: " + semBindings);
    log.trace("ghBindings: " + ghBindings);
    if(!semBindings.isEmpty()) {

      semBindings.set(0, flattenAnd(semBindings.get(0)));
    }
    return Pair.of(semBindings, ghBindings);

  }

  private static Symbol flattenAnd(Symbol semantics) {

    if (semantics.isTerm() && semantics.getName().equals("and")) {
      Term t = (Term) semantics;
      List<Term> toReplace = new ArrayList<>();
      for (Symbol arg : t.getArgs()) {
        if (arg.isTerm() && arg.getName().equals("and")) {
          toReplace.add((Term)arg);
        }
      }
      if (toReplace.isEmpty()) {
        return semantics;
      } else {
        List<Symbol> newArgs = t.getArgs();
        newArgs.removeAll(toReplace);
        for (Term arg : toReplace) {
          newArgs.addAll(0,arg.getArgs().stream().map(SemanticRule::flattenAnd).collect(Collectors.toList()));
        }
        return Factory.createPredicate(t.getName(), newArgs);
      }
    } else {
      return semantics;
    }
  }

  public boolean isComplete() {
    return unsetArgs.size() == 0;
  }

  @Override
  public String toString() {
    return body+" "+args+" "+unsetArgs;
  }
}

//This class represents a parse rule
//It has 3 (or maybe 4 but cognitive status is part of the semantic rule class) parts:
//-morpheme (string representation of the word)
//-syntactic rule
//-semantic rule
//--cognitive status info (this is a combination of semantic and syntactic info and probably shouldn't be its own thing) it is used by reference resolution
//It also is used during the parsing process to represent state in the parse tree
//so accordingly, Entries can be created from the definitions in the dictionary well as the combination of other Entries
public class Entry implements Cloneable {
  //static private Template template = new Template();
  private static Logger log = LoggerFactory.getLogger(Entry.class);
  String morpheme;
  SyntacticRule syntax;
  SemanticRule semantics;

  //used to construct empty version of the object and have the rest of it filled out
  //maybe should be a builder instead?
  public Entry() {
    morpheme = "";
    syntax = null;
    semantics = null;
  }

  //TODO:brad, do we need this, and the other constructor?
  public Entry(String name, String grammar, String meaning, String cognitiveStatus) {
    morpheme = name;
    syntax = new SyntacticRule(grammar);
    semantics = new SemanticRule(meaning);
    semantics.setGHTier(cognitiveStatus);
  }

  public Entry(String name, SyntacticRule grammar, SemanticRule meaning, String cognitiveStatus) {
    morpheme = name;
    try {
      syntax = grammar.clone();
    } catch (Exception e) {
      log.error("exception cloning syntax.", e);
    }
    try {
      semantics = meaning.clone();
      semantics.setGHTier(cognitiveStatus);
    } catch (Exception e) {
      log.error("exception cloning semantics.", e);
    }
  }

  public Entry clone() throws CloneNotSupportedException {
    Entry clonedEntry = (Entry) super.clone();
    clonedEntry.morpheme = this.morpheme;
    if(syntax == null){
      clonedEntry.syntax =  null;
    } else {
      clonedEntry.syntax = this.syntax.clone();
    }
    if(semantics==null){
      clonedEntry.semantics = null;
    }else{
      clonedEntry.semantics = this.semantics.clone();
    }
    return clonedEntry;
  }

  public Entry generateBoundWildcard(String wildcardValue){
    Entry bound= null;
    try {
      bound = this.clone();
      bound.morpheme = bound.morpheme.replace("*",wildcardValue);
      bound.semantics = bound.getSemantics().bindWildcard(wildcardValue);

    } catch (CloneNotSupportedException e) {
      log.error("exception cloning an entry, I dont this this can happen?",e);
    }
    return bound;
  }

  @Override
  public String toString() {
    return "morpheme: " + morpheme + " | sem: " + semantics.toString() + " | syn: " + syntax.toString()+ " | gh: "+semantics.ghTier;
  }

  public SemanticRule getSemantics() {
    return semantics;
  }

  public String getSyntax() {
    return syntax.toString();
  }

  public String getMorpheme() {
    return morpheme;
  }

  String getCognitiveStatus() {
    return semantics.ghTier;
  }

  @Deprecated
  public int numArgs() {
    return semantics.getBodyArgs();
  }

  //This maybe should be a utility method or something
  //generate syntactic rules for all possible combinations of args
  //used in the new rule inference process
  static List<SyntacticRule> generateSyntaxes(List<List<SyntacticRule>> preceding, List<List<SyntacticRule>> following) {
    List<SyntacticRule> syntaxes = new ArrayList<>();
    List<List<SyntacticRule>> precedingCombinations = new ArrayList<>();
    List<SyntacticRule> tempListPreceding = new ArrayList<>();
    GeneratePermutations(preceding, precedingCombinations, 0, tempListPreceding);

    List<List<SyntacticRule>> followingCombinations = new ArrayList<>();
    List<SyntacticRule> tempListFollowing = new ArrayList<>();
    GeneratePermutations(following, followingCombinations, 0, tempListFollowing);

    for (List<SyntacticRule> pArgs : precedingCombinations) {
      for (List<SyntacticRule> fArgs : followingCombinations) {
        //TODO: make this not always a command, but how do we tell if it's a statement or question verb?
        SyntacticRule returnType;
        log.trace("pArgs: " + pArgs);
        if (pArgs.size() == 1 && pArgs.get(0).type.equals("SUBJAGENT")) {
          //If i is the subject, then it can't be a command
          log.debug("inferred returnType: S");
          returnType = new SyntacticRule("S");
        } else {
          returnType = new SyntacticRule("C");
          log.debug("inferred returnType: C");
        }

        List<SyntacticRule> fArgsRev = fArgs.subList(0, fArgs.size());
        Collections.reverse(fArgsRev);
        SyntacticRule syntax = new SyntacticRule(returnType, pArgs, fArgsRev);

        syntaxes.add(syntax);
      }
    }
    return syntaxes;
  }

  //helper method for above
  private static void GeneratePermutations(List<List<SyntacticRule>> Lists, List<List<SyntacticRule>> result, int depth, List<SyntacticRule> current) {
    //if depth is 0, who cares, why do we need this?
    if (depth == Lists.size()) {
      List<SyntacticRule> copy = new ArrayList<>(current.size());
      for (SyntacticRule item : current) {
        log.debug("copy " + item.type);
        copy.add(item);
      }
      result.add(copy);

      return;
    }

    for (int i = 0; i < Lists.get(depth).size(); ++i) {
      current.add(Lists.get(depth).get(i));
      GeneratePermutations(Lists, result, depth + 1, current);
      current.remove(current.size() - 1);
    }
  }
}

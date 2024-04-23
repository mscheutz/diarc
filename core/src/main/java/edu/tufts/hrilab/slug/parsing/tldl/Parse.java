/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.tldl;

import edu.tufts.hrilab.fol.Variable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

import static edu.tufts.hrilab.slug.parsing.tldl.Entry.generateSyntaxes;

public class Parse {

  //Used to generate unique free variable IDs within a parse
  private class VariableFactory {
    int counter;
    VariableFactory() {
      counter = 0;
    }

    Variable getNewVariable(String type) {
      Variable newVar = new Variable("VAR" + counter,type);
      counter += 1;
      return newVar;
    }
  }

  private static VariableFactory variableFactory;

  private Set<String> terminalCategories;

  private Logger log = LoggerFactory.getLogger(this.getClass());

  //The nodes occupying the parse space
  //the top of the stack is the end of the utterance
  private Stack<Node> inFocus;

  public Parse(Set<String> categories) {
    variableFactory = new VariableFactory();
    inFocus = new Stack<>();
    terminalCategories = categories;
  }

  static Variable getNextVar() {
    return variableFactory.getNewVariable("");
  }

  static Variable getNextVar(String type) {
    return variableFactory.getNewVariable(type);
  }

  public void setTerminalCategories(Set<String> categories) {
    terminalCategories = categories;
  }

  public Node getCurrentParse() {
    return inFocus.peek();
  }

  //Update the parse space with a new Node
  public boolean addNode(Node n) {
    boolean retVal = true;
    log.debug("***adding node: " + n.label);
    //if tree is empty
    if (inFocus.isEmpty()) {
      inFocus.push(n);
      log.debug("inFocus: " + inFocus.peek().label);
      return retVal;
    }
    log.debug("inFocus: " + inFocus.peek().label);

    Node combination = inFocus.peek().combine(n);

    if (combination != null) {
      log.debug("combination size " + combination.rules.size());
    } else {
      log.debug("No valid combinations");
    }

    if (combination != null) {
      inFocus.pop();
      log.debug("root morpheme (for combining): " + combination.label);
      addNode(combination);
    } else{
      inFocus.push(n);
      log.debug("inFocus: " + inFocus.peek().label);
    }
    return retVal;
  }

  //checks if any of the current parses are a terminal case, but doesn't care which one
  public boolean checkTerminal() {
    if (!inFocus.isEmpty()) {
      Node focus = inFocus.peek();
      for (NodeEntry r : focus.rules) {
        log.debug(r.syntax.toString() + " " + r.semantics.body + ", " + r.semantics.unsetArgs);
        if (r.syntax != null  && terminalCategories.contains(r.syntax.type) && r.semantics.isComplete()) {
          return true;
        }
      }
    }
    return false;
  }

//  public void pruneNonTerminal(){
//    if (!inFocus.isEmpty()) {
//      Node focus = inFocus.peek();
//      for (NodeEntry r : focus.rules) {
//        log.debug(r.syntax.toString() + " " + r.semantics.body + ", " + r.semantics.unsetArgs);
//        if (r.syntax != null  && terminalCategories.contains(r.syntax.type) && r.semantics.isComplete()) {
//        //keep
//        } else{
//          //remove
//        }
//      }
//    }
//
//  }


  //generate new definitions for a novel morpheme based on the current parse context,
  // and then add them to the current parse
  List<Entry> addUnknown(String id) {
    log.debug("adding new word " + id);
    log.debug("infocus size " + inFocus.size());

    List<Entry> newEntries = new ArrayList<>();
    if (!inFocus.isEmpty() && inFocus.peek() != null) {
      //make multiple rules based on number of args in the syntax
      List<SyntacticRule> possibleSyntaxes = new ArrayList<>();
      for (NodeEntry e : inFocus.peek().rules) {
        if (e.syntax.argType != null) {
          possibleSyntaxes.add(e.syntax.argType);
        }
      }
      if (possibleSyntaxes.size() > 0) {
        //list to hold lists of syntactic rules that share arity
        List<List<SyntacticRule>> argSets = new ArrayList<>();
        //seed it with the first rule
        List<SyntacticRule> seed = new ArrayList<>();
        seed.add(possibleSyntaxes.get(0));
        argSets.add(seed);
        possibleSyntaxes.remove(0);
        for (SyntacticRule r : possibleSyntaxes) {
          boolean added = false;
          for (List<SyntacticRule> arity : argSets) {
            if (r.depth == arity.get(0).depth) {
              arity.add(r);
              added = true;
            }
          }
          if (!added) {
            List<SyntacticRule> newArity = new ArrayList<>();
            newArity.add(r);
            argSets.add(newArity);
          }
        }

        for (List<SyntacticRule> arities : argSets) {
          for (SyntacticRule s : arities) {
            Entry newEntry = new Entry();
            newEntry.morpheme = id;
            newEntry.syntax = s;
            newEntry.semantics = new SemanticRule(id, s.type, "");

            //Check to see if the inferreddefinitionn matches part of any
            // template in the dictionary, if so add the new word as an
            // instance of that template instead.
            List<Entry> templateEntries = TLDLParserComponent.dictionary.getMatchingEntries(newEntry);
            if (templateEntries.size() > 0) {
              TLDLParserComponent.dictionary.addEntries(templateEntries);
              newEntries.addAll(templateEntries);
            } else {
              newEntries.add(newEntry);
            }
          }
        }
      } else {
        log.debug("no valid categories from focus");
        Entry newEntry = new Entry();
        newEntry.morpheme = id;
        newEntries.add(newEntry);
      }
    } else {
      //if there is nothing in focus, e.g. the first word of the utterance,
      //we generate a place holder, and comeback to it later via inferUnknowns
      //realistically this is probably how it should be done every time
      log.debug("no categories from focus");
      Entry newEntry = new Entry();
      newEntry.morpheme = id;
      newEntries.add(newEntry);
    }

    Node newNode = new Node(newEntries);
    addNode(newNode);

    return newEntries;
  }

  //this should only really be called if infocus.peek() represents a single word
  //and the next thing in the stack is a terminal case
  // TODO: fix this it's horrible design
  Node popPreviousParse() {
    //get rid of most recently added node
    log.debug("[popPreviousParse] getting previous parse: " + inFocus);
    Node temp = inFocus.pop();
    log.debug("[popPreviousParse] inFocus: " + temp.label);
    Node previousParse = inFocus.pop();
    log.debug("[popPreviousParse] previous parse: " + previousParse.label);
    inFocus.push(temp);
    log.debug("[popPreviousParse] previous parse:" + previousParse.label);
    return previousParse;
  }

  //verifies that the most recent addition can't be combined with the previous sentence
  boolean verifyParseCompletion() {
    //check if we're in an applicable situation
    if (inFocus.size() > 1) {

      //if the previous Node is a terminal case
      Node temp = inFocus.pop();
      Node previousParse = inFocus.peek();
      log.debug("[verifyParseCompletion] temp: "+temp+" previousParse: "+previousParse);
      inFocus.push(temp);

      boolean previousIsTerminal = false;
      for (NodeEntry r : previousParse.rules) {
        if (r.syntax != null && terminalCategories.contains(r.syntax.type) && r.semantics.isComplete()) {
          previousIsTerminal = true;
          break;
        }
      }
      log.trace("previousIsTerminal: "+previousIsTerminal);
      boolean allComplete = true;

      //check that all of the semantics of the previous node are complete
      for (NodeEntry ne : previousParse.rules) {
        if (ne.semantics != null && !ne.semantics.isComplete()) {
          allComplete = false;
          break;
        }
      }
      log.trace("allComplete: "+allComplete);

      if (previousIsTerminal && allComplete) {
        //check if current focus is derived from previous focus
        if (inFocus.peek().lAncestor != null && !inFocus.peek().lAncestor.equals(previousParse)) {
          //if it isn't then the previous sentence has been completed
          return true;
        } else return inFocus.peek().rAncestor != null && !inFocus.peek().rAncestor.equals(previousParse);
      }
    }
    return false;
  }

  //check to see if all nodes in the parse space have definitions
  boolean checkAllDefinitionsExist() {
    for (Node n : inFocus) {
      if (n.rules.size() == 0) return false;
    }
    return true;
  }

  //infer the definitions of nodes that do not have them
  List<Entry> inferUnknowns() {
    //look for nodes that have rules with size 0
    log.debug("infer unknowns start");
    List<Entry> updatedEntries = new ArrayList<>();
    for (Node n : inFocus) {
      //we only want to learn new definitions for nodes that don't have them
      if (n.rules.size() == 0) {
        log.debug("inferring definition for: " + n.label);
        //count the number of nodes before and after
        List<Node> before = inFocus.subList(0, inFocus.indexOf(n));
        List<List<SyntacticRule>> precedingArgs = new ArrayList<>();
        for (Node b : before) {
          List<SyntacticRule> curNodeArgs = new ArrayList<>();
          for (NodeEntry e : b.rules) {
            curNodeArgs.add(e.syntax);
          }
          precedingArgs.add(curNodeArgs);
        }

        log.debug("preceding args size: " + precedingArgs.size());

        List<Node> after = inFocus.subList(inFocus.indexOf(n) + 1, inFocus.size());
        List<List<SyntacticRule>> followingArgs = new ArrayList<>();
        for (Node a : after) {
          List<SyntacticRule> curNodeArgs = new ArrayList<>();
          for (NodeEntry e : a.rules) {
            curNodeArgs.add(e.syntax);
            log.debug("morpheme " + e.morpheme);
            log.debug("type " + e.syntax.type);
          }
          followingArgs.add(curNodeArgs);
        }
        log.debug("following args size: " + followingArgs.size());
        log.debug("preceding args size: " + precedingArgs.size());

        List<Entry> newEntries = new ArrayList<>();
        //generate new semantic rule based on morpheme, there is only one of these
        //even in cases of multiple syntaxes
        SemanticRule semantics = new SemanticRule(n.label, precedingArgs.size() + followingArgs.size(), "");

        //generate new syntaxes from the set of preceding and/or following syntaxes
        List<SyntacticRule> newSyntaxes = generateSyntaxes(precedingArgs, followingArgs);
        for (SyntacticRule syntax : newSyntaxes) {
          //TODO:brad: can this ever be something that has a cognitive status?
          Entry newEntry = new Entry(n.label,syntax,semantics,"");
          newEntries.add(newEntry);
          n.addRule(newEntry);
        }

        List<Entry> templateEntries =new ArrayList<>();

        //perform template look up and if any templates match add the template versions,
        //otherwise add the inferred version.
        for(Entry e: newEntries) {
          templateEntries.addAll(TLDLParserComponent.dictionary.getMatchingEntries(e));
        }
        if (templateEntries.size() > 0) {
          TLDLParserComponent.dictionary.addEntries(templateEntries);
          updatedEntries.addAll(templateEntries);
        } else {
          updatedEntries.addAll(newEntries);
        }

      }
    }
    //rebuild the tree with new info
    Stack<Node> temp = new Stack<>();
    while (!inFocus.empty()) {
      temp.push(inFocus.pop());
    }

    while (!temp.empty()) {
      addNode(temp.pop());
    }
    log.debug("inFocus size after inference: " + inFocus.size());
    log.debug("new length " + updatedEntries.size());
    return updatedEntries;
  }

  //TODO:brad: implement something like this if we want to be able to revise rules that have been learned on the fly after they've been learned the firs time.
//  //checks to see if any of the nodes in the tree have been learned this utterance
//  public boolean checkForNewRules() {
//    for (Node n : inFocus) {
//      if (checkChildrenLearned(n)) return true;
//    }
//    return false;
//  }
//
//  public boolean checkChildrenLearned(Node n) {
//    if (n == null) {
//      return false;
//    } else if (n.learned) {
//      return true;
//    } else if (checkChildrenLearned(n.lAncestor)) {
//      return true;
//    } else if (checkChildrenLearned(n.rAncestor)) {
//      return true;
//    } else {
//      return false;
//    }
//  }
//  void updateNewRules() {
//    log.debug("updating newly learned rules");
//
//    // for all "complete trees "
//    // find the right most node, if it's learned then we're good for now
//    // check that there is a subsequen tnode that isnt complete;
//
//    //Obtaining list iterator
//    ListIterator<Node> nodeIter =inFocus.listIterator();
//
//    while(nodeIter.hasNext()){
//      Node current =nodeIter.next();
//      Node next = nodeIter.next();
//      if(next == null) break;
//
//      if(current.)
//
//    }
////    while(litr.hasPrevious()){
////      System.out.println(litr.previous());
////    }
//
//
//      if(checkForNewRules() && )
//
//
//    //if adjacent is complete then add that as an arg
//    //if it isn't see if it can combine in the right direction
//    //otherwise there's still a problem
//  }

  //check to see if the last node that is in focus accepts preceding args, if so we might be able to construct a parse that gets everything to work
  boolean checkComplete() {
    if(!inFocus.isEmpty()) {
    for (NodeEntry r : inFocus.peek().rules) {
        if (r.syntax != null && r.syntax.backwardApply) return true;
      }
    }
    return false;
  }

  //undo existing combinations until there is a node that can be combined with current focus
  //once one matches recombine from there
  void reviseParse() {
    log.debug("trying to revise parse");
    Node current = inFocus.pop();
    log.debug("current = "+current.getSemantics());
    //get valid arg types
    List<SyntacticRule> validArgTypes = new ArrayList<>();

    for (NodeEntry ne : current.rules) {
      if (ne.syntax != null) {
        validArgTypes.add(ne.syntax);
      }
    }

    log.debug("current = "+current.getSemantics());
    boolean matchFound = false;
    while (!inFocus.isEmpty() &&inFocus.peek() != null && !matchFound && inFocus.peek().lAncestor != null && inFocus.peek().rAncestor != null) {
      log.debug("current = "+current.getSemantics());
      //remove node and add its children to in focus
      Node curTop = inFocus.pop();
      log.debug("curTop = "+curTop.getSemantics());
      inFocus.push(curTop.lAncestor);
      inFocus.push(curTop.rAncestor);

      for (NodeEntry ne : inFocus.peek().rules) {

        for (SyntacticRule s : validArgTypes) {
          if (s.checkArgsApply(ne.syntax)) {
            matchFound = true;
          }
        }
      }
    }

    if (matchFound) {
      addNode(current);
    }
  }

//  boolean isWildcard(){
//    //TODO:brad this is another area where the whole NodeEntry thing is a problem, but I'm not gonna fix it now...
//    if(!inFocus.isEmpty() && !inFocus.peek().rules.isEmpty() && inFocus.peek().rules.get(0).syntax.argType != null) {
//      return inFocus.peek().rules.get(0).syntax.argType.isWildcard;
//    }
//    return false;
//  }

  boolean isLiteral(){
    //TODO:brad this is another area where the whole NodeEntry thing is a problem, but I'm not gonna fix it now...
    if(!inFocus.isEmpty() && !inFocus.peek().rules.isEmpty() && inFocus.peek().rules.get(0).syntax.argType != null) {
      return inFocus.peek().rules.get(0).syntax.argType.isLiteral;
    }
    return false;
  }

  Entry generateLiteralEntry(String morpheme){
    log.debug("generating wild card entry for: "+morpheme);
    //For now the convention is to have the syntatic type before the star
    log.debug("rules "+inFocus.peek().rules);
    String argtype=inFocus.peek().rules.get(0).syntax.argType.type;
    //String newType = argtype.substring(0,argtype.indexOf("*"));
    String newType = argtype;

    //TODO:brad: it might be nice to be able to provide semantic type info here some how at some point
    return new Entry(morpheme,new SyntacticRule(newType),SemanticRule.generateLiteralSemanticRule(morpheme,""),"");
  }

}

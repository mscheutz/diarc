/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.tldl.parse;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

//This class represents a node in the parse tree for a single rule.
//The whole abstraction here is confusing and this and the Node class shouldn't co exist.
//Anyway, for now this class extends entry, adding pointers to the nodes which were used to create it,
//as well as wrappers for the calls to the apply (and compose if it existed) functions of the
//underlying syntactic and semantic rule classes.

public class NodeEntry extends Entry implements Cloneable {
  private static Logger log = LoggerFactory.getLogger(NodeEntry.class);

  private NodeEntry functionAncestor;
  private NodeEntry argumentAncestor;

  public NodeEntry clone() throws CloneNotSupportedException {
    NodeEntry clonedNodeEntry = (NodeEntry) super.clone();
    //TODO:brad should these be cloned? do we want separate tree histories?
    if (this.functionAncestor == null) {
      clonedNodeEntry.functionAncestor = null;
    } else {
      clonedNodeEntry.functionAncestor = this.functionAncestor.clone();
    }
    if (this.argumentAncestor == null) {
      clonedNodeEntry.argumentAncestor = null;
    } else {
      clonedNodeEntry.argumentAncestor = this.argumentAncestor.clone();
    }
    return clonedNodeEntry;
  }

  NodeEntry(Entry m, NodeEntry fa, NodeEntry aa) {
    //TODO:brad: should these be clones?
    morpheme = m.morpheme;
    try {
      if (m.syntacticRule == null) {
        syntacticRule = null;
      } else {
        syntacticRule = m.syntacticRule.clone();
      }
    } catch (Exception e) {
      log.error("Exception cloning syntacticRule." + e);
    }
    try {
      if (m.semanticRule == null) {
        semanticRule = null;
      } else {
        semanticRule = m.semanticRule.clone();
      }
    } catch (Exception e) {
      log.error("Exception cloning semanticRule." + e);
    }
    functionAncestor = fa;
    argumentAncestor = aa;
  }

  public NodeEntry combine(NodeEntry adjacent) {
    Entry combined = new Entry();

    log.debug("in combine current: " + morpheme + " " + syntacticRule + " adjacent: " + adjacent.morpheme + " " + adjacent.syntacticRule);

    //check for valid forward application
    if (syntacticRule != null && adjacent.syntacticRule != null && syntacticRule.forwardApply) {
      log.trace("trying forward apply");
      //build new node from application
      combined.syntacticRule = syntacticRule.apply(adjacent.syntacticRule);
      if (combined.syntacticRule == null) {
        log.trace("syntacticRule is null");
      }

      if (semanticRule == null) {
        log.trace("semanticRule is null");
      }
      if (adjacent.semanticRule == null) {
        log.trace("adjacent semanticRule is null");
      }
      combined.semanticRule = semanticRule.apply(adjacent.semanticRule);
      combined.morpheme = morpheme + " " + adjacent.morpheme;

      if (combined.syntacticRule != null && combined.semanticRule != null) {
        log.trace("new syntacticRule: " + combined.syntacticRule.getType());
        log.trace("new semanticRule: " + combined.semanticRule);
        log.trace("new morpheme " + combined.morpheme);
        return new NodeEntry(combined, this, adjacent);
      }
    }
    //TODO:brad: reimplement this
//    else if (syntacticRule.forwardApply && syntacticRule.checkArgsCompose(adjacent.syntacticRule)) {
//      log.debug("forward compose");
////      combined.syntacticRule = new SyntacticRule(syntacticRule.returnType, adjacent.syntacticRule.argType, adjacent.syntacticRule.forwardApply);
//      combined.syntacticRule = syntacticRule.compose(adjacent.syntacticRule);
//      log.trace("new syntacticRule: "+combined.syntacticRule.type);
//      combined.semanticRule = semanticRule;
//      combined.semanticRule.apply(adjacent.semanticRule);
//      combined.morpheme = morpheme + adjacent.morpheme;
//
//      return combined;
//    }
    //check for valid backward application
    //todo:brad:made this not else, hopefully it doesn't break too much...
    if (syntacticRule != null && adjacent.syntacticRule != null && adjacent.syntacticRule.backwardApply) {
      log.trace("trying backward apply");
      //build new node from application
      combined.syntacticRule = adjacent.syntacticRule.apply(syntacticRule);
      if (combined.syntacticRule == null) {
        return null;
      }
      log.trace("new syntacticRule: " + combined.syntacticRule.getType() + " " + combined.syntacticRule.forwardApply);
      combined.semanticRule = adjacent.semanticRule.apply(semanticRule);
      combined.morpheme = morpheme + " " + adjacent.morpheme;
      //log.info("new morpheme: " + combined.morpheme);
      if (combined.syntacticRule != null && combined.semanticRule != null) {
        return new NodeEntry(combined, adjacent, this);
      }
    }
    //TODO:brad: reimplement this
//    else if (adjacent.syntacticRule.backwardApply && adjacent.syntacticRule.checkArgsCompose(syntacticRule)) {
//      log.debug("backward compose");
////      combined.syntacticRule = new SyntacticRule(adjacent.syntacticRule.returnType.returnType, syntacticRule.argType.returnType, syntacticRule.returnType.forwardApply);
//      combined.syntacticRule= adjacent.syntacticRule.compose(syntacticRule);
//      log.trace("new syntacticRule: "+combined.syntacticRule.type+" "+combined.syntacticRule.forwardApply);
//      combined.semanticRule = adjacent.semanticRule;
//      combined.semanticRule.apply(semanticRule);
//      combined.morpheme = morpheme + adjacent.morpheme;
//      return combined;
//    }
//    else {
    log.debug("No valid entry combinations found, returning NULL");
    return null;
//    }

  }

}

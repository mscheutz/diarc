/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.tldl;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

//This class represents a node in the parse tree for a single rule.
//The whole abstraction here is confusing and this and the Node class shouldn't co exist.
//Anyway, for now this class extends entry, adding pointers to the nodes which were used to create it,
//as well as wrappers for the calls to the apply (and compose if it existed) functions of the
//underlying syntactic and semantic rule classes.

public class NodeEntry extends Entry implements Cloneable  {
  private static Logger log = LoggerFactory.getLogger(NodeEntry.class);

  private NodeEntry functionAncestor;
  private NodeEntry argumentAncestor;

  public NodeEntry clone() throws CloneNotSupportedException {
    NodeEntry clonedNodeEntry = (NodeEntry) super.clone();
    //TODO:brad should these be cloned? do we want separate tree histories?
    if(this.functionAncestor == null){
      clonedNodeEntry.functionAncestor=null;
    }else {
      clonedNodeEntry.functionAncestor = this.functionAncestor.clone();
    }
    if(this.argumentAncestor == null){
      clonedNodeEntry.argumentAncestor=null;
    }else {
      clonedNodeEntry.argumentAncestor = this.argumentAncestor.clone();
    }
    return clonedNodeEntry;
  }

  NodeEntry(Entry m, NodeEntry fa, NodeEntry aa) {
    //TODO:brad: should these be clones?
    morpheme = m.morpheme;
    try {
      if(m.syntax == null) {
        syntax = null;
      }else{
          syntax = m.syntax.clone();
      }
    }catch (Exception e){
      log.error("Exception cloning syntax." +e);
    }
    try{
      if(m.semantics == null){
        semantics=null;
      }else {
        semantics = m.semantics.clone();
      }
    }catch (Exception e){
      log.error("Exception cloning semantics." +e);
    }
    functionAncestor = fa;
    argumentAncestor = aa;
  }

  public NodeEntry combine(NodeEntry adjacent) {
    Entry combined = new Entry();

    log.debug("in combine current: " + morpheme +" "+syntax+ " adjacent: " + adjacent.morpheme+" "+adjacent.syntax);

    //check for valid forward application
    if (syntax != null && adjacent.syntax != null && syntax.forwardApply) {
      log.trace("trying forward apply");
      //build new node from application
      combined.syntax = syntax.apply(adjacent.syntax);
      if(combined.syntax == null){
        log.trace("syntax is null");
      }

      if (semantics == null) {
        log.trace("semantics is null");
      }
      if (adjacent.semantics == null) {
        log.trace("adjacent semantics is null");
      }
      combined.semantics= semantics.apply(adjacent.semantics);
      combined.morpheme = morpheme + " " + adjacent.morpheme;

      if(combined.syntax != null && combined.semantics != null) {
        log.trace("new syntax: " + combined.syntax.type);
        log.trace("new semantics: " + combined.semantics);
        log.trace("new morpheme " + combined.morpheme);
        return new NodeEntry(combined, this, adjacent);
      }
    }
    //TODO:brad: reimplement this
//    else if (syntax.forwardApply && syntax.checkArgsCompose(adjacent.syntax)) {
//      log.debug("forward compose");
////      combined.syntax = new SyntacticRule(syntax.returnType, adjacent.syntax.argType, adjacent.syntax.forwardApply);
//      combined.syntax = syntax.compose(adjacent.syntax);
//      log.trace("new syntax: "+combined.syntax.type);
//      combined.semantics = semantics;
//      combined.semantics.apply(adjacent.semantics);
//      combined.morpheme = morpheme + adjacent.morpheme;
//
//      return combined;
//    }
    //check for valid backward application
    //todo:brad:made this not else, hopefully it doesn't break too much...
    if (syntax !=null && adjacent.syntax != null && adjacent.syntax.backwardApply) {
      log.trace("trying backward apply");
      //build new node from application
      combined.syntax = adjacent.syntax.apply(syntax);
      if (combined.syntax== null ) return null;
      log.trace("new syntax: " + combined.syntax.type + " " + combined.syntax.forwardApply);
      combined.semantics= adjacent.semantics.apply(semantics);
      combined.morpheme = morpheme + " " + adjacent.morpheme;
      //log.info("new morpheme: " + combined.morpheme);
      if(combined.syntax != null && combined.semantics != null) {
        return new NodeEntry(combined, adjacent, this);
      }
    }
    //TODO:brad: reimplement this
//    else if (adjacent.syntax.backwardApply && adjacent.syntax.checkArgsCompose(syntax)) {
//      log.debug("backward compose");
////      combined.syntax = new SyntacticRule(adjacent.syntax.returnType.returnType, syntax.argType.returnType, syntax.returnType.forwardApply);
//      combined.syntax= adjacent.syntax.compose(syntax);
//      log.trace("new syntax: "+combined.syntax.type+" "+combined.syntax.forwardApply);
//      combined.semantics = adjacent.semantics;
//      combined.semantics.apply(semantics);
//      combined.morpheme = morpheme + adjacent.morpheme;
//      return combined;
//    }
//    else {
      log.debug("No valid entry combinations found, returning NULL");
      return null;
//    }

  }

}
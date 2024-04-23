/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.tldl;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

import static java.util.Objects.isNull;  

//This class is the highest level unit of a parse tree.
//Because i didn't know what I was doing when I wrote this class,
// it contains a set of possible meanings for this point in the parse,
// but that doesn't really make sense, because it only has a single pair of ancestors.
//Really all of the possible meanings of a parse should be kept in separate nodes in separate trees
//so that different possible meanings can be associated with different possible parse structures,
//and then we wouldn't need Parse.reviseParse/
public class Node {
  private static Logger log = LoggerFactory.getLogger(Node.class);

  String label;
  List<NodeEntry> rules;
  //assuming we read left to right
  Node lAncestor;
  Node rAncestor;

  boolean learned = false;
  //may or may not need

  //Construct a node form a list of entries returned by the dictionary
  public Node(List<Entry> entries) {
    label = entries.get(0).morpheme;
    rules = new ArrayList<>();
    for (Entry e : entries) {
      if(e.syntax != null) rules.add(new NodeEntry(e,null,null));
    }
    learned = true;
    lAncestor = null;
    rAncestor = null;
  }

 //constructor used to create a new Node during the combination process, when NodeEntries already exist.
  private Node(Node left, Node right, List<NodeEntry> validGrammar) {
    label = left.label + " " + right.label;
    rules = validGrammar;
    lAncestor = left;
    rAncestor = right;
  }

  //Combine Node with another Node in the parse space, assumed to occur after it in the utterance?
  public Node combine(Node partner) {
    ArrayList<NodeEntry> validCombinations = new ArrayList<>();
    for (NodeEntry current : this.rules) {
      for (NodeEntry next : partner.rules) {
        log.debug("checking possible combination: \""+current+"\" \""+next+"\"");
        //cloning here so multiple trees can be built
        NodeEntry currentClone=null;
        NodeEntry nextClone=null;
        try{
          if(current !=  null) {
            currentClone=current.clone();
          }
          if(next != null) {
            nextClone = next.clone();
          }
        }catch (Exception e){
          log.error("Exception in cloning in combine. "+e);
        }

        NodeEntry combination =null;
        if(currentClone != null){
          combination = currentClone.combine(nextClone);
          //if null, nextClone.combine(currentCLone);
        }
        if (combination != null) validCombinations.add(combination);
        log.trace("valid combinations size" + validCombinations.size());
      }
    }

    if (!validCombinations.isEmpty()) {
      log.debug("Combining " + this.label + " and " + partner.label);
      return new Node(this, partner, validCombinations);
    } else {
      return null;
    }
  }

  public String getLabel() {
    return label;
  }

  public List<String> getSemantics() {
    List<String> possibleSemantics = new ArrayList<>();
    for (NodeEntry e : rules) {
      possibleSemantics.add(e.semantics.getSemantics());
    }
    return possibleSemantics;
  }

  void addRule(Entry e) {
    rules.add(new NodeEntry(e, null, null));
  }

}

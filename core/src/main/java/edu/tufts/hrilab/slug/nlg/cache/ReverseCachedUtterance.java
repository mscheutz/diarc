package edu.tufts.hrilab.slug.nlg.cache;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.slug.cache.CachedUtterance;
import edu.tufts.hrilab.slug.common.UtteranceType;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class ReverseCachedUtterance extends CachedUtterance {
  private String language;

  public ReverseCachedUtterance(UtteranceType type, Symbol semantics, List<Term> indirectSemantics, List<Map<Variable, Symbol>> bindings, List<Term> supplementalSemantics, Map<Variable, Symbol> tierAssignments, String language) {
    super(new ArrayList<>(), type, semantics, indirectSemantics, bindings, supplementalSemantics, tierAssignments);

    if (bindings == null || bindings.isEmpty()) {
      this.bindings = null;
    } else {
      this.bindings = new ArrayList<>();
    }
    this.language = language;
  }
}

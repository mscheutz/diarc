package edu.tufts.hrilab.slug.nlg.cache;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.slug.parsing.cache.CachedUtterance;

import java.util.List;
import java.util.Map;

public class ReverseCachedUtterance extends CachedUtterance {
    private String language;

    public ReverseCachedUtterance (List<String> words, UtteranceType type, Symbol semantics, List<Term> indirectSemantics, List<Map<Variable, Symbol>> bindings, List<Term> supplementalSemantics, Map<Variable,Symbol> tierAssignments, String language) {
        super(words,type,semantics,indirectSemantics,bindings,supplementalSemantics,tierAssignments);
        this.language = language;
    }
}

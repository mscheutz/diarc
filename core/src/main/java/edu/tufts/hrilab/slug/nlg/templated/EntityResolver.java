package edu.tufts.hrilab.slug.nlg.templated;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class EntityResolver {
    private final Map<String, String> aliases = new HashMap<>();

    public EntityResolver alias(String semanticId, String displayName) {
        aliases.put(semanticId, displayName);
        return this;
    }

    public String resolve(Term term) {
        if (!(term instanceof Atom atom)) return term.renderSource();
        String raw = atom.value();
        if (atom.quoted()) return raw;
        String aliased = aliases.get(raw);
        if (aliased != null) return aliased;
        //TODO: configure specific types for which to attempt generateRE for?
        if (raw.contains(":")) {
            //TODO: decide whether to remove aliases or not (or populate aliases through RR)
            Map<Symbol, List<edu.tufts.hrilab.fol.Term>> refs;
            Symbol arg = Factory.createSymbol(raw);
            try {
                refs = TRADE.getAvailableService(new TRADEServiceConstraints().name("generateRE").argTypes(Symbol.class)).call(Map.class, arg);
                if (refs.get(arg).size() == 1) {
                    raw = refs.get(arg).get(0).getName();
                }
            } catch (TRADEException ignored) {
            }
        }

        int typeIndex = raw.lastIndexOf(':');
        if (typeIndex > 0) raw = raw.substring(0, typeIndex);
        return humanize(raw);
    }

    public String quotedLabel(Term term) {
        String value = resolve(term);
        return "\u201c" + value + "\u201d";
    }

    public static String humanize(String raw) {
        if (raw == null || raw.isBlank()) return "";
        String s = raw.replace('_', ' ').trim();
        s = s.replaceAll("(?<=[a-z0-9])(?=[A-Z])", " ");
        return s.toLowerCase();
    }
}

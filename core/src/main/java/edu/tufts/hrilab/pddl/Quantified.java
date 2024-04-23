/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.pddl;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;
import java.util.StringJoiner;

public class Quantified {
    private final List<Symbol> head;
    private final List<Predicate> body;
    private final QuantifiedType type; //forall or exists
    private static final Logger log = LoggerFactory.getLogger(Quantified.class);

    public enum QuantifiedType {
        EXISTS,
        FORALL;
    }

    Quantified(QuantifiedType type, List<Symbol> head, List<Predicate> body) {
        this.head = head;
        this.body = body;
        this.type = type;
    }

    // Turns the object into a PDDL formatted string
    String generate() {
        //Actions
        StringBuilder prefix = new StringBuilder();
        prefix.append("(").append(this.type).append(" (");
        for(Symbol s : this.head) {
            prefix.append(Generator.generateTyped(s)).append(" ");
        }
        prefix.append(")\n");

        StringJoiner action = new StringJoiner("\n", prefix, "\n)");

        //effects
        if (body.size() > 0) {
            StringJoiner effectString;
            String effectPrefix, effectSuffix;
            effectPrefix = "\t(and\n";
            effectSuffix = "\n\t)";

            effectString = new StringJoiner("\n", effectPrefix, effectSuffix);
            for (Predicate predicate : body) {
                effectString.add("\t\t" + Generator.generate(predicate));
            }
            action.add(effectString.toString());
        } else {
            return "";
        }

        return action.toString();
    }


}

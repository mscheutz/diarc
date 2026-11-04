/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.pddl;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;
import java.util.Set;
import java.util.StringJoiner;

public class Disjunctive {
    private final Set<Predicate> body;
    private static final Logger log = LoggerFactory.getLogger(Disjunctive.class);

    Disjunctive(Set<Predicate> body) {
        this.body = body;
    }

    // Turns the object into a PDDL formatted string
    String generate() {
        //effects
        if (body.size() > 0) {
            String prefix, suffix;
            prefix = "\t(or\n";
            suffix = "\n\t)";
            StringJoiner disjunctiveString = new StringJoiner("\n", prefix, suffix);
            for (Predicate predicate : body) {
                disjunctiveString.add("\t\t" + Generator.generate(predicate));
            }
            return disjunctiveString.toString();
        } else {
            return "";
        }
    }
}

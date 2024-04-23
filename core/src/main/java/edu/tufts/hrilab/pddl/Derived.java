/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.pddl;

import edu.tufts.hrilab.fol.Predicate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;
import java.util.StringJoiner;

public class Derived {
    private final Predicate head;
    private final List<Predicate> body;
    private static final Logger log = LoggerFactory.getLogger(Derived.class);

    public Derived(Predicate head, List<Predicate> body) {
        this.head = head;
        this.body = body;
    }

    // Turns the object into a PDDL formatted string
    String generate() {
        //Actions
        StringBuilder prefix = new StringBuilder();
        prefix.append("(:derived ").append(Generator.generateTyped(this.head)).append("\n");


        StringJoiner action = new StringJoiner("\n", prefix, "\n)");

        //effects
        if (body.size() > 0) {
            StringJoiner effectString;
            String effectPrefix, effectSuffix;
            effectPrefix = "\t\t(and\n";
            effectSuffix = "\n\t\t)";

            effectString = new StringJoiner("\n", effectPrefix, effectSuffix);
            for (Predicate predicate : body) {
                effectString.add("\t\t\t" + Generator.generate(predicate));
            }
            action.add(effectString.toString());
        } else {
            return "";
        }

        return action.toString();
    }


}

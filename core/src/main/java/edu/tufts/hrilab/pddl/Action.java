/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.pddl;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;
import java.util.StringJoiner;

public class Action {
    private static final Logger log = LoggerFactory.getLogger(Action.class);
    private final String name;
    private final boolean isEvent;
    private final LinkedHashMap<String, Variable> parameters; // order matters!!
    private final List<Predicate> preconditions;
    private final List<Disjunctive> disjunctivePreconditions;
    private final List<Quantified> quantifiedPreconditions;
    private final List<Quantified> quantifiedEffects;
    private final List<Predicate> effects;
    private final List<Variable> aslParameters;

    Action(Builder builder) {
        this.name = builder.name;
        this.isEvent = builder.isEvent;
        this.parameters = new LinkedHashMap<>(builder.parameters);
        this.preconditions = new ArrayList<>(builder.preconditions);
        this.quantifiedPreconditions = new ArrayList<>(builder.quantifiedPreconditions);
        this.quantifiedEffects = new ArrayList<>(builder.quantifiedEffects);
        this.disjunctivePreconditions = new ArrayList<>(builder.disjunctivePreconditions);
        this.effects = new ArrayList<>(builder.effects);

        // if ASL action has (implicitly) existentially quantified pre-conditions (i.e., contains free-variables that
        // aren't input parameters), and the free-variables in those pre-condition are not already input parameters,
        // add them as explicit input parameters in the PDDL action, so they can be used in post-conditions
        this.aslParameters = new ArrayList<>(builder.parameters.values());
        for (Predicate condition : preconditions) {
            for (Variable var : condition.getVars()) {
                if (!builder.parameters.containsKey(var.getName())) {
                    this.parameters.put(var.getName(), var);
                }
            }
        }
    }

    public boolean isEvent() {
        return isEvent;
    }

    // Turns the object into a PDDL formatted string
    String generate() {
        if (isEvent) {
            return generate("event");
        } else {
            return generate("action");
        }
    }

    private String generate(String pddlType) {
        //Actions
        StringBuilder prefix = new StringBuilder();
        prefix.append("(:").append(pddlType).append(" ").append(name).append("\n");
        StringJoiner action = new StringJoiner("\n", prefix, "\n)");

        //Parameters
        StringJoiner paramString = new StringJoiner(" ", "\t:parameters (", ")\n");
        if (parameters.size() > 0) {
            for (Symbol parameter : parameters.values()) {
                paramString.add(Generator.generateTyped(parameter));
            }
            action.add(paramString.toString());
        } else {
            return "";
        }

        //preconditions
        int numPreconditions = preconditions.size() + quantifiedPreconditions.size() + disjunctivePreconditions.size();
        if (numPreconditions > 0) {
            StringJoiner precondString;
            String precondPrefix, precondSuffix;
            if (numPreconditions == 1) {
                precondPrefix = "\t:precondition\n";
                precondSuffix = "\n";
            } else {
                precondPrefix = "\t:precondition (and\n";
                precondSuffix = "\n\t)";
            }

            precondString = new StringJoiner("\n", precondPrefix, precondSuffix);
            for (Predicate precondition : preconditions) {
                precondString.add("\t\t" + Generator.generate(precondition));
            }
            for (Quantified q : quantifiedPreconditions) {
                precondString.add("\t\t" + q.generate());
            }
            for (Disjunctive d : disjunctivePreconditions) {
                precondString.add("\t\t" + d.generate());
            }
            action.add(precondString.toString());
        }

        //effects
        int numEffects = effects.size() + quantifiedEffects.size();
        if (numEffects > 0) {
            StringJoiner effectString;
            String effectPrefix, effectSuffix;
            if (numEffects == 1) {
                effectPrefix = "\t:effect\n";
                effectSuffix = "\n";
            } else {
                effectPrefix = "\t:effect (and\n";
                effectSuffix = "\n\t)";
            }

            effectString = new StringJoiner("\n", effectPrefix, effectSuffix);
            for (Predicate effect : effects) {
                effectString.add("\t\t" + Generator.generate(effect));
            }
            for (Quantified q : quantifiedEffects) {
                effectString.add("\t\t" + q.generate());
            }
            action.add(effectString.toString());
        } else {
            return "";
        }

        return action.toString();
    }


    public List<Variable> getParameters() {
        return new ArrayList<>(parameters.values());
    }

    public List<Predicate> getPreconditions() {
        return preconditions;
    }

    public List<Predicate> getEffects() {
        return effects;
    }

    public String getName() {
        return name;
    }

    public Predicate getPddlSignature() {
        return Factory.createPredicate(name, new ArrayList<>(parameters.values()));
    }

    public Predicate getAslSignature() {
        return Factory.createPredicate(name, aslParameters);
    }

    public static class Builder {
        private final String name;
        private boolean isEvent = false;
        private final LinkedHashMap<String, Variable> parameters = new LinkedHashMap<>();
        private final List<Predicate> preconditions = new ArrayList<>();
        private final List<Predicate> effects = new ArrayList<>();
        private final List<Quantified> quantifiedPreconditions = new ArrayList<>();
        private final List<Quantified> quantifiedEffects = new ArrayList<>();
        private final List<Disjunctive> disjunctivePreconditions = new ArrayList<>();

        public Builder(String name) {
            this.name = name;
        }

        public void setIsEvent(boolean flag) {
            this.isEvent = flag;
        }

        public Builder addParameter(String varName, String typeName) {
            if (!varName.startsWith("?")) {
                log.warn("[addParameter] variable does not start with a ?: " + varName);
                // TODO: EAK: why does this ever happen?
                //  if this is for testing code, the testing code should be changed,
                //  source code should never be changed to accommodate testing code
                varName = "?" + varName;
            }
            Variable parameter = new Variable(varName, typeName);
            this.parameters.put(varName, parameter);
            return this;
        }

        public boolean isValidParameter(String varName) {
            return parameters.containsKey(varName);
        }

        public Builder addPrecondition(Predicate preCondition) {
            preconditions.add(preCondition);
            return this;
        }

        public Builder addDisjunctivePrecondition(Set<Predicate> preCondition) {
            disjunctivePreconditions.add(new Disjunctive(preCondition));
            return this;
        }

        public Builder addPrecondition(Set<Predicate> preCondition) {
            if (preCondition.isEmpty()) {
                log.warn("Attempting to add empty precondition.");
            } else if (preCondition.size() == 1) {
                addPrecondition(preCondition.iterator().next());
            } else { // disjunctive precondition
                return addDisjunctivePrecondition(preCondition);
            }
            return this;
        }

        public Builder addQuantifiedPrecondition(Quantified.QuantifiedType type, Predicate precondition) {
            List<Symbol> head = new ArrayList<>();
            List<Predicate> body = new ArrayList<>();
            body.add(Factory.createPredicate(precondition.toString().replace("!", "?")));
            for (Symbol a : precondition.getArgs()) {
                if (a.toString().startsWith("!")) {
                    head.add(Factory.createVariable(a.toString().replace("!", "?")));
                }
            }
            this.quantifiedPreconditions.add(new Quantified(type, head, body));
            return this;
        }

        public Builder addQuantifiedPrecondition(Quantified.QuantifiedType type, Set<Predicate> precondition) {
            if (precondition.isEmpty()) {
                log.warn("Attempting to add empty precondition.");
            } else {
                //get the local arg
                for (Predicate p : precondition) {
                    addQuantifiedPrecondition(type, p);
                }
            }
            return this;
        }

        public Builder addQuantifiedEffect(Predicate effect) {
            List<Symbol> head = new ArrayList<>();
            List<Predicate> body = new ArrayList<>();
            body.add(Factory.createPredicate(effect.toString().replace("!", "?")));
            for (Symbol a : effect.getArgs()) {
                if (a.toString().startsWith("!")) {
                    head.add(Factory.createVariable(a.toString().replace("!", "?")));
                }
            }
            this.quantifiedEffects.add(new Quantified(Quantified.QuantifiedType.FORALL, head, body));
            return this;
        }

        public Builder addQuantifiedEffect(Set<Predicate> effect) {
            if (effect.isEmpty()) {
                log.warn("Attempting to add empty precondition.");
            } else {
                //get the local arg
                for (Predicate p : effect) {
                    addQuantifiedEffect(p);
                }
            }
            return this;
        }

        public Builder addEffect(Predicate effect) {
            effects.add(effect);
            return this;
        }

        public Builder addEffect(String name, boolean negated, String... argNames) {
            Predicate predicate = Factory.createPredicate(name, argNames);

            if (negated) {
                effects.add(predicate.toNegatedForm());
            } else {
                effects.add(predicate);
            }
            return this;
        }

        public Action build() {
            Action action = new Action(this);
            validateAction(action);
            return action;
        }

        private void validateAction(Action action) {
            //todo
        }
    }
}

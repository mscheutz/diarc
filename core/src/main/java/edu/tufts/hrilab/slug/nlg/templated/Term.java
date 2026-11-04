package edu.tufts.hrilab.slug.nlg.templated;

import java.util.List;

public sealed interface Term permits Atom, Call {
    String renderSource();
}

record Atom(String value, boolean quoted) implements Term {
    @Override public String renderSource() {
        return quoted ? '"' + value.replace("\"", "\\\"") + '"' : value;
    }
}

record Call(String name, List<Term> args) implements Term {
    public Call {
        args = List.copyOf(args);
    }

    @Override public String renderSource() {
        return name + "(" + args.stream().map(Term::renderSource).reduce((a,b) -> a + "," + b).orElse("") + ")";
    }
}

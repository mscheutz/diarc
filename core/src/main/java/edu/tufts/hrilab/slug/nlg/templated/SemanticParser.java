package edu.tufts.hrilab.slug.nlg.templated;

import java.util.ArrayList;
import java.util.List;

public final class SemanticParser {
    private final String input;
    private int pos;

    private SemanticParser(String input) {
        this.input = input;
    }

    public static Term parse(String input) {
        SemanticParser p = new SemanticParser(input);
        Term term = p.parseTerm();
        p.skipWs();
        if (!p.eof()) throw p.error("Unexpected trailing input");
        return term;
    }

    private Term parseTerm() {
        skipWs();
        if (eof()) throw error("Expected term");
        if (peek() == '"') return new Atom(parseQuoted(), true);

        int start = pos;
        while (!eof() && peek() != '(' && peek() != ',' && peek() != ')') pos++;
        String head = input.substring(start, pos).trim();
        if (head.isEmpty()) throw error("Expected token");

        if (!eof() && peek() == '(') {
            pos++;
            List<Term> args = new ArrayList<>();
            skipWs();
            if (!eof() && peek() != ')') {
                while (true) {
                    args.add(parseTerm());
                    skipWs();
                    if (!eof() && peek() == ',') {
                        pos++;
                        continue;
                    }
                    break;
                }
            }
            expect(')');
            return new Call(head, args);
        }
        return new Atom(head, false);
    }

    private String parseQuoted() {
        expect('"');
        StringBuilder out = new StringBuilder();
        while (!eof()) {
            char c = input.charAt(pos++);
            if (c == '"') return out.toString();
            if (c == '\\' && !eof()) {
                char escaped = input.charAt(pos++);
                out.append(switch (escaped) {
                    case 'n' -> '\n';
                    case 't' -> '\t';
                    case 'r' -> '\r';
                    case '"' -> '"';
                    case '\\' -> '\\';
                    default -> escaped;
                });
            } else {
                out.append(c);
            }
        }
        throw error("Unterminated quoted string");
    }

    private void expect(char expected) {
        skipWs();
        if (eof() || peek() != expected) throw error("Expected '" + expected + "'");
        pos++;
    }

    private void skipWs() {
        while (!eof() && Character.isWhitespace(peek())) pos++;
    }

    private char peek() { return input.charAt(pos); }
    private boolean eof() { return pos >= input.length(); }
    private IllegalArgumentException error(String message) {
        return new IllegalArgumentException(message + " at position " + pos + " in: " + input);
    }
}

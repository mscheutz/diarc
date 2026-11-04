package edu.tufts.hrilab.slug.nlg.templated;

import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

final class English {
    private English() {}

    static String join(List<String> items) {
        List<String> clean = items.stream().filter(s -> s != null && !s.isBlank()).toList();
        if (clean.isEmpty()) return "";
        if (clean.size() == 1) return clean.get(0);
        if (clean.size() == 2) return clean.get(0) + " and " + clean.get(1);
        return String.join(", ", clean.subList(0, clean.size() - 1)) + ", and " + clean.get(clean.size() - 1);
    }

    static String unit(long n, String unit) {
        String normalized = unit.toLowerCase(Locale.ROOT);
        if (normalized.endsWith("s")) normalized = normalized.substring(0, normalized.length() - 1);
        return n + " " + normalized + (n == 1 ? "" : "s");
    }

    static String every(long n, String unit) {
        String normalized = unit.toLowerCase(Locale.ROOT);
        if (normalized.endsWith("s")) normalized = normalized.substring(0, normalized.length() - 1);
        return n == 1 ? "every " + normalized : "every " + n + " " + normalized + "s";
    }

    static String clock(String semanticTime) {
        String[] parts = semanticTime.split("-");
        if (parts.length < 2) return semanticTime;
        int hour = Integer.parseInt(parts[0]);
        int minute = Integer.parseInt(parts[1]);
        LocalTime t = LocalTime.of(hour, minute);
        DateTimeFormatter f = minute == 0
                ? DateTimeFormatter.ofPattern("h a", Locale.US)
                : DateTimeFormatter.ofPattern("h:mm a", Locale.US);
        return t.format(f).toLowerCase(Locale.US).replace("am", "a.m.").replace("pm", "p.m.");
    }

    static List<Term> flattenAnd(Term term) {
        List<Term> out = new ArrayList<>();
        flatten(term, out);
        return out;
    }

    private static void flatten(Term term, List<Term> out) {
        if (term instanceof Call c && c.name().equals("and")) {
            c.args().forEach(t -> flatten(t, out));
        } else {
            out.add(term);
        }
    }

    static String capitalize(String s) {
        if (s == null || s.isBlank()) return s;
        return Character.toUpperCase(s.charAt(0)) + s.substring(1);
    }
}

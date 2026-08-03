package edu.tufts.hrilab.slug.nlg.templated;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;

public final class TemplatedNLG {
    private final EntityResolver entities;
    private final RealizationOptions options;
    private final Map<String, Function<Call, String>> lexicalizers = new LinkedHashMap<>();
    private String variationKey = "";

    public TemplatedNLG(EntityResolver entities) {
        this(entities, RealizationOptions.canonical());
    }

    public TemplatedNLG(EntityResolver entities, RealizationOptions options) {
        this.entities = Objects.requireNonNull(entities);
        this.options = Objects.requireNonNull(options);
        installDefaultLexicon();
    }

    public static TemplatedNLG defaults() {
        return new TemplatedNLG(new EntityResolver());
    }

    public static TemplatedNLG varied(EntityResolver resolver, long seed) {
        return new TemplatedNLG(resolver, RealizationOptions.varied(seed));
    }

    public TemplatedNLG lexicalizer(String predicate, Function<Call, String> fn) {
        lexicalizers.put(predicate, fn);
        return this;
    }

    public Utterance convertSemanticsToText(Utterance utterance) {
        Symbol semantics;
        List<edu.tufts.hrilab.fol.Term> boundSemantics = utterance.getBoundSemantics();
        if (boundSemantics.isEmpty()) {
            semantics = utterance.getSemantics();
        } else if (boundSemantics.size() == 1) {
            semantics = boundSemantics.get(0);
        } else {
            semantics = Factory.createPredicate("and", boundSemantics);
        }

        if (semantics != null) {
            String output = generate(semantics.toString());
            utterance.setWords(output);
        }
        return utterance;
    }

    public String generate(String semantics) {
        return generate(SemanticParser.parse(semantics));
    }

    public String generate(Term term) {
        variationKey = term.renderSource();
        if (term instanceof Call c && c.name().equals("mean")) return realizeValidation(c);
        return English.capitalize(realize(term, Form.CLAUSE)) + punctuation(term);
    }

    private String realizeValidation(Call mean) {
        requireArity(mean, 2);
        Term content = mean.args().get(1);
        String text;
        if (content instanceof Call c) {
            text = switch (c.name()) {
                case "startMission" -> realizeMission(c);
                case "submitTaskSequenceDefinition" -> realizeTaskDefinition(c);
                case "scheduled" -> realizeScheduledValidation(c);
                default -> choose("generic-confirm", "Confirming you want me to ", "Just to confirm, you want me to ", "Making sure you want me to ") + realize(c, Form.INFINITIVE);
            };
        } else {
            text = "Confirming you meant " + entities.resolve(content);
        }
        return ensureQuestion(text);
    }

    private String realizeMission(Call mission) {
        requireArity(mission, 2);
        List<Term> actions = English.flattenAnd(mission.args().get(1));
        List<String> realized = new ArrayList<>();

        for (int i = 0; i < actions.size(); i++) {
            Term current = actions.get(i);
            if (i + 1 < actions.size() && isMatchingNavigationPair(current, actions.get(i + 1))) {
                realized.add(realize(actions.get(i + 1), Form.GERUND));
                i++;
            } else {
                String phrase = realize(current, Form.GERUND);
                if (i == actions.size() - 1 && current instanceof Call c &&
                        (c.name().equals("goToLocationWrapper") || c.name().equals("goToLocation"))) {
                    phrase = phrase.replaceFirst("^going to ", "returning to ");
                }
                realized.add(phrase);
            }
        }
        realized = aggregateRepeatedRoomSearches(realized);
        return choose("mission-frame",
                "Confirming you want me to start the mission consisting of ",
                "Just to confirm, you want me to begin a mission that involves ",
                "Making sure you want me to start the mission by ") + joinSequence(realized);
    }

    private boolean isMatchingNavigationPair(Term a, Term b) {
        if (!(a instanceof Call nav) || !(b instanceof Call action)) return false;
        if (!(nav.name().equals("goToRoom") || nav.name().equals("goToLocation") || nav.name().equals("goToLocationWrapper"))) return false;
        if (nav.args().size() < 2 || action.args().size() < 2) return false;
        String destination = nav.args().get(1).renderSource();
        String actionLocation = action.args().get(1).renderSource();
        return destination.equals(actionLocation);
    }

    private List<String> aggregateRepeatedRoomSearches(List<String> phrases) {
        List<String> result = new ArrayList<>();
        List<String> rooms = new ArrayList<>();
        for (String phrase : phrases) {
            if (phrase.startsWith("searching ") && phrase.endsWith(" for people")) {
                rooms.add(phrase.substring("searching ".length(), phrase.length() - " for people".length()));
            } else {
                flushRoomSearches(result, rooms);
                result.add(phrase);
            }
        }
        flushRoomSearches(result, rooms);
        return result;
    }

    private void flushRoomSearches(List<String> result, List<String> rooms) {
        if (!rooms.isEmpty()) {
            result.add("searching " + English.join(rooms) + " for people");
            rooms.clear();
        }
    }

    private String realizeTaskDefinition(Call def) {
        requireArity(def, 3);
        String name = entities.quotedLabel(def.args().get(1));
        List<Term> pairs = topLevelAndArgs(def.args().get(2));

        List<LocationAction> parsed = pairs.stream().map(this::parseLocationAction).toList();
        boolean allSameAction = !parsed.isEmpty() && parsed.stream()
                .map(p -> p.action().renderSource())
                .distinct().count() == 1;

        String description;
        if (allSameAction) {
            String action = realize(parsed.get(0).action(), Form.INFINITIVE);
            List<String> locations = parsed.stream().map(LocationAction::location).toList();
            description = injectLocations(action, locations);
        } else {
            description = English.join(parsed.stream()
                    .map(p -> realize(p.action(), Form.INFINITIVE) + " at " + p.location())
                    .toList());
        }
        return choose("task-definition-frame",
                "Let me make sure I understood. You want to save a new task sequence definition called ",
                "Just to confirm, you want to save a new task sequence named ",
                "Checking my understanding: you want to create and save a task sequence called ") +
                name + choose("task-definition-link", " where you ", " in which you ", " that has me ") + description;
    }


    private List<Term> topLevelAndArgs(Term term) {
        if (term instanceof Call c && c.name().equals("and")) return c.args();
        return List.of(term);
    }

    private LocationAction parseLocationAction(Term term) {
        if (term instanceof Call c && c.name().equals("and") && c.args().size() == 2) {
            return new LocationAction(entities.resolve(c.args().get(0)), c.args().get(1));
        }
        throw new IllegalArgumentException("Expected location/action pair, got: " + term.renderSource());
    }

    private String injectLocations(String action, List<String> locations) {
        String locationPhrase = compressNumberedLocations(locations);
        if (action.startsWith("present a survey")) {
            int with = action.indexOf(" with ");
            if (with >= 0) return action.substring(0, with) + " at " + locationPhrase + action.substring(with);
            return action + " at " + locationPhrase;
        }
        return action + " at " + locationPhrase;
    }

    private String compressNumberedLocations(List<String> locations) {
        if (locations.size() >= 2) {
            String first = locations.get(0);
            int split = first.lastIndexOf(' ');
            if (split > 0) {
                String stem = first.substring(0, split);
                boolean sameStem = locations.stream().allMatch(x -> x.startsWith(stem + " "));
                boolean numeric = locations.stream().allMatch(x -> x.substring(stem.length() + 1).matches("\\d+"));
                if (sameStem && numeric) {
                    List<String> nums = locations.stream().map(x -> x.substring(stem.length() + 1)).toList();
                    return pluralize(stem) + " " + English.join(nums);
                }
            }
        }
        return English.join(locations);
    }

    private String pluralize(String noun) {
        if (noun.endsWith("s")) return noun;
        if (noun.endsWith("y") && noun.length() > 1) return noun.substring(0, noun.length() - 1) + "ies";
        return noun + "s";
    }

    private String realizeScheduledValidation(Call scheduled) {
        if (scheduled.args().size() < 2) throw new IllegalArgumentException("scheduled requires a task and timing");
        String task = realize(scheduled.args().get(0), Form.INFINITIVE);
        List<String> timing = new ArrayList<>();
        boolean hasPeriod = false;
        boolean hasEnd = false;

        for (int i = 1; i < scheduled.args().size(); i++) {
            Term t = scheduled.args().get(i);
            if (t instanceof Call c) {
                switch (c.name()) {
                    case "start" -> timing.add(realizeStart(c, scheduled.args().size() == 2));
                    case "period" -> { timing.add(realizePeriod(c)); hasPeriod = true; }
                    case "end" -> { timing.add(realizeEnd(c)); hasEnd = true; }
                    default -> timing.add(realize(c, Form.CLAUSE));
                }
            }
        }
        if (hasPeriod && !hasEnd) timing.add("until you tell me to stop");

        String suffix = timing.isEmpty() ? "" : ", " + joinTiming(timing);
        return choose("schedule-frame",
                "Making sure I should schedule the task to ",
                "Just to confirm, I should schedule a task to ",
                "Checking that you want me to schedule the task to ") + task + suffix;
    }

    private String realizeStart(Call c, boolean onlyTiming) {
        requireArity(c, 1, 2);
        if (c.args().size() == 2) {
            long n = number(c.args().get(0));
            String unit = atom(c.args().get(1));
            if (n == 0) return choose("start-now", "now", "starting now", "immediately");
            return onlyTiming
                    ? choose("single-relative-start", "in ", "after ") + English.unit(n, unit) + choose("from-now", " from now", "", "")
                    : choose("relative-start", "starting in ", "beginning in ", "with the first run in ") + English.unit(n, unit);
        }
        return choose("absolute-start", "starting at ", "beginning at ", "with the first run at ") + English.clock(atom(c.args().get(0)));
    }

    private String realizePeriod(Call c) {
        requireArity(c, 1, 2);
        if (c.args().size() == 2) return choose("period", "repeating ", "and recurring ", "running ") + English.every(number(c.args().get(0)), atom(c.args().get(1)));
        return choose("daily-period", "repeating every day at ", "and recurring daily at ", "running each day at ") + English.clock(atom(c.args().get(0)));
    }

    private String realizeEnd(Call c) {
        requireArity(c, 1, 2);
        if (c.args().size() == 2) return choose("relative-end", "until ", "ending ") + English.unit(number(c.args().get(0)), atom(c.args().get(1))) + " from now";
        return choose("absolute-end", "until ", "ending at ", "through ") + English.clock(atom(c.args().get(0)));
    }

    private String joinTiming(List<String> timing) {
        String joined = English.join(timing);
        // An end condition syntactically modifies the repetition clause.
        return joined.replace(", and until ", " until ")
                     .replace(" and until ", " until ");
    }

    private String realize(Term term, Form form) {
        if (term instanceof Atom a) return entities.resolve(a);
        Call c = (Call) term;
        if (c.name().equals("and")) return realizeAnd(c, form);
        String modifier = realizeGeneralTemplate(c, form);
        if (modifier != null) return modifier;
        Function<Call, String> lexicalizer = lexicalizers.get(c.name());
        if (lexicalizer != null) return adaptForm(lexicalizer.apply(c), form);
        return adaptForm(fallback(c), form);
    }


    /** Realizers for the document's General Templates / Modifiers section. */
    private String realizeGeneralTemplate(Call c, Form form) {
        return switch (c.name()) {
            case "not" -> realizeNot(c, form);
            case "error" -> realizeError(c);
            case "cannot" -> realizeCannot(c);
            case "because" -> realizeBecause(c);
            case "ack" -> realizeAcknowledgement(c);
            case "greeting" -> realizeGreeting(c);
            case "attempting" -> realizeAttempting(c, form);
            case "doNotKnowWhat" -> realizeUnknownWord(c);
            case "know" -> realizeKnow(c);
            case "howTo" -> realizeHowTo(c);
            case "urgentPriority" -> realizeUrgent(c, form);
            default -> null;
        };
    }

    private String realizeAnd(Call c, Form form) {
        List<String> parts = English.flattenAnd(c).stream().map(t -> realize(t, form)).toList();
        if (form == Form.CLAUSE && parts.size() == 2) {
            return parts.get(0) + ", and " + lowerInitial(parts.get(1));
        }
        return English.join(parts);
    }

    private String realizeNot(Call c, Form form) {
        requireArity(c, 1);
        Term inner = c.args().get(0);
        if (inner instanceof Atom atom) {
            if (atom.value().equals("localized")) return "I am not localized";
            if (atom.value().equals("detectDoor")) return "I did not see a door";
        }
        if (inner instanceof Call call) {
            if (call.name().equals("open") && call.args().size() == 1) {
                return entities.resolve(call.args().get(0)) + " is not open";
            }
            if (call.name().equals("localized") && call.args().isEmpty()) return "I am not localized";
            if (call.name().equals("detectDoor") && call.args().isEmpty()) return "I did not see a door";
            if (call.name().equals("know") && call.args().size() >= 2) return negateClause(realizeKnow(call));
        }
        String positive = realize(inner, Form.CLAUSE);
        if (positive.startsWith("I am ")) return "I am not " + positive.substring(5);
        if (positive.startsWith("I can ")) return "I cannot " + positive.substring(6);
        return form == Form.CLAUSE ? "it is not the case that " + lowerInitial(positive) : "not " + positive;
    }

    private String realizeError(Call c) {
        requireArity(c, 2);
        // The actor is perspective metadata. The error content carries the message.
        String content = realize(c.args().get(1), Form.CLAUSE);
        if (content.startsWith("it is not the case that ")) return content.substring("it is not the case that ".length());
        return content;
    }

    private String realizeCannot(Call c) {
        requireArity(c, 2);
        String goal = realize(c.args().get(0), Form.INFINITIVE);
        String reason = c.args().get(1) instanceof Call b && b.name().equals("because")
                ? realizeBecause(b) : realize(c.args().get(1), Form.CLAUSE);
        return "I cannot " + goal + " because " + stripBecause(reason);
    }

    private String realizeBecause(Call c) {
        requireArity(c, 1);
        return "because " + lowerInitial(realize(c.args().get(0), Form.CLAUSE));
    }

    private String realizeAcknowledgement(Call c) {
        requireArity(c, 1);
        Term content = c.args().get(0);
        if (content instanceof Call a && a.name().equals("attempting")) {
            return "Okay, " + lowerInitial(realizeAttempting(a, Form.CLAUSE)).replaceFirst("^I am attempting to ", "I will attempt to ");
        }
        return "Okay, " + lowerInitial(realize(content, Form.CLAUSE));
    }

    private String realizeGreeting(Call c) {
        requireArity(c, 3);
        String addressee = entities.resolve(c.args().get(1));
        return "Hello " + capitalizeName(addressee);
    }

    private String realizeAttempting(Call c, Form form) {
        requireArity(c, 2);
        String action = realize(c.args().get(1), Form.INFINITIVE);
        String phrase = "attempt to " + action;
        if (form == Form.GERUND) return "attempting to " + action;
        return "I am " + phrase.replaceFirst("^attempt ", "attempting ");
    }

    private String realizeUnknownWord(Call c) {
        requireArity(c, 3);
        String word = c.args().get(1) instanceof Atom a ? a.value() : entities.resolve(c.args().get(1));
        return "I don't know what “" + word + "” means in the current context";
    }

    private String realizeKnow(Call c) {
        if (c.args().size() < 2) throw new IllegalArgumentException("know requires actor and content");
        Term content = c.args().get(1);
        if (content instanceof Call h && h.name().equals("howTo")) return "I know " + realizeHowTo(h);
        return "I know " + lowerInitial(realize(content, Form.CLAUSE));
    }

    private String realizeHowTo(Call c) {
        requireArity(c, 1);
        return "how to " + realize(c.args().get(0), Form.INFINITIVE);
    }

    private String realizeUrgent(Call c, Form form) {
        requireArity(c, 1);
        String action = realize(c.args().get(0), Form.INFINITIVE);
        if (form == Form.GERUND) return adaptForm(action, Form.GERUND) + " immediately";
        if (action.startsWith("go to ")) return "I am going to " + action.substring(6) + " immediately";
        return "I am going to " + action + " immediately";
    }

    private String negateClause(String clause) {
        if (clause.startsWith("I know ")) return "I do not know " + clause.substring(7);
        if (clause.startsWith("I am ")) return "I am not " + clause.substring(5);
        return "I do not " + lowerInitial(clause);
    }

    private String stripBecause(String text) {
        return text.startsWith("because ") ? text.substring(8) : text;
    }

    private String lowerInitial(String s) {
        if (s == null || s.isBlank()) return s;
        if (s.startsWith("I ") || s.startsWith("I'")) return s;
        return Character.toLowerCase(s.charAt(0)) + s.substring(1);
    }

    private String definite(String nounPhrase) {
        if (nounPhrase == null || nounPhrase.isBlank()) return nounPhrase;
        String lower = nounPhrase.toLowerCase(java.util.Locale.ROOT);
        if (lower.startsWith("the ") || lower.startsWith("my ") || lower.startsWith("your ")) return nounPhrase;
        return "the " + nounPhrase;
    }

    private String capitalizeName(String s) {
        if (s == null || s.isBlank()) return s;
        return Character.toUpperCase(s.charAt(0)) + s.substring(1);
    }

    private String fallback(Call c) {
        String verb = EntityResolver.humanize(c.name());
        List<Term> semanticArgs = c.args().stream().filter(t -> !isActor(t)).toList();
        if (semanticArgs.isEmpty()) return verb;
        return verb + " " + English.join(semanticArgs.stream().map(t -> realize(t, Form.CLAUSE)).toList());
    }

    private void installDefaultLexicon() {
        lexicalizer("goToRoom", c -> "go to " + definite(entities.resolve(c.args().get(1))));
        lexicalizer("goToLocation", c -> "go to " + entities.resolve(c.args().get(1)));
        lexicalizer("goToLocationWrapper", c -> "go to " + entities.resolve(c.args().get(1)));
        lexicalizer("searchRoomForPeople", c -> "search " + definite(entities.resolve(c.args().get(1))) + " for people");
        lexicalizer("measureWifiAtLocations", c -> "measure Wi-Fi at " + realizeLocationList(c.args().get(1)));
        lexicalizer("measureTemperatureAtLocations", c -> "measure the temperature at " + realizeLocationList(c.args().get(1)));
        lexicalizer("presentSurvey", c -> "present a survey with the associated message " + quoteMessage(c.args().get(1)));
        lexicalizer("waitForDuration", c -> "wait for " + English.unit(number(c.args().get(1)), "seconds"));
        lexicalizer("scanForPeople", c -> "scan the area in front of me for people");
        lexicalizer("scanForDoor", c -> "scan the area in front of me for doors");
        lexicalizer("breakOpenDoor", c -> "open the door");
        lexicalizer("followUser", c -> "follow you");
        lexicalizer("stopFollow", c -> "stop following you");
        lexicalizer("goHome", c -> "go home to charge");
        lexicalizer("cancelCurrentTask", c -> "cancel my current task");
        lexicalizer("suspendCurrentTask", c -> "suspend my current task");
        lexicalizer("resumeSystemGoals", c -> "resume my paused task");
        lexicalizer("sit", c -> "sit");
        lexicalizer("stand", c -> "stand");
        lexicalizer("open", c -> "open " + entities.resolve(c.args().get(0)));
        lexicalizer("detectDoor", c -> "detect a door");
        lexicalizer("localized", c -> "be localized");
        lexicalizer("learnedTourSequence", c -> "I have learned the " + entities.resolve(c.args().get(0)) + " sequence");
        lexicalizer("canAskToPerformBySaying", c -> "you can ask me to perform it by saying “" + entities.resolve(c.args().get(0)) + " the " + entities.resolve(c.args().get(1)) + "”");
    }

    private String realizeLocationList(Term term) {
        return English.join(English.flattenAnd(term).stream().map(entities::resolve).toList());
    }

    private String quoteMessage(Term term) {
        String message = term instanceof Atom a ? a.value() : entities.resolve(term);
        message = message.strip();
        if (message.toLowerCase().startsWith("hello,")) message = message.substring(message.indexOf(',') + 1).strip();
        return "\u201c" + message + "\u201d";
    }

    private String adaptForm(String phrase, Form form) {
        if (form == Form.INFINITIVE || form == Form.CLAUSE) return phrase;
        if (form == Form.GERUND) {
            if (phrase.startsWith("go ")) return "going " + phrase.substring(3);
            if (phrase.startsWith("search ")) return "searching " + phrase.substring(7);
            if (phrase.startsWith("measure ")) return "measuring " + phrase.substring(8);
            if (phrase.startsWith("present ")) return "presenting " + phrase.substring(8);
            if (phrase.startsWith("wait ")) return "waiting " + phrase.substring(5);
            if (phrase.startsWith("scan ")) return "scanning " + phrase.substring(5);
            if (phrase.startsWith("open ")) return "opening " + phrase.substring(5);
            if (phrase.startsWith("follow ")) return "following " + phrase.substring(7);
            if (phrase.startsWith("return ")) return "returning " + phrase.substring(7);
        }
        return phrase;
    }

    private boolean isActor(Term t) {
        if (!(t instanceof Atom a)) return false;
        return switch (a.value()) {
            case "ava", "alex", "spot", "self" -> true;
            default -> false;
        };
    }

    private String joinSequence(List<String> steps) {
        if (steps.isEmpty()) return "no actions";
        if (steps.size() == 1) return steps.get(0);
        String middle = choose("sequence-middle", ", then ", ", followed by ", ", next ");
        String last = choose("sequence-last", ", and then ", ", before finally ", ", and finally ");
        if (steps.size() == 2) return steps.get(0) + last + steps.get(1);
        return String.join(middle, steps.subList(0, steps.size() - 1)) + last + steps.get(steps.size() - 1);
    }


    private String choose(String slot, String... alternatives) {
        if (!options.varied() || alternatives.length == 1) return alternatives[0];
        int hash = Objects.hash(options.seed(), variationKey, slot);
        return alternatives[Math.floorMod(hash, alternatives.length)];
    }

    private String punctuation(Term term) { return "."; }
    private String ensureQuestion(String text) { return text.endsWith("?") ? text : text + "?"; }
    private String atom(Term t) {
        if (t instanceof Atom a) return a.value();
        throw new IllegalArgumentException("Expected atom, got " + t.renderSource());
    }
    private long number(Term t) { return Long.parseLong(atom(t)); }

    private void requireArity(Call c, int... allowed) {
        for (int n : allowed) if (c.args().size() == n) return;
        throw new IllegalArgumentException(c.name() + " expected arity " + java.util.Arrays.toString(allowed) +
                " but got " + c.args().size());
    }

    private enum Form { CLAUSE, INFINITIVE, GERUND }
    private record LocationAction(String location, Term action) {}
}

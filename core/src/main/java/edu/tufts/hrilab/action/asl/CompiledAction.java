/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.asl;

import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.util.Utilities;
import edu.tufts.hrilab.fol.Symbol;
import java.util.List;
import java.util.Map;
import java.util.LinkedHashMap;
import java.util.Objects;

/** An immutable invocation-local ASL module. Its definitions never enter the global action DB. */
public final class CompiledAction {
  private final Map<String, ActionDBEntry> actions;
  private final String entry;

  private CompiledAction(Map<String, ActionDBEntry> actions, String entry) {
    this.actions = Map.copyOf(actions);
    this.entry = entry;
    if (!actions.containsKey(entry)) throw new IllegalArgumentException("No compiled ASL entry: " + entry);
  }

  public static CompiledAction compile(String sourceName, String source, String entry) {
    Objects.requireNonNull(source);
    if (source.length() > 1_048_576) throw new IllegalArgumentException("Compiled ASL source exceeds 1 MiB");
    Map<String, ActionDBEntry> actions = new LinkedHashMap<>();
    for (ActionDBEntry action : new ActionScriptLanguageParser().parseStrict(sourceName, null, source)) {
      if (actions.putIfAbsent(action.getName(), action) != null) {
        throw new IllegalArgumentException("Duplicate scoped ASL definition: " + action.getName());
      }
    }
    return new CompiledAction(actions, entry);
  }

  public ActionDBEntry getEntry() { return actions.get(entry); }

  public ActionDBEntry find(String name) { return actions.get(name); }

  public ActionDBEntry find(String name, Symbol actor, List<Class<?>> inputTypes) {
    ActionDBEntry action = actions.get(name);
    return action == null ? null
        : Utilities.getEntry(Map.of(name, List.of(action)), name, actor, inputTypes);
  }
}

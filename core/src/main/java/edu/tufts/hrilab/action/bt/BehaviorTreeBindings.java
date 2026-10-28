/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.bt;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.io.Reader;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Explicit bindings between registered BT leaves and DIARC actions. Input order is
 * the DIARC action's argument order (the actor is supplied separately).
 */
public final class BehaviorTreeBindings {
  public static final class Port {
    public String name;
    public String type = "Symbol";
    public String defaultValue;

    public Port() {
    }

    public Port(String name, String type) {
      this.name = name;
      this.type = type;
    }
  }

  public static final class Leaf {
    public String action;
    public List<Port> inputs = new ArrayList<>();
    /** Output ports in the DIARC action's return-role order. */
    public List<Port> outputs = new ArrayList<>();
    /** action, boolean, or justification. */
    public String result = "action";
    /** Synchronous leaves finish before traversal advances to the next child. */
    public boolean synchronous;

    public Leaf() {
    }

    public Leaf(String action, boolean synchronous, String result, Port... inputs) {
      this.action = action;
      this.synchronous = synchronous;
      this.result = result;
      this.inputs.addAll(List.of(inputs));
    }
  }

  private final Map<String, Leaf> leaves = new LinkedHashMap<>();
  private final Map<String, Long> enums = new LinkedHashMap<>();
  private final Map<String, Belief> beliefs = new LinkedHashMap<>();

  public static final class Belief {
    public String predicate;
    public String type = "boolean";
  }

  public BehaviorTreeBindings registerEnum(String name, long value) {
    if (!name.matches("[A-Za-z_][A-Za-z_0-9]*") || beliefs.containsKey(name)
        || enums.putIfAbsent(name, value) != null) {
      throw new IllegalArgumentException("Invalid or duplicate script enum: " + name);
    }
    return this;
  }

  public BehaviorTreeBindings bindBelief(String key, String predicate, String type) {
    if (!key.matches("[A-Za-z_][A-Za-z_0-9]*")) {
      throw new IllegalArgumentException("Invalid belief binding key: " + key);
    }
    Belief binding = new Belief();
    binding.predicate = Objects.requireNonNull(predicate);
    binding.type = Objects.requireNonNull(type);
    Class<?> javaType = BehaviorTreeExecution.portType(type);
    if (!List.of(String.class, Boolean.class, Integer.class, Long.class, Double.class,
        edu.tufts.hrilab.fol.Symbol.class, edu.tufts.hrilab.fol.Predicate.class).contains(javaType)) {
      throw new IllegalArgumentException("Belief-backed ports require a scalar or logical term type");
    }
    var variables = edu.tufts.hrilab.fol.Factory.createPredicate(predicate).getVars();
    if (variables.size() > 1 || variables.isEmpty() && javaType != Boolean.class) {
      throw new IllegalArgumentException("Belief binding needs one value variable, or a ground boolean predicate");
    }
    if (enums.containsKey(key) || beliefs.putIfAbsent(key, binding) != null) {
      throw new IllegalArgumentException("Duplicate belief binding: " + key);
    }
    return this;
  }

  String configuration() {
    return new Gson().toJson(Map.of("enums", enums, "beliefs", beliefs));
  }

  public BehaviorTreeBindings bind(String id, Leaf leaf) {
    if (!id.matches("[A-Za-z_][A-Za-z_0-9]*")) {
      throw new IllegalArgumentException("Invalid BT node ID: " + id);
    }
    Objects.requireNonNull(leaf, "leaf");
    if (leaf.action == null || !leaf.action.matches("[A-Za-z_][A-Za-z_0-9]*")) {
      throw new IllegalArgumentException("Invalid DIARC action for " + id);
    }
    if (!List.of("action", "boolean", "justification").contains(leaf.result)) {
      throw new IllegalArgumentException("Unknown result convention for " + id + ": " + leaf.result);
    }
    List<String> names = new ArrayList<>();
    for (Port port : concat(leaf.inputs, leaf.outputs)) {
      if (port.name == null || !port.name.matches("[A-Za-z_][A-Za-z_0-9]*")
          || names.contains(port.name)) {
        throw new IllegalArgumentException("Invalid or duplicate port for " + id + ": " + port.name);
      }
      BehaviorTreeExecution.portType(port.type);
      names.add(port.name);
    }
    if ("boolean".equals(leaf.result) && !leaf.outputs.isEmpty()
        && BehaviorTreeExecution.portType(leaf.outputs.get(0).type) != Boolean.class) {
      throw new IllegalArgumentException("The first output of " + id + " must be boolean");
    }
    if ("justification".equals(leaf.result) && !leaf.outputs.isEmpty()) {
      throw new IllegalArgumentException("Justification leaves use the implicit first return; omit outputs for " + id);
    }
    if (leaves.putIfAbsent(id, leaf) != null) {
      throw new IllegalArgumentException("Duplicate binding: " + id);
    }
    return this;
  }

  private static List<Port> concat(List<Port> first, List<Port> second) {
    List<Port> ports = new ArrayList<>(Objects.requireNonNull(first, "inputs"));
    ports.addAll(Objects.requireNonNull(second, "outputs"));
    return ports;
  }

  public Leaf get(String id) {
    Leaf leaf = leaves.get(id);
    if (leaf == null) {
      throw new IllegalArgumentException("No DIARC binding for BT leaf: " + id);
    }
    return leaf;
  }

  public static BehaviorTreeBindings read(Reader reader) {
    var json = JsonParser.parseReader(reader);
    if (!json.isJsonObject()) {
      throw new IllegalArgumentException("Expected a JSON object of BT leaf bindings");
    }
    BehaviorTreeBindings bindings = new BehaviorTreeBindings();
    JsonObject object = json.getAsJsonObject();
    Gson gson = new Gson();
    object.entrySet().forEach(entry -> {
      switch (entry.getKey()) {
        case "$enums":
          entry.getValue().getAsJsonObject().entrySet().forEach(e ->
              bindings.registerEnum(e.getKey(), new java.math.BigDecimal(e.getValue().getAsString()).longValueExact()));
          break;
        case "$beliefs":
          entry.getValue().getAsJsonObject().entrySet().forEach(e -> {
            Belief belief = gson.fromJson(e.getValue(), Belief.class);
            bindings.bindBelief(e.getKey(), belief.predicate, belief.type);
          });
          break;
        default: bindings.bind(entry.getKey(), gson.fromJson(entry.getValue(), Leaf.class));
      }
    });
    return bindings;
  }
}

/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.bt;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.GoalContext;
import edu.tufts.hrilab.action.execution.ActionContext;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.asl.CompiledAction;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;

import java.math.BigDecimal;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Base64;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.List;
import java.util.Deque;
import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Function;
import java.util.concurrent.ExecutionException;

/**
 * Per-invocation blackboard, local control data, and native asynchronous execution
 * handles. No tree traversal, control-node execution, threads, or scheduler live
 * here: generated ASL and the Action Interpreter perform all execution.
 */
public final class BehaviorTreeExecution {
  private static final Object UNINITIALIZED = new Object();
  private static final double COMPARISON_EPSILON = Math.ulp(1.0f);
  private final Goal owner;
  private final BehaviorTreeExecution parent;
  private final Map<String, String> aliases;
  private final boolean autoremap;
  private final int depth;
  private final Map<String, Object> blackboard = new LinkedHashMap<>();
  private final Map<String, Goal> children = new LinkedHashMap<>();
  private final Map<String, Integer> integers = new HashMap<>();
  private final Map<String, Integer> memory = new HashMap<>();
  private final Map<String, String> texts = new HashMap<>();
  private final Map<String, Long> clocks = new HashMap<>();
  private final Map<String, BehaviorTreeExecution> subtrees = new LinkedHashMap<>();
  private final Map<String, String> persistentTexts = new HashMap<>();
  private final Map<String, Long> versions = new HashMap<>();
  private final Map<String, Long> seenVersions = new HashMap<>();
  private final Map<String, Deque<Object>> queues = new HashMap<>();
  private final Map<String, CompiledAction> scripts = new HashMap<>();
  private final Map<String, Long> enums = new HashMap<>();
  private final Map<String, BehaviorTreeBindings.Belief> beliefs = new HashMap<>();
  private static final Map<String, Class<?>> PORT_TYPES = Map.ofEntries(
      Map.entry("String", String.class), Map.entry("java.lang.String", String.class),
      Map.entry("Symbol", Symbol.class), Map.entry("edu.tufts.hrilab.fol.Symbol", Symbol.class),
      Map.entry("Predicate", Predicate.class), Map.entry("edu.tufts.hrilab.fol.Predicate", Predicate.class),
      Map.entry("boolean", Boolean.class), Map.entry("Boolean", Boolean.class), Map.entry("java.lang.Boolean", Boolean.class),
      Map.entry("int", Integer.class), Map.entry("Integer", Integer.class), Map.entry("java.lang.Integer", Integer.class),
      Map.entry("long", Long.class), Map.entry("Long", Long.class), Map.entry("java.lang.Long", Long.class),
      Map.entry("double", Double.class), Map.entry("Double", Double.class), Map.entry("java.lang.Double", Double.class));
  private static final Map<String, Conversion<?>> converters = new ConcurrentHashMap<>();
  private record Conversion<T>(Class<T> type, Function<String, T> convert) { }
  private boolean closed;

  public BehaviorTreeExecution(Context caller, String initialBlackboard) {
    parent = null;
    aliases = Map.of();
    autoremap = false;
    depth = 0;
    while (caller != null && !(caller instanceof GoalContext)) {
      caller = caller.getParentContext();
    }
    if (caller == null) {
      throw new IllegalArgumentException("A behavior tree must execute inside a DIARC goal");
    }
    owner = ((GoalContext) caller).getGoal();
    Map<String, Object> initial = new Gson().fromJson(initialBlackboard,
        new TypeToken<LinkedHashMap<String, Object>>() { }.getType());
    if (initial == null || initial.containsValue(null)) {
      throw new IllegalArgumentException("Blackboard must be a JSON object with non-null values");
    }
    blackboard.putAll(initial);
    initial.keySet().forEach(key -> versions.put(key, 1L));
  }

  private BehaviorTreeExecution root() {
    return parent == null ? this : parent.root();
  }

  public synchronized void configure(String encoded) {
    var config = com.google.gson.JsonParser.parseString(decoded(encoded)).getAsJsonObject();
    config.getAsJsonObject("enums").entrySet().forEach(e -> enums.put(e.getKey(), e.getValue().getAsLong()));
    config.getAsJsonObject("beliefs").entrySet().forEach(e ->
        beliefs.put(e.getKey(), new Gson().fromJson(e.getValue(), BehaviorTreeBindings.Belief.class)));
    if (beliefs.keySet().stream().anyMatch(blackboard::containsKey)) {
      throw new IllegalArgumentException("Initial local blackboard cannot shadow explicit belief bindings");
    }
  }

  /** Register once per JVM, using a dotted application namespace (for example, my.robot.Point). */
  public static <T> void registerPortConverter(String name, Class<T> type, Function<String, T> converter) {
    if (name == null || PORT_TYPES.containsKey(name)
        || Set.of("java.util.List", "java.util.Queue", "java.util.Deque").contains(name)) {
      throw new IllegalArgumentException("Reserved or missing BT port converter name: " + name);
    }
    if (!name.matches("[A-Za-z_][A-Za-z_0-9]*(\\.[A-Za-z_][A-Za-z_0-9]*)+")) {
      throw new IllegalArgumentException("BT port converter requires a dotted namespaced identifier: " + name);
    }
    if (converters.putIfAbsent(name, new Conversion<>(java.util.Objects.requireNonNull(type),
        java.util.Objects.requireNonNull(converter))) != null) {
      throw new IllegalArgumentException("Duplicate BT port converter: " + name);
    }
  }

  private BehaviorTreeExecution(BehaviorTreeExecution parent, Map<String, String> remaps, boolean autoremap) {
    this.parent = parent;
    owner = parent.owner;
    depth = parent.depth + 1;
    this.autoremap = autoremap;
    aliases = new LinkedHashMap<>();
    remaps.forEach((key, value) -> {
      if (value.startsWith("{") && value.endsWith("}")) {
        aliases.put(key, value.substring(1, value.length() - 1));
      } else {
        blackboard.put(key, value);
        versions.put(key, 1L);
      }
    });
  }

  public synchronized BehaviorTreeExecution subtree(String path, String encodedRemaps, boolean auto) {
    if (depth >= 128) {
      throw new IllegalStateException("Behavior-tree recursion exceeds 128 active subtree frames");
    }
    return subtrees.computeIfAbsent(path, key -> new BehaviorTreeExecution(this,
        new Gson().fromJson(decoded(encodedRemaps), new TypeToken<LinkedHashMap<String, String>>() { }.getType()), auto));
  }

  public synchronized boolean hasSubtree(String path) {
    return subtrees.containsKey(path);
  }

  public synchronized BehaviorTreeExecution existingSubtree(String path) {
    BehaviorTreeExecution subtree = subtrees.get(path);
    if (subtree == null) {
      throw new IllegalStateException("Missing subtree frame: " + path);
    }
    return subtree;
  }

  private String parentKey(String key) {
    if (aliases.containsKey(key)) {
      return aliases.get(key);
    }
    return parent != null && autoremap && !key.startsWith("_") ? key : null;
  }

  private synchronized boolean contains(String key) {
    if (key.startsWith("@")) {
      return root().contains(key.substring(1));
    }
    String target = parentKey(key);
    return target == null ? blackboard.containsKey(key) || beliefs.containsKey(key) : parent.contains(target);
  }

  public synchronized Object scriptRead(String key) {
    if (key.startsWith("@")) {
      return root().scriptRead(key.substring(1));
    }
    if (root().enums.containsKey(key)) {
      return root().enums.get(key);
    }
    String target = parentKey(key);
    if (target != null) {
      return parent.scriptRead(target);
    }
    if (beliefs.containsKey(key)) {
      return readBelief(beliefs.get(key));
    }
    if (!blackboard.containsKey(key)) {
      throw new IllegalArgumentException("Missing behavior-tree blackboard entry: " + key);
    }
    return blackboard.get(key);
  }

  public synchronized boolean started(String path) {
    if (closed) {
      throw new IllegalStateException("Behavior-tree execution is closed");
    }
    return children.containsKey(path);
  }

  public void attach(String path, long contextId) {
    Goal child = owner.getActionInterpreter().getAsynchronousGoal(contextId);
    if (child == null) {
      throw new IllegalStateException("No native asynchronous execution for context " + contextId);
    }
    attachGoal(path, child);
  }

  public void attachTask(String path, edu.tufts.hrilab.action.execution.AslTask task) {
    attachGoal(path, task.getNativeGoal());
  }

  private void attachGoal(String path, Goal child) {
    synchronized (this) {
      if (!closed) {
        children.put(path, child);
        return;
      }
    }
    stopAndRelease(child);
    throw new IllegalStateException("Behavior-tree execution is closed");
  }

  public synchronized String status(String path) {
    Goal child = children.get(path);
    if (child == null) {
      throw new IllegalStateException("Missing native execution for " + path);
    }
    GoalStatus status = child.getStatus();
    switch (status) {
      case PENDING: case ACTIVE: case SUSPENDED:
        return "RUNNING";
      case FAILED:
        ActionStatus failure = child.getRootContext().getStatus();
        if (failure == ActionStatus.FAIL_NOTFOUND || failure == ActionStatus.FAIL_ARGUMENTS
            || failure == ActionStatus.FAIL_ARGUMENTUNBOUND || failure == ActionStatus.FAIL_SYNTAX) {
          throw new IllegalStateException("Invalid BT leaf binding: " + child.getRootContext().getJustification());
        }
        return "FAILURE";
      case SUCCEEDED:
        return texts.getOrDefault(path + ":outcome", "SUCCESS");
      default:
        throw new IllegalStateException("Unexpected BT child status: " + status);
    }
  }

  public synchronized void write(String key, Object value) {
    if (closed || value == null) {
      throw new IllegalStateException("Cannot publish an unbound output or write to a closed blackboard");
    }

    if (key.startsWith("@")) {
      root().write(key.substring(1), value);
      return;
    }
    String target = parentKey(key);
    if (target != null) {
      parent.write(target, value);
    } else {
      if (beliefs.containsKey(key)) {
        writeBelief(beliefs.get(key), value);
      } else {
        blackboard.put(key, value);
      }
      if (value == UNINITIALIZED) {
        versions.putIfAbsent(key, 0L);
      } else {
        versions.merge(key, 1L, Long::sum);
      }
    }
  }

  public synchronized Object value(String expression, String type) {
    Object raw = expression;
    if (expression.startsWith("{") && expression.endsWith("}")) {
      String key = expression.substring(1, expression.length() - 1);
      raw = scriptRead(key);
    }
    return convertValue(raw, type);
  }

  private static Object convertValue(Object raw, String type) {
    Conversion<?> converter = converters.get(type);
    if (converter != null) {
      return converter.type.isInstance(raw) ? raw : java.util.Objects.requireNonNull(converter.convert.apply(raw.toString()));
    }
    if (type.matches("(List|Queue)<(int|long|double|boolean|String)>")) {
      String element = type.substring(type.indexOf('<') + 1, type.length() - 1);
      if (type.startsWith("Queue") && raw instanceof Deque<?> queue) {
        Class<?> elementType = portType(element);
        synchronized (queue) {
          for (Object value : queue) {
            if (!elementType.isInstance(value)) {
              throw new IllegalArgumentException("Shared " + type + " requires " + elementType.getName()
                  + " elements, not " + (value == null ? "null" : value.getClass().getName()));
            }
          }
        }
        return queue;
      }
      Collection<?> source = raw instanceof Collection<?> ? (Collection<?>) raw :
          raw.toString().isEmpty() ? List.of() : List.of(raw.toString().split(";", -1));
      List<Object> values = new ArrayList<>();
      source.forEach(v -> values.add(convertValue(v, element)));
      return type.startsWith("Queue") ? new ArrayDeque<>(values) : values;
    }
    Class<?> target = portType(type);
    if (target.isInstance(raw)) {
      return raw;
    }
    String text = raw.toString();
    if (target == String.class) {
      return raw instanceof Boolean ? ((Boolean) raw ? "1" : "0") : scriptString(raw);
    } else if (target == Symbol.class) {
      return Factory.createSymbol(text);
    } else if (target == Predicate.class) {
      return Factory.createPredicate(text);
    } else if (target == Boolean.class) {
      if (raw instanceof Number) {
        return scriptTruthy(raw);
      }
      if (!text.equalsIgnoreCase("true") && !text.equalsIgnoreCase("false")) {
        throw new IllegalArgumentException("Not a boolean: " + text);
      }
      return Boolean.valueOf(text);
    } else if (target == Integer.class) {
      return new BigDecimal(text).intValueExact();
    } else if (target == Long.class) {
      return new BigDecimal(text).longValueExact();
    } else {
      double number = Double.parseDouble(text);
      if (!Double.isFinite(number)) {
        throw new IllegalArgumentException("Not a finite number: " + text);
      }
      return number;
    }
  }

  public Object encodedValue(String encodedExpression, String type) {
    return value(new String(Base64.getDecoder().decode(encodedExpression), StandardCharsets.UTF_8), type);
  }

  public static Class<?> portType(String type) {
    Class<?> builtin = PORT_TYPES.get(type);
    if (builtin != null) {
      return builtin;
    }
    if (type.matches("(List|Queue)<(int|long|double|boolean|String)>")) {
      return type.startsWith("Queue") ? Deque.class : List.class;
    }
    Conversion<?> converter = converters.get(type);
    if (converter != null) {
      return converter.type;
    }
    throw new IllegalArgumentException("Unsupported BT port type: " + type);
  }

  public synchronized int integer(String path, String field) {
    return integers.getOrDefault(path + ":" + field, 0);
  }

  public synchronized void integer(String path, String field, int value) {
    integers.put(path + ":" + field, value);
  }

  public synchronized int memory(String path) {
    return memory.getOrDefault(path, 0);
  }

  public synchronized void memory(String path, int value) {
    memory.put(path, value);
  }

  public synchronized String text(String path) {
    return texts.getOrDefault(path, "IDLE");
  }

  public synchronized void text(String path, String value) {
    texts.put(path, value);
  }

  public synchronized String persistentText(String path) {
    return persistentTexts.getOrDefault(path, "IDLE");
  }

  public synchronized void persistentText(String path, String value) {
    persistentTexts.put(path, value);
  }

  public static String decoded(String encoded) {
    return new String(Base64.getDecoder().decode(encoded), StandardCharsets.UTF_8);
  }

  public static Object scriptNumber(String text) {
    if (text.startsWith("0x") || text.startsWith("0X")) {
      return Long.parseLong(text.substring(2), 16);
    }
    if (text.contains(".") || text.contains("e") || text.contains("E")) {
      return Double.valueOf(text);
    }
    return Long.valueOf(text);
  }

  public static Object scriptIdentity(Object value) {
    return value;
  }

  public synchronized void scriptPrepareAssignment(String key, String operator) {
    if (contains(key)) {
      return;
    }
    if (!operator.equals(":=")) {
      throw new IllegalArgumentException("Assignment to undeclared blackboard entry: " + key);
    }
    write(key, UNINITIALIZED);
  }

  public synchronized Object scriptAssign(String key, String operator, Object value) {
    if (key.startsWith("@")) {
      return root().scriptAssign(key.substring(1), operator, value);
    }
    if (value == UNINITIALIZED) {
      throw new IllegalArgumentException("Uninitialized script assignment value: " + key);
    }
    if (!operator.equals(":=") && !contains(key)) {
      throw new IllegalArgumentException("Assignment to undeclared blackboard entry: " + key);
    }
    Object assigned = value;
    String target = parentKey(key);
    if (target != null) {
      return parent.scriptAssign(target, operator, value);
    }
    if (!operator.equals("=") && !operator.equals(":=")) {
      assigned = scriptBinary(operator.substring(0, operator.length() - 1), scriptRead(key), value);
    }
    Object previous = blackboard.get(key);
    if (previous != null && previous != UNINITIALIZED) {
      if (previous instanceof String && !(assigned instanceof String)) {
        throw new IllegalArgumentException("Cannot assign a numeric value to a string script variable: " + key);
      }
      if (previous instanceof Number) {
        if (assigned instanceof String) {
          assigned = stringNumber((String) assigned);
        } else if (previous instanceof Long) {
          assigned = new BigDecimal(assigned.toString()).longValueExact();
        } else if (previous instanceof Integer) {
          assigned = new BigDecimal(assigned.toString()).intValueExact();
        } else {
          assigned = number(assigned);
        }
      } else if (previous instanceof Boolean) {
        assigned = scriptTruthy(assigned);
      }
    }
    write(key, assigned);
    return assigned;
  }

  public static boolean scriptTruthy(Object value) {
    if (value instanceof Boolean) {
      return (Boolean) value;
    }
    if (value instanceof Number) {
      if ((value instanceof Long || value instanceof Integer)
          && ((Number) value).longValue() != 0 && ((Number) value).longValue() != 1) {
        throw new IllegalArgumentException("Integer script boolean must be 0 or 1: " + value);
      }
      return ((Number) value).doubleValue() != 0;
    }
    throw new IllegalArgumentException("Script condition requires a boolean or number: " + value);
  }

  public static boolean scriptConditionalTruthy(Object value) {
    if (value instanceof String) {
      if (((String) value).isEmpty()) {
        throw new IllegalArgumentException("Cannot convert empty script string to a number");
      }
      return true;
    }
    return number(value) != 0;
  }

  private static double number(Object value) {
    if (value instanceof Boolean) {
      return (Boolean) value ? 1 : 0;
    }
    if (value instanceof Number) {
      return ((Number) value).doubleValue();
    }
    throw new IllegalArgumentException("Script arithmetic requires a number: " + value);
  }

  public static Object scriptUnary(String operator, Object operand) {
    switch (operator) {
      case "!": return number(operand) != 0 ? 0.0 : 1.0;
      case "-": return -number(operand);
      case "+": return number(operand);
      case "~": return (double) ~integral(number(operand), false);
      default: throw new IllegalArgumentException("Unknown script unary operator: " + operator);
    }
  }

  private static long integral(double value, boolean exact) {
    if (!Double.isFinite(value) || value < Long.MIN_VALUE || value >= 0x1.0p63
        || exact && Math.rint(value) != value) {
      throw new IllegalArgumentException("Script bitwise operand is not a signed 64-bit integer: " + value);
    }
    return (long) value;
  }

  private static double truth(boolean value) {
    return value ? 1.0 : 0.0;
  }

  private static String scriptString(Object value) {
    if (value instanceof Double || value instanceof Float) {
      return String.format(java.util.Locale.ROOT, "%.6f", ((Number) value).doubleValue());
    }
    return value.toString();
  }

  private double stringNumber(String value) {
    if (value.equals("true")) { return 1; }
    if (value.equals("false")) { return 0; }
    if (root().enums.containsKey(value)) { return root().enums.get(value); }
    return Double.parseDouble(value);
  }

  public Object scriptBinary(String operator, Object left, Object right) {
    if (left == UNINITIALIZED || right == UNINITIALIZED) {
      throw new IllegalArgumentException("Uninitialized script operand");
    }
    if (operator.equals("..") && (left instanceof String || right instanceof String)) {
      return scriptString(left) + scriptString(right);
    }
    boolean comparison = java.util.Set.of("==", "!=", "<", "<=", ">", ">=").contains(operator);
    if (left instanceof String || right instanceof String) {
      if (!(left instanceof String && right instanceof String)) {
        if (!comparison) {
          throw new IllegalArgumentException("Cannot mix string and numeric script operands");
        }
        left = left instanceof String ? stringNumber((String) left) : left;
        right = right instanceof String ? stringNumber((String) right) : right;
      } else {
        int ordering = ((String) left).compareTo((String) right);
        switch (operator) {
          case "+": return left.toString() + right;
          case "==": return truth(ordering == 0);
          case "!=": return truth(ordering != 0);
          case "<": return truth(ordering < 0);
          case "<=": return truth(ordering <= 0);
          case ">": return truth(ordering > 0);
          case ">=": return truth(ordering >= 0);
          default: throw new IllegalArgumentException("Invalid string operator: " + operator);
        }
      }
    }
    double a = number(left);
    double b = number(right);
    switch (operator) {
      case "+": return a + b;
      case "-": return a - b;
      case "*": return a * b;
      case "/": return a / b;
      case "==": return truth(Math.abs(a - b) <= COMPARISON_EPSILON);
      case "!=": return truth(!(Math.abs(a - b) <= COMPARISON_EPSILON));
      case "<": return truth(!(a >= b));
      case "<=": return truth(!(a > b));
      case ">": return truth(!(a <= b));
      case ">=": return truth(!(a < b));
      case "&": return (double) (integral(a, true) & integral(b, true));
      case "|": return (double) (integral(a, true) | integral(b, true));
      case "^": return (double) (integral(a, true) ^ integral(b, true));
      case "&&": return truth(scriptTruthy(left) & scriptTruthy(right));
      case "||": return truth(scriptTruthy(left) | scriptTruthy(right));
      default: throw new IllegalArgumentException("Unknown script binary operator: " + operator);
    }
  }

  public synchronized boolean switchMatches(String variable, String candidate) {
    String expression = decoded(variable);
    String expected = decoded(candidate);
    for (String value : new String[]{expression, expected}) {
      if (value.startsWith("{") && value.endsWith("}") && !contains(value.substring(1, value.length() - 1))) {
        return false;
      }
    }
    String actual = value(expression, "String").toString();
    expected = value(expected, "String").toString();
    if (actual.equals(expected)) {
      return true;
    }
    try {
      return Math.abs(stringNumber(actual) - stringNumber(expected)) <= COMPARISON_EPSILON;
    } catch (NumberFormatException notNumeric) {
      return false;
    }
  }

  private StateMachine stateMachine() {
    return owner.getRootContext().getStateMachine();
  }

  private Object readBelief(BehaviorTreeBindings.Belief binding) {
    Predicate predicate = Factory.createPredicate(binding.predicate);
    var result = stateMachine().holds(predicate);
    if (predicate.getVars().isEmpty()) {
      return result.getValue();
    }
    Variable variable = predicate.getVars().iterator().next();
    Set<Symbol> values = result.getBindings(variable);
    if (!result.getValue() || values.size() != 1) {
      throw new IllegalStateException("Belief-backed port requires exactly one value: " + binding.predicate);
    }
    return convertValue(values.iterator().next(), binding.type);
  }

  private void writeBelief(BehaviorTreeBindings.Belief binding, Object value) {
    if (value == UNINITIALIZED) {
      throw new IllegalArgumentException("Belief-backed ports cannot contain uninitialized values");
    }
    Predicate predicate = Factory.createPredicate(binding.predicate);
    StateMachine state = stateMachine();
    synchronized (state) {
      if (predicate.getVars().isEmpty()) {
        state.updateState(Map.of(predicate, (Boolean) convertValue(value, "boolean")));
        return;
      }
      Variable variable = predicate.getVars().iterator().next();
      Object converted = convertValue(value, binding.type);
      Symbol symbol = converted instanceof Symbol ? (Symbol) converted :
          Factory.createSymbol(converted instanceof String ? new Gson().toJson(converted) : converted.toString());
      var previous = state.holds(predicate);
      for (Map<Variable, Symbol> bindings : previous.getBindings()) {
        state.retractBelief(predicate.copyWithNewBindings(bindings));
      }
      state.assertBelief(predicate.copyWithNewBindings(Map.of(variable, symbol)), MemoryLevel.EPISODIC,
          Factory.createPredicate("source(behaviorTree)"));
    }
  }

  public synchronized void setBlackboard(String encodedKey, String encodedValue) {
    String key = value(decoded(encodedKey), "String").toString();
    String source = decoded(encodedValue);
    Object result = source.startsWith("{") && source.endsWith("}") ?
        scriptRead(source.substring(1, source.length() - 1)) : source;
    if (result == UNINITIALIZED) {
      throw new IllegalArgumentException("Cannot copy an uninitialized blackboard entry");
    }
    write(key, result);
  }

  public synchronized void unsetBlackboard(String encodedKey) {
    String key = value(decoded(encodedKey), "String").toString();
    remove(key);
  }

  private synchronized void remove(String key) {
    if (beliefs.containsKey(key)) {
      Predicate predicate = Factory.createPredicate(beliefs.get(key).predicate);
      StateMachine state = stateMachine();
      synchronized (state) {
        if (predicate.getVars().isEmpty()) {
          state.retractBelief(predicate);
        } else {
          for (var bindings : state.holds(predicate).getBindings()) {
            state.retractBelief(predicate.copyWithNewBindings(bindings));
          }
        }
      }
    } else {
      blackboard.remove(key);
      versions.remove(key);
    }
  }

  private synchronized long version(String key) {
    if (key.startsWith("@")) {
      return root().version(key.substring(1));
    }
    String target = parentKey(key);
    if (target != null) {
      return parent.version(target);
    }
    if (beliefs.containsKey(key)) {
      throw new IllegalArgumentException("Use native belief conditions/notifications for belief-backed ports, not BT entry stamps");
    }
    return versions.getOrDefault(key, -1L);
  }

  public synchronized boolean entryUpdated(String path, String encodedEntry) {
    String key = decoded(encodedEntry);
    if (key.startsWith("{") && key.endsWith("}")) {
      key = key.substring(1, key.length() - 1);
    }
    long current = version(key);
    long previous = seenVersions.getOrDefault(path, 0L);
    if (current >= 0) {
      seenVersions.put(path, current);
    }
    return current >= 0 && current != previous;
  }

  public synchronized boolean queuePop(String path, String encodedQueue, String encodedOutput, String type) {
    String expression = decoded(encodedQueue);
    Object raw = expression.startsWith("{") && expression.endsWith("}") ?
        scriptRead(expression.substring(1, expression.length() - 1)) : expression;
    Deque<?> queue;
    if (raw instanceof Deque<?>) {
      queue = (Deque<?>) raw;
    } else {
      queue = queues.computeIfAbsent(path, key -> {
        Collection<?> values = raw instanceof Collection<?> ? (Collection<?>) raw :
            raw.toString().isEmpty() ? List.of() : List.of(raw.toString().split(";", -1));
        Deque<Object> copy = new ArrayDeque<>();
        values.forEach(v -> copy.add(convertValue(v, type)));
        return copy;
      });
    }
    synchronized (queue) {
      if (queue.isEmpty()) {
        return false;
      }
      String output = decoded(encodedOutput);
      Object value = convertValue(queue.getFirst(), type);
      write(output.substring(1, output.length() - 1), value);
      queue.removeFirst();
      return true;
    }
  }

  public synchronized void appendScript(Context parentContext, String encodedSource, boolean predicate) {
    Context caller = parentContext.getChildContexts().getCurrent();
    if (!(caller instanceof ActionContext)
        || !((ActionContext) caller).hasLocalArgument("?answer")) {
      throw new IllegalStateException("Dynamic script must be appended to its calling ASL helper");
    }
    String source = value(decoded(encodedSource), "String").toString();
    String key = predicate + ":" + source;
    CompiledAction entry = scripts.get(key);
    if (entry == null) {
      if (scripts.size() >= 256) {
        throw new IllegalStateException("Too many distinct dynamic scripts in one invocation");
      }
      String body = "import java.lang.Boolean;\n(Boolean ?answer) = btRuntimeScript("
          + BehaviorTreeExecution.class.getCanonicalName() + " ?execution) {\njava.lang.Object !scriptResult;\n"
          + BehaviorTreeScript.compile(source, "s", "!scriptResult", false)
          + "?answer = " + (predicate ? "op:invokeMethod(?execution, \"scriptTruthy\", !scriptResult)"
          : "op:equals(true, true)") + ";\n}\n";
      entry = CompiledAction.compile("dynamic-behavior-tree-script.asl", body, "btRuntimeScript");
      scripts.put(key, entry);
    }
    Object actor = caller.getArgumentValue("?actor");
    if (!(actor instanceof Symbol)) {
      throw new IllegalStateException("Dynamic script caller has no bound actor");
    }
    caller.addCompiledAction(entry, List.of("?execution"), List.of("?answer"), (Symbol) actor);
  }

  public static void validateStatus(String status) {
    if (!java.util.Set.of("SUCCESS", "FAILURE", "SKIPPED").contains(status)) {
      throw new IllegalArgumentException("Invalid Precondition else status: " + status);
    }
  }

  public synchronized long elapsed(String path) {
    return (System.nanoTime() - clocks.computeIfAbsent(path, key -> System.nanoTime())) / 1_000_000;
  }

  private static boolean inside(String key, String prefix) {
    return key.equals(prefix) || key.startsWith(prefix + "/") || key.startsWith(prefix + ":");
  }

  /** Halt/reset a branch; successful blackboard writes are deliberately retained. */
  public void reset(String prefix) {
    Map<String, Goal> removed = new LinkedHashMap<>();
    synchronized (this) {
      for (Map.Entry<String, Goal> child : new ArrayList<>(children.entrySet())) {
        if (inside(child.getKey(), prefix)) {
          removed.put(child.getKey(), child.getValue());
          children.remove(child.getKey());
        }
      }
    }
    // Never hold the blackboard monitor while interrupt handlers finish execution.
    for (Goal child : removed.values()) {
      stopAndRelease(child);
    }
    synchronized (this) {
      integers.keySet().removeIf(key -> inside(key, prefix));
      texts.keySet().removeIf(key -> inside(key, prefix));
      clocks.keySet().removeIf(key -> inside(key, prefix));
      queues.keySet().removeIf(key -> inside(key, prefix));
    }
  }

  private void stopAndRelease(Goal child) {
    if (!child.getStatus().isTerminated() && !child.cancel()
        && !child.getStatus().isTerminated()) {
      throw new IllegalStateException("Could not halt BT child " + child.getId());
    }
    try {
      if (!owner.getActionInterpreter().releaseAsynchronousGoal(child.getRootContext().getId())) {
        throw new IllegalStateException("Could not join BT child " + child.getId());
      }
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IllegalStateException("Interrupted while joining BT child", e);
    } catch (ExecutionException e) {
      throw new IllegalStateException("BT child interpreter failed", e);
    }
  }

  public void close() {
    synchronized (this) {
      closed = true;
    }
    reset("0");
    for (BehaviorTreeExecution subtree : new ArrayList<>(subtrees.values())) {
      subtree.close();
    }
  }
}

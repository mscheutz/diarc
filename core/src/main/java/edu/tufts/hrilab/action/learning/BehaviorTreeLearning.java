/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.learning;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.bt.BehaviorTreeBindings;
import edu.tufts.hrilab.action.bt.BehaviorTreeConverter;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import java.io.IOException;
import java.io.StringReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Explicit structured instruction learning, independent of the legacy linear
 * goal queue. Instructions identify parents and insertion positions; ordinary
 * action teaching never implies a fallback or another control structure.
 *
 * A draft can contain empty controls, but publishing/export requires a complete
 * tree accepted by the native BT converter. Finished models remain available for
 * export. Bindings are snapshotted, not borrowed from a mutable caller.
 */
public final class BehaviorTreeLearning {
  private static final Gson GSON = new Gson();
  private static final Set<String> CONTROLS = Set.of("Sequence", "Fallback",
      "ReactiveSequence", "ReactiveFallback", "SequenceWithMemory", "Parallel", "ParallelAll",
      "IfThenElse", "WhileDoElse", "AsyncSequence", "AsyncFallback", "TryCatch");
  private static final Set<String> DECORATORS = Set.of("Inverter", "ForceSuccess", "ForceFailure",
      "RetryUntilSuccessful", "Repeat", "Timeout", "Delay", "KeepRunningUntilFailure", "RunOnce",
      "Precondition", "LoopInt", "LoopBool", "LoopDouble", "LoopString", "SkipUnlessUpdated", "WaitValueUpdate");
  private static final Set<String> BUILTINS = Set.of("AlwaysSuccess", "AlwaysFailure", "Script",
      "ScriptCondition", "SetBlackboard", "UnsetBlackboard", "Sleep", "WasEntryUpdated");
  private static final Set<String> RESERVED = Set.of("SubTree", "Action", "Condition", "Control", "Decorator");
  private final Map<String, Draft> drafts = new LinkedHashMap<>();

  private static final class Node {
    final String id;
    final String kind;
    final String subtree;
    final Map<String, String> ports;
    final List<Node> children = new ArrayList<>();

    Node(String id, String kind, Map<String, String> ports) {
      this(id, kind, ports, null);
    }

    Node(String id, String kind, Map<String, String> ports, String subtree) {
      this.id = id;
      this.kind = kind;
      this.subtree = subtree;
      this.ports = new LinkedHashMap<>(ports);
    }

    boolean branch() {
      return isControl(kind) || DECORATORS.contains(kind);
    }
  }

  private static final class Draft {
    final String name;
    final Symbol actor;
    final String bindingsJson;
    final BehaviorTreeBindings bindings;
    final Node root;
    final Map<String, Node> definitions = new LinkedHashMap<>();
    boolean finished;

    Draft(String name, Symbol actor, String rootKind, String bindingsJson, Map<String, String> attributes) {
      this.name = name;
      this.actor = actor;
      this.bindingsJson = bindingsJson;
      bindings = BehaviorTreeBindings.read(new StringReader(bindingsJson));
      root = new Node("root", rootKind, attributes);
      definitions.put(name, root);
    }
  }

  /** Begin a draft with an explicit root control, whose stable node ID is "root". */
  public synchronized boolean begin(String name, Symbol actor, String rootKind, String bindingsJson) {
    return begin(name, actor, rootKind, bindingsJson, "{}");
  }

  /** Begin with explicit root parameters; root arity is checked only when the tree is validated. */
  public synchronized boolean begin(String name, Symbol actor, String rootKind,
                                      String bindingsJson, String attributesJson) {
    identifier(name);
    Objects.requireNonNull(actor, "actor");
    if (drafts.containsKey(name)) {
      throw new IllegalArgumentException("A learned tree or draft already exists: " + name);
    }
    // Validate before making the draft visible.
    Draft draft = new Draft(name, actor, rootKind, bindingsJson, attributes(attributesJson));
    kind(draft, rootKind);
    drafts.put(name, draft);
    return true;
  }

  public synchronized boolean beginFromFile(String name, Symbol actor, String rootKind,
                                             String bindingsFile) throws IOException {
    return begin(name, actor, rootKind, Files.readString(Path.of(bindingsFile), StandardCharsets.UTF_8));
  }

  public synchronized boolean beginFromFile(String name, Symbol actor, String rootKind,
                                             String bindingsFile, String attributesJson) throws IOException {
    return begin(name, actor, rootKind,
        Files.readString(Path.of(bindingsFile), StandardCharsets.UTF_8), attributesJson);
  }

  public synchronized boolean addControl(String name, String parent, String id, String kind, int index) {
    control(kind);
    return insert(editable(name), parent, new Node(id, kind, Map.of()), index);
  }

  /**
   * Add any converter-supported native node or registered action leaf. Native
   * attributes and final child counts are checked by the converter at export or
   * publication, so incomplete branches/decorators remain editable.
   */
  public synchronized boolean addNode(String name, String parent, String id, String kind,
                                       String attributesJson, int index) {
    Draft draft = editable(name);
    kind(draft, kind);
    return insert(draft, parent, new Node(id, kind, attributes(attributesJson)), index);
  }

  /** Define a callable tree. Node identifiers are unique across all definitions in a draft. */
  public synchronized boolean addDefinition(String name, String treeId, String rootId,
                                             String kind, String attributesJson) {
    Draft draft = editable(name);
    identifier(treeId);
    identifier(rootId);
    if (draft.definitions.containsKey(treeId)) {
      throw new IllegalArgumentException("Duplicate behavior-tree definition: " + treeId);
    }
    if (lookup(draft, rootId) != null) {
      throw new IllegalArgumentException("Duplicate node ID: " + rootId);
    }
    kind(draft, kind);
    Node root = new Node(rootId, kind, attributes(attributesJson));
    draft.definitions.put(treeId, root);
    return true;
  }

  /** Explicit call/remappings; forward and recursive references are validated at publication/export. */
  public synchronized boolean addSubtree(String name, String parent, String id, String treeId,
                                          String remapsJson, int index) {
    Draft draft = editable(name);
    identifier(treeId);
    return insert(draft, parent, new Node(id, "SubTree", attributes(remapsJson), treeId), index);
  }

  /**
   * Add a registered leaf. Ports are an explicit JSON string-to-string object,
   * including any output remappings. The converter validates all port names,
   * required inputs and blackboard expressions before insertion.
   */
  public synchronized boolean addLeaf(String name, String parent, String id,
                                       String binding, String portsJson, int index) {
    Map<String, String> ports = attributes(portsJson);
    Draft draft = editable(name);
    if (nativeKind(binding) || RESERVED.contains(binding)) {
      throw new IllegalArgumentException("Binding ID conflicts with a native BT node: " + binding);
    }
    BehaviorTreeBindings.Leaf registered = draft.bindings.get(binding);
    Set<String> allowed = new java.util.HashSet<>();
    registered.inputs.forEach(port -> allowed.add(port.name));
    registered.outputs.forEach(port -> allowed.add(port.name));
    for (String port : allowed) {
      bindingPort(port);
    }
    for (String port : ports.keySet()) {
      if (!allowed.contains(port) || !port.matches("[A-Za-z_][A-Za-z_0-9]*")
          || Set.of("ID", "name").contains(port) || port.startsWith("_")) {
        throw new IllegalArgumentException("Unknown or reserved leaf port: " + port);
      }
    }
    Node leaf = new Node(id, binding, ports);
    validateLeaf(draft, leaf);
    return insert(draft, parent, leaf, index);
  }

  /**
   * Record an existing ASL action call into a specified registered leaf.
   * EventSpec supplies the same actor/argument convention as action learning.
   * Concrete terms are retained; role abstraction and output mapping must be
   * supplied explicitly with addLeaf rather than guessed from demonstrations.
   */
  public synchronized boolean record(String name, String parent, String id,
                                      String binding, Predicate step, int index) {
    Draft draft = editable(name);
    if (step == null || step.getArgs().isEmpty()
        || !draft.actor.toUntyped().equals(step.get(0).toUntyped())) {
      throw new IllegalArgumentException("Recorded action must have the learning actor as its first argument");
    }
    if (step.getName().equals("goal")) {
      throw new IllegalArgumentException("Record an action call, not a goal-state instruction");
    }
    EventSpec event = new EventSpec(step, true);
    BehaviorTreeBindings.Leaf leaf = draft.bindings.get(binding);
    if (event.getType() != EventSpec.EventType.ACTION || !leaf.action.equals(event.getCommand())
        || leaf.inputs.size() != event.getInputArgs().size() || !leaf.outputs.isEmpty()) {
      throw new IllegalArgumentException("Action does not match the binding, or requires explicit output mappings");
    }
    Map<String, String> ports = new LinkedHashMap<>();
    for (int i = 0; i < leaf.inputs.size(); i++) {
      String value = event.getInputArgs().get(i);
      if (!step.getVars().isEmpty() || value.startsWith("?") || value.startsWith("!") || value.startsWith("{")) {
        throw new IllegalArgumentException("Record concrete values; use addLeaf for explicit blackboard mappings");
      }
      ports.put(leaf.inputs.get(i).name, value);
    }
    return addLeaf(name, parent, id, binding, GSON.toJson(ports), index);
  }

  /** Remove a whole subtree, never just its control marker. */
  public synchronized boolean remove(String name, String id) {
    Draft draft = editable(name);
    Node node = nonRoot(draft, id);
    parent(draft, node).children.remove(node);
    return true;
  }

  /** Index is in the destination's child list after the node is removed. */
  public synchronized boolean move(String name, String id, String newParent, int index) {
    Draft draft = editable(name);
    Node node = nonRoot(draft, id);
    Node destination = find(draft, newParent);
    if (!destination.branch() || contains(node, destination)) {
      throw new IllegalArgumentException("Destination must be a control outside the moved subtree");
    }
    Node oldParent = parent(draft, node);
    int size = destination.children.size() - (oldParent == destination ? 1 : 0);
    position(index, size);
    oldParent.children.remove(node);
    destination.children.add(index, node);
    return true;
  }

  /**
   * Validate and install native ASL actions, then seal the draft. No entries are
   * installed on validation/name-conflict failure, and the draft stays editable.
   */
  public synchronized boolean finish(String name) {
    Draft draft = editable(name);
    List<ActionDBEntry> entries = new BehaviorTreeConverter(draft.bindings).actions(getXml(name), name);
    synchronized (Database.getActionDB()) {
      for (ActionDBEntry entry : entries) {
        if (Database.getActionDB().getAction(entry.getName()) != null) {
          throw new IllegalArgumentException("Behavior-tree action already exists: " + entry.getName());
        }
      }
      entries.forEach(Database.getInstance()::addActionDBEntry);
      draft.finished = true;
    }
    return true;
  }

  /** Cancel only an unpublished draft; installed trees remain retained for export. */
  public synchronized boolean cancel(String name) {
    editable(name);
    drafts.remove(name);
    return true;
  }

  /** XML is generated from retained structure, never reverse-engineered from ASL. */
  public synchronized String getXml(String name) {
    Draft draft = draft(name);
    String xml = xml(draft, name);
    // The converter compiles reachable definitions. Validate each retained
    // definition as an entry point too, so unused invalid instructions cannot hide.
    for (String treeId : draft.definitions.keySet()) {
      new BehaviorTreeConverter(draft.bindings).convert(
          treeId.equals(name) ? xml : xml(draft, treeId), name);
    }
    return xml;
  }

  public synchronized String getBindingsJson(String name) {
    return draft(name).bindingsJson;
  }

  /**
   * Stage and validate both UTF-8 artifacts before replacement, restoring the
   * previous pair on publication failure. This is not a two-file transaction
   * for concurrent readers or process crashes; deploy versioned pairs instead.
   */
  public synchronized boolean export(String name, String xmlFile, String bindingsFile) throws IOException {
    return export(name, xmlFile, bindingsFile, BehaviorTreeLearning::replaceExportFile);
  }

  @FunctionalInterface
  interface FileReplacer {
    void replace(Path source, Path destination) throws IOException;
  }

  private record ExportTarget(Path destination, Path staged, Path backup) { }

  synchronized boolean export(String name, String xmlFile, String bindingsFile, FileReplacer replacer)
      throws IOException {
    Path xmlPath = exportPath(xmlFile);
    Path bindingPath = exportPath(bindingsFile);
    if (xmlPath.equals(bindingPath)
        || (Files.exists(xmlPath) && Files.exists(bindingPath) && Files.isSameFile(xmlPath, bindingPath))) {
      throw new IllegalArgumentException("XML and bindings require distinct files");
    }
    String xml = getXml(name);
    List<Path> temporaryFiles = new ArrayList<>();
    List<ExportTarget> targets = new ArrayList<>();
    List<ExportTarget> attempted = new ArrayList<>();
    Throwable failure = null;
    try {
      for (Path destination : List.of(bindingPath, xmlPath)) {
        if (Files.exists(destination) && !Files.isRegularFile(destination)) {
          throw new IOException("Export destination is not a regular file: " + destination);
        }
        Path staged = Files.createTempFile(destination.getParent(), ".diarc-bt-stage-", ".tmp");
        temporaryFiles.add(staged);
        Path backup = null;
        if (Files.exists(destination)) {
          backup = Files.createTempFile(destination.getParent(), ".diarc-bt-backup-", ".tmp");
          temporaryFiles.add(backup);
          Files.copy(destination, backup, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.COPY_ATTRIBUTES);
          Files.copy(backup, staged, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.COPY_ATTRIBUTES);
        }
        Files.writeString(staged, destination.equals(xmlPath) ? xml : getBindingsJson(name), StandardCharsets.UTF_8);
        targets.add(new ExportTarget(destination, staged, backup));
      }
      try (var reader = Files.newBufferedReader(targets.get(0).staged, StandardCharsets.UTF_8)) {
        new BehaviorTreeConverter(BehaviorTreeBindings.read(reader)).convert(targets.get(1).staged, name);
      }
      for (ExportTarget target : targets) {
        // A failed move can have changed its destination, so restore attempted moves too.
        attempted.add(target);
        replacer.replace(target.staged, target.destination);
      }
      return true;
    } catch (IOException | RuntimeException e) {
      failure = e;
      for (int i = attempted.size() - 1; i >= 0; i--) {
        ExportTarget target = attempted.get(i);
        try {
          if (target.backup == null) {
            Files.deleteIfExists(target.destination);
          } else {
            replacer.replace(target.backup, target.destination);
          }
        } catch (IOException | RuntimeException recovery) {
          temporaryFiles.remove(target.backup);
          e.addSuppressed(new IOException("Could not restore " + target.destination
              + (target.backup == null ? "; remove the incomplete export manually"
              : "; recovery copy retained at " + target.backup), recovery));
        }
      }
      throw e;
    } finally {
      IOException cleanupFailure = null;
      for (Path temporary : temporaryFiles) {
        try {
          Files.deleteIfExists(temporary);
        } catch (IOException | RuntimeException e) {
          IOException cleanup = new IOException("Could not remove export temporary file: " + temporary, e);
          if (cleanupFailure == null) {
            cleanupFailure = cleanup;
          } else {
            cleanupFailure.addSuppressed(cleanup);
          }
        }
      }
      if (cleanupFailure != null) {
        if (failure != null) {
          failure.addSuppressed(cleanupFailure);
        } else {
          throw cleanupFailure;
        }
      }
    }
  }

  private static Path exportPath(String file) throws IOException {
    Path path = Path.of(file).toAbsolutePath().normalize();
    return Files.exists(path, LinkOption.NOFOLLOW_LINKS) ? path.toRealPath()
        : path.getParent().toRealPath().resolve(path.getFileName());
  }

  private static void replaceExportFile(Path source, Path destination) throws IOException {
    try {
      Files.move(source, destination, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
    } catch (AtomicMoveNotSupportedException e) {
      Files.move(source, destination, StandardCopyOption.REPLACE_EXISTING);
    }
  }

  private static void validateLeaf(Draft draft, Node leaf) {
    StringBuilder body = new StringBuilder();
    append(leaf, body);
    new BehaviorTreeConverter(draft.bindings).convert(document(draft.name, body.toString()), draft.name);
  }

  private static boolean insert(Draft draft, String parentId, Node node, int index) {
    identifier(node.id);
    if (lookup(draft, node.id) != null) {
      throw new IllegalArgumentException("Duplicate node ID: " + node.id);
    }
    Node parent = find(draft, parentId);
    if (!parent.branch()) {
      throw new IllegalArgumentException("A leaf cannot have children");
    }
    position(index, parent.children.size());
    parent.children.add(index, node);
    return true;
  }

  private Draft draft(String name) {
    Draft draft = drafts.get(name);
    if (draft == null) {
      throw new IllegalArgumentException("No behavior-tree learning state: " + name);
    }
    return draft;
  }

  private Draft editable(String name) {
    Draft draft = draft(name);
    if (draft.finished) {
      throw new IllegalArgumentException("Behavior tree is already published: " + name);
    }
    return draft;
  }

  private static Node nonRoot(Draft draft, String id) {
    Node node = find(draft, id);
    if (draft.definitions.containsValue(node)) {
      throw new IllegalArgumentException("Cannot remove or move a definition root");
    }
    return node;
  }

  private static Node find(Draft draft, String id) {
    Node node = lookup(draft, id);
    if (node == null) {
      throw new IllegalArgumentException("Unknown node ID: " + id);
    }
    return node;
  }

  private static Node lookup(Draft draft, String id) {
    for (Node root : draft.definitions.values()) {
      Node found = lookup(root, id);
      if (found != null) {
        return found;
      }
    }
    return null;
  }

  private static Node lookup(Node root, String id) {
    if (root.id.equals(id)) {
      return root;
    }
    for (Node child : root.children) {
      Node found = lookup(child, id);
      if (found != null) {
        return found;
      }
    }
    return null;
  }

  private static Node parent(Node root, Node target) {
    for (Node child : root.children) {
      if (child == target) {
        return root;
      }
      Node found = parent(child, target);
      if (found != null) {
        return found;
      }
    }
    return null;
  }

  private static Node parent(Draft draft, Node target) {
    for (Node root : draft.definitions.values()) {
      Node found = parent(root, target);
      if (found != null) {
        return found;
      }
    }
    return null;
  }

  private static boolean contains(Node root, Node target) {
    return lookup(root, target.id) != null;
  }

  private static void identifier(String id) {
    if (id == null || !id.matches("[A-Za-z][A-Za-z_0-9]*")) {
      throw new IllegalArgumentException("Invalid tree/node identifier: " + id);
    }
  }

  private static void control(String kind) {
    if (!isControl(kind)) {
      throw new IllegalArgumentException("Unsupported learned control: " + kind);
    }
  }

  private static boolean isControl(String kind) {
    return kind != null && (CONTROLS.contains(kind) || kind.matches("Switch[2-6]"));
  }

  private static boolean nativeKind(String kind) {
    return kind != null && (isControl(kind) || DECORATORS.contains(kind) || BUILTINS.contains(kind));
  }

  private static void kind(Draft draft, String kind) {
    if (kind == null || RESERVED.contains(kind)) {
      throw new IllegalArgumentException("Unsupported learned node: " + kind);
    }
    if (!nativeKind(kind)) {
      BehaviorTreeBindings.Leaf binding = draft.bindings.get(kind);
      for (BehaviorTreeBindings.Port port : binding.inputs) {
        bindingPort(port.name);
      }
      for (BehaviorTreeBindings.Port port : binding.outputs) {
        bindingPort(port.name);
      }
    }
  }

  private static void bindingPort(String name) {
    if (Set.of("ID", "name").contains(name) || name.startsWith("_")) {
      throw new IllegalArgumentException("Binding has a reserved XML port: " + name);
    }
  }

  private static Map<String, String> attributes(String json) {
    Map<String, String> attributes = GSON.fromJson(json,
        new TypeToken<LinkedHashMap<String, String>>() { }.getType());
    if (attributes == null || attributes.values().stream().anyMatch(Objects::isNull)) {
      throw new IllegalArgumentException("Expected a JSON object of non-null attribute values");
    }
    for (String key : attributes.keySet()) {
      if (!key.matches("[A-Za-z_][A-Za-z_0-9]*") || Set.of("ID", "name").contains(key)) {
        throw new IllegalArgumentException("Invalid or reserved XML attribute: " + key);
      }
    }
    return attributes;
  }

  private static void position(int index, int size) {
    if (index < 0 || index > size) {
      throw new IllegalArgumentException("Insertion index out of range: " + index);
    }
  }

  private static String xml(Draft draft, String main) {
    StringBuilder body = new StringBuilder();
    body.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
        .append("<root BTCPP_format=\"4\" main_tree_to_execute=\"").append(main).append("\">\n");
    draft.definitions.forEach((treeId, root) -> {
      body.append("<BehaviorTree ID=\"").append(treeId).append("\">\n");
      append(root, body);
      body.append("</BehaviorTree>\n");
    });
    return body.append("</root>\n").toString();
  }

  private static String document(String name, String body) {
    return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        + "<root BTCPP_format=\"4\" main_tree_to_execute=\"" + name + "\">\n"
        + "<BehaviorTree ID=\"" + name + "\">\n" + body + "</BehaviorTree>\n</root>\n";
  }

  private static void append(Node node, StringBuilder xml) {
    if (node.subtree != null) {
      xml.append("<SubTree ID=\"").append(escape(node.subtree)).append("\"");
    } else {
      xml.append(nativeKind(node.kind) ? "<" + node.kind : "<Action ID=\"" + escape(node.kind) + "\"");
    }
    xml.append(" name=\"").append(escape(node.id)).append("\"");
    node.ports.forEach((key, value) -> xml.append(" ").append(key).append("=\"").append(escape(value)).append("\""));
    if (node.children.isEmpty()) {
      xml.append("/>\n");
    } else {
      xml.append(">\n");
      node.children.forEach(child -> append(child, xml));
      xml.append("</").append(nativeKind(node.kind) ? node.kind : "Action").append(">\n");
    }
  }

  private static String escape(String text) {
    return text.replace("&", "&amp;").replace("\"", "&quot;").replace("<", "&lt;")
        .replace(">", "&gt;").replace("\r", "&#13;").replace("\n", "&#10;").replace("\t", "&#9;");
  }
}

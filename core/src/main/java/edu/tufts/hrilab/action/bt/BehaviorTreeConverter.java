/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.bt;

import com.google.gson.Gson;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageParser;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import javax.xml.XMLConstants;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import java.io.IOException;
import java.io.StringReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Base64;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Compiles the supported BehaviorTree.CPP format-4 nodes into ASL supervision
 * scripts. All traversal, branch selection, repetition, and threshold logic is
 * emitted as ASL; leaves run in native asynchronous Action Interpreters.
 */
public final class BehaviorTreeConverter {
  private static final String EXECUTION = BehaviorTreeExecution.class.getCanonicalName();
  private static final Set<String> CONTROLS = Set.of("Sequence", "SequenceWithMemory", "Fallback",
      "ReactiveSequence", "ReactiveFallback", "Parallel", "ParallelAll", "IfThenElse", "WhileDoElse",
      "AsyncSequence", "AsyncFallback", "TryCatch");
  private static final Set<String> DECORATORS = Set.of("Inverter", "ForceSuccess", "ForceFailure",
      "RetryUntilSuccessful", "Repeat", "Timeout", "Delay", "KeepRunningUntilFailure", "RunOnce", "Precondition",
      "LoopInt", "LoopBool", "LoopDouble", "LoopString", "SkipUnlessUpdated", "WaitValueUpdate");
  private static final Set<String> HOOKS = Set.of("_failureIf", "_successIf", "_skipIf", "_while",
      "_onSuccess", "_onFailure", "_post", "_onHalted");

  private final BehaviorTreeBindings bindings;
  private final Map<String, Element> trees = new LinkedHashMap<>();
  private final Map<String, Map<String, String>> subtreeDefaults = new HashMap<>();
  private final Map<String, CompiledNode> treeRoots = new LinkedHashMap<>();
  private final Set<String> referencedTrees = new java.util.LinkedHashSet<>();
  private final List<CompiledNode> nodes = new ArrayList<>();
  private String action;

  private static final class Scope {
    String resolve(String expression) {
      if (!expression.startsWith("{") || !expression.endsWith("}")) {
        return expression;
      }
      String key = expression.substring(1, expression.length() - 1);
      if (!key.matches("@?[A-Za-z_][A-Za-z_0-9]*")) {
        throw new IllegalArgumentException("Unsupported blackboard reference: " + expression);
      }
      return expression;
    }
  }

  private static final class CompiledNode {
    int id;
    String kind;
    String path;
    Element xml;
    Scope scope;
    BehaviorTreeBindings.Leaf binding;
    String subtree;
    Map<String, String> remaps;
    boolean auto;
    final List<CompiledNode> children = new ArrayList<>();
  }

  public BehaviorTreeConverter(BehaviorTreeBindings bindings) {
    this.bindings = bindings;
  }

  public String convert(Path xml, String actionName) throws IOException {
    clear(actionName);
    Path file = xml.toRealPath();
    Element root = read(file);
    collect(root, file.getParent(), file.getParent(), new HashSet<>(), new HashSet<>());
    return generate(root);
  }

  public String convert(String xml, String actionName) {
    clear(actionName);
    Element root;
    try {
      root = factory().newDocumentBuilder()
          .parse(new org.xml.sax.InputSource(new StringReader(xml))).getDocumentElement();
    } catch (ParserConfigurationException | SAXException | IOException e) {
      throw new IllegalArgumentException("Invalid behavior-tree XML", e);
    }
    try {
      collect(root, null, null, new HashSet<>(), new HashSet<>());
    } catch (IOException e) {
      throw new IllegalArgumentException("Could not read behavior-tree includes", e);
    }
    return generate(root);
  }

  public List<ActionDBEntry> actions(String xml, String actionName) {
    return parse(convert(xml, actionName));
  }

  public static List<ActionDBEntry> parse(String asl) {
    return new ActionScriptLanguageParser().parseStrict("generated-behavior-tree.asl", null, asl);
  }

  private void clear(String actionName) {
    if (actionName == null || !actionName.matches("[a-zA-Z][a-zA-Z_0-9]*")) {
      throw new IllegalArgumentException("Invalid ASL action name: " + actionName);
    }
    action = actionName;
    trees.clear();
    subtreeDefaults.clear();
    treeRoots.clear();
    referencedTrees.clear();
    nodes.clear();
  }

  private static DocumentBuilderFactory factory() throws ParserConfigurationException {
    DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
    factory.setFeature(XMLConstants.FEATURE_SECURE_PROCESSING, true);
    factory.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
    factory.setFeature("http://xml.org/sax/features/external-general-entities", false);
    factory.setFeature("http://xml.org/sax/features/external-parameter-entities", false);
    factory.setAttribute(XMLConstants.ACCESS_EXTERNAL_DTD, "");
    factory.setAttribute(XMLConstants.ACCESS_EXTERNAL_SCHEMA, "");
    factory.setAttribute("http://www.oracle.com/xml/jaxp/properties/maxElementDepth", 256);
    factory.setXIncludeAware(false);
    factory.setExpandEntityReferences(false);
    return factory;
  }

  private static Element read(Path path) throws IOException {
    try {
      Document document = factory().newDocumentBuilder().parse(path.toFile());
      return document.getDocumentElement();
    } catch (ParserConfigurationException | SAXException e) {
      throw new IllegalArgumentException("Invalid BT XML in " + path, e);
    }
  }

  private void collect(Element root, Path directory, Path allowedRoot,
                       Set<Path> loading, Set<Path> loaded) throws IOException {
    if (!root.getTagName().equals("root") || !root.getAttribute("BTCPP_format").equals("4")) {
      throw new IllegalArgumentException("Expected <root BTCPP_format=\"4\">");
    }
    attributes(root, Set.of("BTCPP_format", "main_tree_to_execute"));
    for (Element element : children(root)) {
      switch (element.getTagName()) {
        case "BehaviorTree":
          attributes(element, Set.of("ID"));
          String id = element.getAttribute("ID");
          if (id.isEmpty() || trees.putIfAbsent(id, element) != null) {
            throw error(element, "Missing or duplicate BehaviorTree ID: " + id);
          }
          if (children(element).size() != 1) {
            throw error(element, "BehaviorTree must contain exactly one root node");
          }
          break;
        case "include":
          attributes(element, Set.of("path"));
          if (directory == null || element.getAttribute("path").isEmpty()) {
            throw error(element, "Includes require a source file and a path");
          }
          Path included = directory.resolve(element.getAttribute("path")).toRealPath();
          if (loaded.size() >= 256) {
            throw error(element, "Too many included XML files");
          }
          if (!included.startsWith(allowedRoot)) {
            throw error(element, "Include is outside the input directory: " + included);
          }
          if (loading.contains(included)) {
            throw error(element, "Cyclic include: " + included);
          }
          if (loaded.add(included)) {
            loading.add(included);
            collect(read(included), included.getParent(), allowedRoot, loading, loaded);
            loading.remove(included);
          }
          break;
        case "TreeNodesModel":
          for (Element model : children(element)) {
            if (model.getTagName().equals("SubTree")) {
              Map<String, String> defaults = new LinkedHashMap<>();
              for (Element port : children(model)) {
                if (!Set.of("input_port", "output_port", "inout_port").contains(port.getTagName())) {
                  throw error(port, "Unknown subtree port declaration");
                }
                if (port.hasAttribute("default")) {
                  defaults.put(port.getAttribute("name"), port.getAttribute("default"));
                }
              }
              subtreeDefaults.put(model.getAttribute("ID"), defaults);
            }
          }
          break;
        default:
          throw error(element, "Unsupported root element");
      }
    }
  }

  private String generate(Element root) {
    String main = root.getAttribute("main_tree_to_execute");
    if (main.isEmpty() && trees.size() == 1) {
      main = trees.keySet().iterator().next();
    }
    Element tree = trees.get(main);
    if (tree == null) {
      throw error(root, "Select a valid main_tree_to_execute");
    }
    referencedTrees.add(main);
    while (treeRoots.size() < referencedTrees.size()) {
      String id = referencedTrees.stream().filter(key -> !treeRoots.containsKey(key)).findFirst().orElseThrow();
      treeRoots.put(id, compile(children(trees.get(id)).get(0), "0",
          new Scope()));
    }
    StringBuilder output = new StringBuilder();
    output.append("// Generated from BehaviorTree.CPP format 4. Control logic executes in ASL.\n");
    output.append("import java.lang.String;\nimport java.lang.Boolean;\n");
    output.append("() = ").append(action)
        .append("(String ?blackboard = \"{}\", long ?pollMillis = 10) {\n")
        .append("  ").append(EXECUTION).append(" !execution;\n")
        .append("  edu.tufts.hrilab.action.execution.Context !caller;\n")
        .append("  String !result = \"RUNNING\";\n")
        .append("  if (op:le(?pollMillis, 0)) { exit(FAIL, invalidBehaviorTreeInterval()); }\n")
        .append("  !caller = this:getParentContext();\n")
        .append("  !execution = op:newObject(\"").append(EXECUTION).append("\", !caller, ?blackboard);\n")
        .append("  op:invokeMethod(!execution, \"configure\", ").append(encoded(bindings.configuration())).append(");\n");
    output.append("  try {\n")
        .append("    while (op:invokeMethod(!result, \"equals\", \"RUNNING\")) {\n")
        .append("      !result = act:").append(method(nodes.get(0))).append("(!execution);\n")
        .append("      if (op:invokeMethod(!result, \"equals\", \"RUNNING\")) {\n")
        .append("        op:sleep(?pollMillis);\n")
        .append("      }\n")
        .append("    }\n")
        .append("    if (op:invokeMethod(!result, \"equals\", \"FAILURE\")) {\n")
        .append("      exit(FAIL, behaviorTreeFailed(").append(action).append("));\n")
        .append("    }\n")
        .append("  } finally {\n")
        .append("    act:").append(method(nodes.get(0))).append("_halt(!execution);\n")
        .append("    op:invokeMethod(!execution, \"close\");\n")
        .append("  }\n}\n");
    for (CompiledNode node : nodes) {
      output.append(emit(node));
      output.append(emitLifecycle(node));
      if (node.binding != null) {
        output.append(emitWorker(node));
      }
    }
    String asl = output.toString();
    parse(asl);
    return asl;
  }

  private CompiledNode compile(Element xml, String path, Scope scope) {
    if (nodes.size() >= 10000) {
      throw error(xml, "Behavior tree exceeds node limit");
    }
    String tag = xml.getTagName();
    CompiledNode node = new CompiledNode();
    node.id = nodes.size();
    node.xml = xml;
    node.path = path;
    node.scope = scope;
    node.kind = Set.of("Action", "Condition", "Control", "Decorator").contains(tag)
        ? xml.getAttribute("ID") : tag;
    nodes.add(node);
    Set<String> allowed = new HashSet<>(Set.of("name"));
    allowed.addAll(HOOKS);
    if (!node.kind.equals(tag)) {
      allowed.add("ID");
    }
    if (tag.equals("SubTree")) {
      String id = xml.getAttribute("ID");
      Element tree = trees.get(id);
      if (tree == null) {
        throw error(xml, "Unknown subtree: " + id);
      }
      Map<String, String> remaps = new LinkedHashMap<>(subtreeDefaults.getOrDefault(id, Map.of()));
      for (int i = 0; i < xml.getAttributes().getLength(); i++) {
        Node attr = xml.getAttributes().item(i);
        if (!Set.of("ID", "name", "_autoremap").contains(attr.getNodeName())
            && !HOOKS.contains(attr.getNodeName())) {
          if (attr.getNodeName().startsWith("_")) {
            throw error(xml, "Unsupported subtree attribute: " + attr.getNodeName());
          }
          remaps.put(attr.getNodeName(), attr.getNodeValue());
        }
      }
      boolean auto = false;
      if (xml.hasAttribute("_autoremap")) {
        String value = xml.getAttribute("_autoremap");
        if (!Set.of("true", "false").contains(value)) {
          throw error(xml, "_autoremap must be true or false");
        }
        auto = Boolean.parseBoolean(value);
      }
      if (!children(xml).isEmpty()) {
        throw error(xml, "SubTree cannot have children");
      }
      remaps.forEach((key, value) -> {
        if (!key.matches("[A-Za-z_][A-Za-z_0-9]*")) {
          throw error(xml, "Invalid subtree port: " + key);
        }
        scope.resolve(value);
      });
      node.subtree = id;
      node.remaps = remaps;
      node.auto = auto;
      referencedTrees.add(id);
      return node;
    }
    if (CONTROLS.contains(node.kind) || DECORATORS.contains(node.kind) || node.kind.matches("Switch[2-6]")) {
      switch (node.kind) {
        case "Parallel": allowed.addAll(Set.of("success_count", "failure_count")); break;
        case "RetryUntilSuccessful": allowed.add("num_attempts"); break;
        case "Repeat": allowed.add("num_cycles"); break;
        case "Timeout": allowed.add("msec"); break;
        case "Delay": allowed.add("delay_msec"); break;
        case "ParallelAll": allowed.add("max_failures"); break;
        case "RunOnce": allowed.add("then_skip"); break;
        case "Precondition": allowed.addAll(Set.of("if", "else")); break;
        case "TryCatch": allowed.add("catch_on_halt"); break;
        case "LoopInt": case "LoopBool": case "LoopDouble": case "LoopString":
          allowed.addAll(Set.of("queue", "value", "if_empty")); break;
        case "SkipUnlessUpdated": case "WaitValueUpdate": allowed.add("entry"); break;
        default: break;
      }
      if (node.kind.startsWith("Switch")) {
        allowed.add("variable");
        for (int i = 1; i <= Integer.parseInt(node.kind.substring(6)); i++) {
          allowed.add("case_" + i);
        }
      }
      List<Element> children = children(xml);
      if (children.isEmpty() || (DECORATORS.contains(node.kind) && children.size() != 1)) {
        throw error(xml, "Invalid child count for " + node.kind);
      }
      if ((Set.of("IfThenElse", "WhileDoElse").contains(node.kind)
          && children.size() != 2 && children.size() != 3)
          || node.kind.equals("TryCatch") && children.size() < 2
          || (node.kind.startsWith("Switch")
          && children.size() != Integer.parseInt(node.kind.substring(6)) + 1)) {
        throw error(xml, "Invalid child count for " + node.kind);
      }
      for (int i = 0; i < children.size(); i++) {
        node.children.add(compile(children.get(i), path + "/" + i, scope));
      }
    } else if (Set.of("AlwaysSuccess", "AlwaysFailure", "Script", "ScriptCondition",
        "SetBlackboard", "UnsetBlackboard", "Sleep", "WasEntryUpdated").contains(node.kind)) {
      if (!children(xml).isEmpty()) {
        throw error(xml, "A constant leaf cannot have children");
      }
      if (node.kind.startsWith("Script")) {
        allowed.add("code");
        if (!xml.hasAttribute("code")) {
          throw error(xml, "Missing script code");
        }
      }
      switch (node.kind) {
        case "SetBlackboard": allowed.addAll(Set.of("value", "output_key")); break;
        case "UnsetBlackboard": allowed.add("key"); break;
        case "Sleep": allowed.add("msec"); break;
        case "WasEntryUpdated": allowed.add("entry"); break;
        default: break;
      }
    } else {
      node.binding = bindings.get(node.kind);
      if (!children(xml).isEmpty()) {
        throw error(xml, "A leaf cannot have children");
      }
      node.binding.inputs.forEach(port -> allowed.add(port.name));
      node.binding.outputs.forEach(port -> allowed.add(port.name));
      validatePorts(node);
    }
    attributes(xml, allowed);
    return node;
  }

  private void validatePorts(CompiledNode node) {
    for (BehaviorTreeBindings.Port port : node.binding.inputs) {
      String value = node.xml.hasAttribute(port.name) ? node.xml.getAttribute(port.name) : port.defaultValue;
      if (value == null) {
        throw error(node.xml, "Missing required input: " + port.name);
      }
      node.scope.resolve(value);
    }
    for (BehaviorTreeBindings.Port port : node.binding.outputs) {
      if (node.xml.hasAttribute(port.name)) {
        String value = node.scope.resolve(node.xml.getAttribute(port.name));
        if (!value.startsWith("{") || !value.endsWith("}")) {
          throw error(node.xml, "Output must reference a blackboard entry: " + port.name);
        }
      }
    }
  }

  private String method(CompiledNode node) {
    return action + "_bt_" + node.id;
  }

  private String invoke(String method, String... args) {
    return "op:invokeMethod(?execution, \"" + method + "\""
        + (args.length == 0 ? "" : ", " + String.join(", ", args)) + ")";
  }

  private static String quote(String text) {
    if (text.indexOf('"') >= 0 || text.indexOf('\\') >= 0 || text.indexOf('\n') >= 0
        || text.indexOf('\r') >= 0 || text.indexOf('?') >= 0 || text.indexOf('!') >= 0) {
      throw new IllegalArgumentException("Value cannot be represented literally in ASL: " + text);
    }

    return "\"" + text + "\"";
  }

  private static String encoded(String text) {
    return quote(Base64.getEncoder().encodeToString(text.getBytes(StandardCharsets.UTF_8)));
  }

  private static String equal(String variable, String text) {
    return "(" + variable + " == \"" + text + "\")";
  }

  private String reset(CompiledNode node) {
    return "act:" + method(node) + "_reset(?execution);\n";
  }

  private String halt(CompiledNode node) {
    return "act:" + method(node) + "_halt(?execution);\n";
  }

  private String child(CompiledNode node) {
    return "?result = act:" + method(node) + "(?execution);\n";
  }

  private static String result(String value) {
    return "?result = \"" + value + "\";\n";
  }

  private String parameter(CompiledNode node, String name, String fallback) {
    String value = node.xml.hasAttribute(name) ? node.xml.getAttribute(name) : fallback;
    if (value == null) {
      throw error(node.xml, "Missing parameter: " + name);
    }
    value = node.scope.resolve(value);
    if (!(value.startsWith("{") && value.endsWith("}"))) {
      try {
        Integer.parseInt(value);
      } catch (NumberFormatException e) {
        throw error(node.xml, "Expected integer or blackboard reference for " + name);
      }
    }
    return invoke("encodedValue", encoded(value), "\"int\"");
  }

  private String emit(CompiledNode node) {
    StringBuilder body = new StringBuilder();
    String path = quote(node.path);
    if (node.subtree != null) {
      body.append("!subtree = ").append(invoke("subtree", path,
          encoded(new Gson().toJson(node.remaps)), Boolean.toString(node.auto))).append(";\n")
          .append("?result = act:").append(method(treeRoots.get(node.subtree))).append("(!subtree);\n");
    } else if (node.binding != null) {
      body.append(emitLeaf(node));
      if (node.binding.synchronous || node.xml.getTagName().equals("Condition")
          || !node.binding.result.equals("action")) {
        body.append("while (").append(equal("?result", "RUNNING")).append(") {\n")
            .append("op:sleep(1);\n?result = ")
            .append(invoke("status", path)).append(";\n}\n");
      }
    } else {
      switch (node.kind) {
        case "AlwaysSuccess": body.append(result("SUCCESS")); break;
        case "AlwaysFailure": body.append(result("FAILURE")); break;
        case "Script":
          body.append(scriptCall(node, "code", false)).append(result("SUCCESS"));
          break;
        case "ScriptCondition":
          body.append("!condition = ").append(scriptCall(node, "code", true))
              .append("if (op:equals(!condition, true)) {\n").append(result("SUCCESS"))
              .append("} else {\n").append(result("FAILURE")).append("}\n");
          break;
        case "Sequence": case "SequenceWithMemory": case "Fallback": case "AsyncSequence": case "AsyncFallback":
        case "ReactiveSequence": case "ReactiveFallback":
          emitOrdered(node, body);
          break;
        case "Parallel": emitParallel(node, body); break;
        case "ParallelAll": emitParallelAll(node, body); break;
        case "IfThenElse": case "WhileDoElse": emitConditional(node, body); break;
        case "TryCatch": emitTryCatch(node, body); break;
        case "LoopInt": case "LoopBool": case "LoopDouble": case "LoopString":
          emitQueueLoop(node, body); break;
        case "SetBlackboard":
          body.append(invoke("setBlackboard", encoded(required(node, "output_key")),
              encoded(required(node, "value")))).append(";\n").append(result("SUCCESS"));
          break;
        case "UnsetBlackboard":
          body.append(invoke("unsetBlackboard", encoded(required(node, "key"))))
              .append(";\n").append(result("SUCCESS"));
          break;
        case "Sleep":
          body.append("!index = ").append(invoke("integer", path, "\"durationInitialized\"")).append(";\n")
              .append("if (op:==(!index, 0)) {\n!limit = ").append(parameter(node, "msec", null)).append(";\n")
              .append("if (op:lt(!limit, 0)) { exit(FAIL, invalidBehaviorTreeDuration()); }\n")
              .append(invoke("integer", path, "\"duration\"", "!limit")).append(";\n")
              .append(invoke("integer", path, "\"durationInitialized\"", "1")).append(";\n}\n")
              .append("!limit = ").append(invoke("integer", path, "\"duration\"")).append(";\n")
              .append("!elapsed = ").append(invoke("elapsed", path)).append(";\n")
              .append("if (op:ge(!elapsed, !limit)) {\n").append(reset(node)).append(result("SUCCESS"))
              .append("} else {\n").append(result("RUNNING")).append("}\n");
          break;
        case "WasEntryUpdated": case "SkipUnlessUpdated": case "WaitValueUpdate":
          body.append("!condition = ").append(invoke("entryUpdated", path, encoded(required(node, "entry"))))
              .append(";\nif (op:equals(!condition, false)) {\n")
              .append(result(node.kind.equals("WasEntryUpdated") ? "FAILURE" :
                  node.kind.equals("SkipUnlessUpdated") ? "SKIPPED" : "RUNNING")).append("return;\n}\n");
          if (node.kind.equals("WasEntryUpdated")) {
            body.append(result("SUCCESS"));
          } else {
            body.append(child(node.children.get(0)));
          }
          break;
        case "RunOnce":
          body.append("!cached = ").append(invoke("persistentText", path)).append(";\n")
              .append("if (~").append(equal("!cached", "IDLE")).append(") {\n")
              .append("if (").append(port(node, "then_skip", "true", "boolean")).append(") {\n")
              .append(result("SKIPPED")).append("} else {\n?result = op:bind(!cached);\n}\nreturn;\n}\n")
              .append(child(node.children.get(0)))
              .append("if (").append(completed("?result")).append(") {\n")
              .append(invoke("persistentText", path, "?result")).append(";\n")
              .append(reset(node)).append("}\n");
          break;
        case "Precondition":
          body.append("!index = ").append(invoke("integer", path, "\"conditionPassed\"")).append(";\n")
              .append("if (op:==(!index, 0)) {\n!condition = ").append(scriptCall(node, "if", true))
              .append("if (op:equals(!condition, false)) {\n?result = ").append(port(node, "else", "FAILURE", "String"))
              .append(";\n").append(invoke("validateStatus", "?result")).append(";\nreturn;\n}\n")
              .append(invoke("integer", path, "\"conditionPassed\"", "1")).append(";\n}\n")
              .append(child(node.children.get(0))).append("if (").append(completed("?result"))
              .append(") {\n").append(reset(node)).append("}\n");
          break;
        case "RetryUntilSuccessful": case "Repeat": emitLoop(node, body); break;
        case "Timeout": case "Delay":
          body.append("!index = ").append(invoke("integer", path, "\"durationInitialized\"")).append(";\n")
              .append("if (op:==(!index, 0)) {\n")
              .append("!limit = ").append(parameter(node,
              node.kind.equals("Timeout") ? "msec" : "delay_msec", null)).append(";\n")
              .append("if (op:lt(!limit, 0)) { exit(FAIL, invalidBehaviorTreeDuration()); }\n")
              .append(invoke("integer", path, "\"duration\"", "!limit")).append(";\n")
              .append(invoke("integer", path, "\"durationInitialized\"", "1")).append(";\n")
              .append("} else {\n!limit = ").append(invoke("integer", path, "\"duration\"")).append(";\n}\n")
              .append("!elapsed = ").append(invoke("elapsed", path)).append(";\n")
              .append("if (");
          if (node.kind.equals("Timeout")) {
            body.append("op:gt(!limit, 0) && ");
          }
          body.append("op:ge(!elapsed, !limit)) {\n");
          if (node.kind.equals("Timeout")) {
            body.append(reset(node)).append(result("FAILURE"));
          } else {
            body.append(child(node.children.get(0)));
          }
          body.append("} else {\n");
          if (node.kind.equals("Timeout")) {
            body.append(child(node.children.get(0)));
          } else {
            body.append(result("RUNNING"));
          }
          body.append("}\nif (~").append(equal("?result", "RUNNING")).append(") {\n")
              .append(reset(node)).append("}\n");
          break;
        default:
          if (node.kind.startsWith("Switch")) {
            emitSwitch(node, body);
            break;
          }
          body.append(child(node.children.get(0)));
          body.append("if (").append(equal("?result", "SKIPPED")).append(") { return; }\n");
          body.append("if (~").append(equal("?result", "RUNNING")).append(") {\n")
              .append(reset(node));
          switch (node.kind) {
            case "Inverter":
              body.append("if (").append(equal("?result", "SUCCESS")).append(") {\n")
                  .append(result("FAILURE")).append("} else {\n").append(result("SUCCESS")).append("}\n");
              break;
            case "ForceSuccess": body.append(result("SUCCESS")); break;
            case "ForceFailure": body.append(result("FAILURE")); break;
            case "KeepRunningUntilFailure":
              body.append("if (").append(equal("?result", "SUCCESS")).append(") {\n")
                  .append(result("RUNNING")).append("}\n");
              break;
            default: throw error(node.xml, "Unsupported node: " + node.kind);
          }
          body.append("}\n");
      }
    }
    StringBuilder locals = new StringBuilder("edu.tufts.hrilab.action.execution.AslTask !task;\n");
    if (node.binding != null) {
      for (int i = 0; i < node.binding.inputs.size(); i++) {
        locals.append(BehaviorTreeExecution.portType(node.binding.inputs.get(i).type).getCanonicalName())
            .append(" !input").append(i).append(";\n");
      }
    }
    String remember = invoke("text", quote(node.path + ":previous"), "?result") + ";\n";
    String instructions = body.toString().replace("return;", remember + "return;");
    org.w3c.dom.Node definition = node.xml;
    while (definition != null && !definition.getNodeName().equals("BehaviorTree")) definition = definition.getParentNode();
    String tree = definition instanceof Element ? ((Element) definition).getAttribute("ID") : "";
    String file = node.xml.getOwnerDocument().getDocumentURI();
    String origin = (file == null ? "behavior-tree.xml" : file) + "#" + tree + ":" + node.path + " <" + node.kind + ">";
    String marker = Base64.getEncoder().encodeToString(origin.getBytes(StandardCharsets.UTF_8));
    return "\n// @diarc-origin " + marker + "\n(String ?result = \"RUNNING\") = " + method(node) + "_tick(" + EXECUTION + " ?execution) {\n"
        + locals
        + "int !index;\nint !count;\nint !limit;\nint !successes = 0;\nint !failures = 0;\n"
        + "int !skipped = 0;\nint !selected;\nboolean !negative;\nBoolean !condition;\n"
        + EXECUTION + " !subtree;\n"
        + "int !successLimit;\nint !failureLimit;\nlong !elapsed;\nString !cached;\nString !previous;\n"
        + instructions + remember + "}\n";
  }

  private static String completed(String variable) {
    return equal(variable, "SUCCESS") + " || " + equal(variable, "FAILURE");
  }

  private String port(CompiledNode node, String name, String fallback, String type) {
    String value = node.xml.hasAttribute(name) ? node.xml.getAttribute(name) : fallback;
    if (value == null) {
      throw error(node.xml, "Missing parameter: " + name);
    }
    return invoke("encodedValue", encoded(node.scope.resolve(value)), quote(type));
  }

  private String required(CompiledNode node, String name) {
    if (!node.xml.hasAttribute(name)) {
      throw error(node.xml, "Missing parameter: " + name);
    }
    return node.scope.resolve(node.xml.getAttribute(name));
  }

  private String scriptCall(CompiledNode node, String attribute, boolean predicate) {
    if (!node.xml.hasAttribute(attribute)) {
      throw error(node.xml, "Missing script: " + attribute);
    }
    return "act:" + method(node) + "_script" + attribute + "(?execution);\n";
  }

  private String emitLifecycle(CompiledNode node) {
    String state = quote(node.path + ":state");
    StringBuilder body = new StringBuilder("\n(String ?result = \"RUNNING\") = ")
        .append(method(node)).append("(").append(EXECUTION).append(" ?execution) {\n")
        .append("Boolean !condition;\nString !state;\n")
        .append("!state = ").append(invoke("text", state)).append(";\n");
    for (String hook : List.of("_failureIf", "_successIf", "_skipIf", "_while")) {
      if (!node.xml.hasAttribute(hook)) {
        continue;
      }
      body.append("if (").append(equal("!state", "IDLE"));
      if (hook.equals("_while")) {
        body.append(" || ").append(equal("!state", "RUNNING"));
      }
      body.append(") {\n!condition = ").append(scriptCall(node, hook, true))
          .append("if (op:equals(!condition, ").append(hook.equals("_while") ? "false" : "true").append(")) {\n");
      if (hook.equals("_while")) {
        body.append("if (").append(equal("!state", "RUNNING")).append(") {\n").append(halt(node)).append("}\n");
      }
      body.append(result(hook.equals("_failureIf") ? "FAILURE" :
          hook.equals("_successIf") ? "SUCCESS" : "SKIPPED"))
          .append("?result = act:").append(method(node)).append("_post(?execution, ?result);\nreturn;\n}\n}\n");
    }
    body.append(invoke("text", state, "\"RUNNING\"")).append(";\n")
        .append("?result = act:").append(method(node)).append("_tick(?execution);\n")
        .append("?result = act:").append(method(node)).append("_post(?execution, ?result);\n}\n");
    body.append("\n(String ?result) = ").append(method(node)).append("_post(")
        .append(EXECUTION).append(" ?execution, String ?outcome) {\n")
        .append("?result = op:bind(?outcome);\n");
    for (String hook : List.of("_onSuccess", "_onFailure", "_post")) {
      if (node.xml.hasAttribute(hook)) {
        body.append("if (").append(hook.equals("_post") ? completed("?result")
            : equal("?result", hook.equals("_onSuccess") ? "SUCCESS" : "FAILURE")).append(") {\n")
            .append(scriptCall(node, hook, false)).append("}\n");
      }
    }
    body.append("if (~").append(equal("?result", "SKIPPED")).append(") {\n")
        .append(invoke("text", state, "?result")).append(";\n}\n")
        .append(invoke("text", quote(node.path + ":previous"), "?result")).append(";\n}\n");
    body.append("\n() = ").append(method(node)).append("_halt(").append(EXECUTION).append(" ?execution) {\n")
        .append("String !state;\nString !cleanup;\nint !catching;\n!state = ").append(invoke("text", state)).append(";\n");
    if (node.kind.equals("TryCatch")) {
      body.append("!catching = ").append(invoke("integer", quote(node.path), "\"catching\"")).append(";\n")
          .append("if (").append(equal("!state", "RUNNING")).append(" && op:==(!catching, 0) && ")
          .append(port(node, "catch_on_halt", "false", "boolean")).append(") {\n");
      node.children.subList(0, node.children.size() - 1).forEach(c -> body.append(halt(c)));
      body.append("!cleanup = act:").append(method(node.children.get(node.children.size() - 1)))
          .append("(?execution);\n}\n");
    }
    body.append(reset(node));
    if (node.xml.hasAttribute("_onHalted")) {
      body.append("if (").append(equal("!state", "RUNNING")).append(") {\n")
          .append(scriptCall(node, "_onHalted", false)).append("}\n");
    }
    body.append("}\n\n() = ").append(method(node)).append("_reset(").append(EXECUTION)
        .append(" ?execution) {\n");
    if (node.subtree != null) {
      body.append(EXECUTION).append(" !subtree;\n")
          .append("if (").append(invoke("hasSubtree", quote(node.path))).append(") {\n")
          .append("!subtree = ").append(invoke("existingSubtree", quote(node.path))).append(";\n")
          .append("act:").append(method(treeRoots.get(node.subtree))).append("_halt(!subtree);\n}\n");
    }
    for (CompiledNode child : node.children) {
      body.append(halt(child));
    }
    body.append(invoke("reset", quote(node.path))).append(";\n}\n");
    Set<String> scripts = new HashSet<>(HOOKS);
    if (node.kind.startsWith("Script")) {
      scripts.add("code");
    }
    if (node.kind.equals("Precondition")) {
      scripts.add("if");
    }
    for (String attribute : new java.util.TreeSet<>(scripts)) {
      if (node.xml.hasAttribute(attribute)) {
        boolean predicate = Set.of("_failureIf", "_successIf", "_skipIf", "_while", "if").contains(attribute)
            || node.kind.equals("ScriptCondition") && attribute.equals("code");
        String source = node.xml.getAttribute(attribute);
        body.append("\n(Boolean ?answer) = ").append(method(node)).append("_script").append(attribute)
            .append("(").append(EXECUTION).append(" ?execution) {\n");
        if (source.startsWith("{") && source.endsWith("}")) {
          node.scope.resolve(source);
          body.append("edu.tufts.hrilab.action.execution.Context !caller;\n")
              .append("!caller = this:getParentContext();\n")
              .append(invoke("appendScript", "!caller", encoded(source), Boolean.toString(predicate))).append(";\n");
        } else {
          body.append("java.lang.Object !scriptResult;\n")
              .append(BehaviorTreeScript.compile(source, "s", "!scriptResult", false))
              .append("?answer = ").append(predicate ? invoke("scriptTruthy", "!scriptResult")
                  : "op:equals(true, true)").append(";\n");
        }
        body.append("}\n");
      }
    }
    return body.toString();
  }

  private String emitLeaf(CompiledNode node) {
    String path = quote(node.path);
    StringBuilder body = new StringBuilder();
    if (node.binding.synchronous || node.xml.getTagName().equals("Condition")
        || !node.binding.result.equals("action")) {
      body.append("!previous = ").append(invoke("text", quote(node.path + ":previous"))).append(";\n")
          .append("if (").append(completed("!previous")).append(") {\n").append(reset(node)).append("}\n");
    }
    body.append("if (~").append(invoke("started", path)).append(") {\n");
    List<String> inputs = new ArrayList<>();
    for (int i = 0; i < node.binding.inputs.size(); i++) {
      BehaviorTreeBindings.Port port = node.binding.inputs.get(i);
      String value = node.xml.hasAttribute(port.name) ? node.xml.getAttribute(port.name) : port.defaultValue;
      body.append("!input").append(i).append(" = ")
          .append(invoke("encodedValue", encoded(node.scope.resolve(value)), quote(port.type))).append(";\n");
      inputs.add("!input" + i);
    }
    body.append("!task = async {\nact:").append(method(node)).append("_execute(?execution");
    for (String input : inputs) {
      body.append(", ").append(input);
    }
    body.append(");\n}\n").append(invoke("attachTask", path, "!task")).append(";\n}\n")
        .append("?result = ").append(invoke("status", path)).append(";\n");
    return body.toString();
  }

  private String emitWorker(CompiledNode node) {
    // An action scope preserves return bindings; direct async children snapshot them.
    StringBuilder body = new StringBuilder("\n() = ").append(method(node))
        .append("_execute(").append(EXECUTION).append(" ?execution");
    List<String> inputs = new ArrayList<>();
    for (int i = 0; i < node.binding.inputs.size(); i++) {
      body.append(", ").append(BehaviorTreeExecution.portType(node.binding.inputs.get(i).type).getCanonicalName())
          .append(" ?input").append(i);
      inputs.add("?input" + i);
    }
    body.append(") {\n");
    List<String> outputs = new ArrayList<>();
    for (int i = 0; i < node.binding.outputs.size(); i++) {
      body.append(BehaviorTreeExecution.portType(node.binding.outputs.get(i).type).getCanonicalName())
          .append(" !output").append(i).append(";\n");
      outputs.add("!output" + i);
    }
    if (!node.binding.result.equals("action") && outputs.isEmpty()) {
      body.append(node.binding.result.equals("boolean") ? "Boolean" :
          "edu.tufts.hrilab.action.justification.Justification").append(" !answer;\n");
      outputs.add("!answer");
    }
    if (!outputs.isEmpty()) {
      body.append(outputs.size() == 1 ? outputs.get(0) : "(" + String.join(", ", outputs) + ")").append(" = ");
    }
    body.append("act:").append(node.binding.action).append("(").append(String.join(", ", inputs)).append(");\n");
    for (int i = 0; i < node.binding.outputs.size(); i++) {
      BehaviorTreeBindings.Port port = node.binding.outputs.get(i);
      if (node.xml.hasAttribute(port.name)) {
        String expression = node.scope.resolve(node.xml.getAttribute(port.name));
        body.append(invoke("write", quote(expression.substring(1, expression.length() - 1)),
            "!output" + i)).append(";\n");
      }
    }
    if (!node.binding.result.equals("action")) {
      String answer = outputs.get(0);
      String condition = node.binding.result.equals("boolean")
          ? "op:equals(" + answer + ", true)"
          : "op:invokeMethod(" + answer + ", \"getValue\")";
      body.append("if (").append(condition).append(") {\n")
          .append(invoke("text", quote(node.path + ":outcome"), "\"SUCCESS\"")).append(";\n} else {\n")
          .append(invoke("text", quote(node.path + ":outcome"), "\"FAILURE\"")).append(";\n}\n");
    }
    body.append("}\n");
    return body.toString();
  }

  private void emitOrdered(CompiledNode node, StringBuilder body) {
    boolean fallback = node.kind.endsWith("Fallback");
    boolean reactive = node.kind.startsWith("Reactive");
    boolean memory = node.kind.equals("SequenceWithMemory");
    boolean asynchronous = node.kind.startsWith("Async");
    String advance = fallback ? "FAILURE" : "SUCCESS";
    String stop = fallback ? "SUCCESS" : "FAILURE";
    String path = quote(node.path);
    body.append("!index = ").append(memory ? invoke("memory", path) : invoke("integer", path, "\"cursor\""))
        .append(";\n!skipped = ").append(invoke("integer", path, "\"skipped\"")).append(";\n");
    if (reactive) {
      body.append("!skipped = op:+(0, 0);\n");
    }
    for (int i = 0; i < node.children.size(); i++) {
      CompiledNode current = node.children.get(i);
      if (!reactive) {
        body.append("if (op:le(!index, ").append(i).append(")) {\n");
      }
      if (memory || asynchronous) {
        body.append("!previous = ").append(invoke("text", quote(current.path + ":previous"))).append(";\n");
      }
      body.append(child(current));
      body.append("if (").append(equal("?result", "RUNNING")).append(") {\n");
      if (reactive) {
        for (CompiledNode other : node.children) {
          if (other != current) {
            body.append(halt(other));
          }
        }
      } else {
        body.append(memory ? invoke("memory", path, Integer.toString(i))
            : invoke("integer", path, "\"cursor\"", Integer.toString(i))).append(";\n");
      }
      body.append("return;\n}\n");
      body.append("if (").append(equal("?result", stop)).append(") {\n");
      if (memory) {
        body.append(invoke("memory", path, Integer.toString(i))).append(";\n");
      }
      body.append(reset(node)).append("return;\n}\n");
      body.append("if (").append(equal("?result", "SKIPPED")).append(") {\n")
          .append("!skipped = op:+(!skipped, 1);\n")
          .append(invoke("integer", path, "\"skipped\"", "!skipped")).append(";\n}\n")
          .append(halt(current));
      if (memory && i + 1 < node.children.size()) {
        body.append(invoke("memory", path, Integer.toString(i + 1))).append(";\n")
            .append("if (").append(equal("!previous", "IDLE")).append(" && ")
            .append(equal("?result", "SUCCESS")).append(") {\n")
            .append(result("RUNNING")).append("return;\n}\n");
      }
      if (asynchronous && i + 1 < node.children.size()) {
        body.append(invoke("integer", path, "\"cursor\"", Integer.toString(i + 1))).append(";\n")
            .append("if (").append(equal("!previous", "IDLE")).append(" && ")
            .append(equal("?result", advance)).append(") {\n")
            .append(result("RUNNING")).append("return;\n}\n");
      }
      if (!reactive) {
        body.append("}\n");
      }
    }
    if (memory) {
      body.append(invoke("memory", path, "0")).append(";\n");
    }
    body.append(reset(node)).append("if (op:==(!skipped, ").append(node.children.size())
        .append(")) {\n").append(result("SKIPPED")).append("} else {\n").append(result(advance)).append("}\n");
  }

  private void emitLoop(CompiledNode node, StringBuilder body) {
    boolean retry = node.kind.equals("RetryUntilSuccessful");
    String path = quote(node.path);
    body.append("!limit = ").append(parameter(node, retry ? "num_attempts" : "num_cycles", null)).append(";\n")
        .append("if (op:lt(!limit, -1)) { exit(FAIL, invalidBehaviorTreeCount()); }\n")
        .append("!count = ").append(invoke("integer", path, "\"count\"")).append(";\n")
        .append("while (true) {\n")
        .append("if (~op:==(!limit, -1) && op:ge(!count, !limit)) {\n").append(reset(node))
        .append(result(retry ? "FAILURE" : "SUCCESS")).append("return;\n}\n")
        .append("!previous = ").append(invoke("text", quote(node.children.get(0).path + ":previous"))).append(";\n")
        .append(child(node.children.get(0)))
        .append("if (").append(equal("?result", "SKIPPED")).append(") { return; }\n")
        .append("if (").append(equal("?result", "RUNNING")).append(") { return; }\n")
        .append("if (").append(equal("?result", retry ? "SUCCESS" : "FAILURE")).append(") {\n")
        .append(reset(node)).append("return;\n}\n")
        .append(reset(node.children.get(0)))
        .append("!count = op:+(!count, 1);\n")
        .append(invoke("integer", path, "\"count\"", "!count")).append(";\n");
    if (retry) {
      body.append("!limit = ").append(parameter(node, "num_attempts", null)).append(";\n");
    }
    body.append("if (").append(equal("!previous", "IDLE")).append(") {\n")
        .append("if (op:==(!limit, -1) || op:lt(!count, !limit)) {\n")
        .append(result("RUNNING")).append("return;\n}\n}\n}\n");
  }

  private void emitParallel(CompiledNode node, StringBuilder body) {
    int count = node.children.size();
    body.append("!successLimit = ").append(parameter(node, "success_count", "-1")).append(";\n")
        .append("!negative = op:lt(!successLimit, 0);\n")
        .append("!failureLimit = ").append(parameter(node, "failure_count", "1")).append(";\n");
    for (String variable : List.of("!successLimit", "!failureLimit")) {
      body.append("if (op:lt(").append(variable).append(", 0)) {\n")
          .append(variable).append(" = op:+(").append(variable).append(", ").append(count + 1).append(");\n")
          .append("if (op:lt(").append(variable).append(", 0)) { ").append(variable)
          .append(" = op:+(0, 0); }\n}\n")
          .append("if (op:gt(").append(variable).append(", ").append(count)
          .append(")) { exit(FAIL, invalidParallelThreshold()); }\n");
    }
    for (CompiledNode child : node.children) {
      body.append("!cached = ").append(invoke("text", quote(child.path))).append(";\n")
          .append("if (").append(equal("!cached", "SUCCESS"))
          .append(") { !successes = op:+(!successes, 1); }\n")
          .append("if (").append(equal("!cached", "FAILURE"))
          .append(") { !failures = op:+(!failures, 1); }\n");
    }
    for (CompiledNode child : node.children) {
      body.append("!cached = ").append(invoke("text", quote(child.path))).append(";\n")
          .append("if (").append(equal("!cached", "IDLE")).append(") {\n")
          .append(child(child))
          .append("if (").append(equal("?result", "SKIPPED")).append(") { !skipped = op:+(!skipped, 1); }\n")
          .append("if (").append(completed("?result")).append(") {\n")
          .append(invoke("text", quote(child.path), "?result")).append(";\n")
          .append("if (").append(equal("?result", "SUCCESS"))
          .append(") { !successes = op:+(!successes, 1); }\n")
          .append("if (").append(equal("?result", "FAILURE"))
          .append(") { !failures = op:+(!failures, 1); }\n}\n")
          .append("} else { ?result = op:bind(!cached); }\n")
          .append("!count = op:+(!successes, !skipped);\n")
          .append("if (op:ge(!successes, !successLimit) || ")
          .append("(op:equals(!negative, true) && op:ge(!count, !successLimit))) {\n")
          .append(reset(node)).append(result("SUCCESS")).append("return;\n}\n")
          .append("!count = op:-(").append(count).append(", !failures);\n")
          .append("if (op:lt(!count, !successLimit)) {\n")
          .append(reset(node)).append(result("FAILURE")).append("return;\n}\n")
          .append("if (op:==(!failures, !failureLimit)) {\n")
          .append(reset(node)).append(result("FAILURE")).append("return;\n}\n");
    }
    body.append("if (op:==(!skipped, ").append(count).append(")) {\n")
        .append(result("SKIPPED")).append("} else {\n").append(result("RUNNING")).append("}\n");
  }

  private void emitConditional(CompiledNode node, StringBuilder body) {
    boolean reactive = node.kind.equals("WhileDoElse");
    String path = quote(node.path);
    body.append("!index = ").append(invoke("integer", path, "\"branch\"")).append(";\n");
    if (!reactive) {
      body.append("if (op:==(!index, 0)) {\n");
    }
    body.append(child(node.children.get(0)))
        .append("if (").append(equal("?result", "RUNNING")).append(") { return; }\n")
        .append("if (").append(equal("?result", "SKIPPED"))
        .append(") { exit(FAIL, skippedBranchCondition()); }\n")
        .append("if (").append(equal("?result", "SUCCESS")).append(") { !index = op:+(1, 0); }")
        .append(" else { !index = op:+(2, 0); }\n")
        .append(invoke("integer", path, "\"branch\"", "!index")).append(";\n");
    if (!reactive) {
      body.append("}\n");
    }
    if (reactive && node.children.size() == 3) {
      body.append("if (op:==(!index, 1)) {\n").append(halt(node.children.get(2)))
          .append("} else {\n").append(halt(node.children.get(1))).append("}\n");
    }
    body.append("if (op:==(!index, 1)) {\n").append(child(node.children.get(1))).append("} else {\n");
    body.append(node.children.size() == 3 ? child(node.children.get(2)) : result("FAILURE"));
    body.append("}\nif (~").append(equal("?result", "RUNNING")).append(") {\n")
        .append(reset(node)).append("}\n");
  }

  private void emitTryCatch(CompiledNode node, StringBuilder body) {
    String path = quote(node.path);
    int count = node.children.size() - 1;
    body.append("!index = ").append(invoke("integer", path, "\"cursor\"")).append(";\n")
        .append("!count = ").append(invoke("integer", path, "\"catching\"")).append(";\n");
    for (int i = 0; i < count; i++) {
      body.append("if (op:==(!count, 0) && op:le(!index, ").append(i).append(")) {\n")
          .append(child(node.children.get(i)))
          .append("if (").append(equal("?result", "RUNNING")).append(") { return; }\n")
          .append("if (").append(equal("?result", "FAILURE")).append(") {\n")
          .append(reset(node)).append("!count = op:+(1, 0);\n")
          .append(invoke("integer", path, "\"catching\"", "1")).append(";\n")
          .append("} else {\n")
          .append(invoke("integer", path, "\"cursor\"", Integer.toString(i + 1))).append(";\n")
          .append("if (").append(equal("?result", "SKIPPED")).append(") {\n")
          .append("!skipped = ").append(invoke("integer", path, "\"skipped\"")).append(";\n")
          .append("!skipped = op:+(!skipped, 1);\n")
          .append(invoke("integer", path, "\"skipped\"", "!skipped")).append(";\n}\n}\n}\n");
    }
    body.append("if (op:==(!count, 0)) {\n")
        .append("!skipped = ").append(invoke("integer", path, "\"skipped\"")).append(";\n")
        .append(reset(node)).append("if (op:==(!skipped, ").append(count).append(")) {\n")
        .append(result("SKIPPED")).append("} else {\n").append(result("SUCCESS")).append("}\nreturn;\n}\n")
        .append(child(node.children.get(count))).append("if (~").append(equal("?result", "RUNNING"))
        .append(") {\n").append(reset(node)).append(result("FAILURE")).append("}\n");
  }

  private void emitQueueLoop(CompiledNode node, StringBuilder body) {
    String path = quote(node.path);
    String type = Map.of("LoopInt", "int", "LoopBool", "boolean", "LoopDouble", "double",
        "LoopString", "String").get(node.kind);
    String output = required(node, "value");
    if (!output.startsWith("{") || !output.endsWith("}")) {
      throw error(node.xml, "Loop value must be a blackboard reference");
    }
    body.append("!index = ").append(invoke("integer", path, "\"childRunning\"")).append(";\n")
        .append("if (op:==(!index, 0)) {\n!condition = ")
        .append(invoke("queuePop", path, encoded(required(node, "queue")), encoded(output), quote(type)))
        .append(";\nif (op:equals(!condition, false)) {\n?result = ")
        .append(port(node, "if_empty", "SUCCESS", "String")).append(";\n")
        .append(invoke("validateStatus", "?result")).append(";\n").append(reset(node)).append("return;\n}\n}\n")
        .append(child(node.children.get(0))).append("if (").append(equal("?result", "RUNNING")).append(") {\n")
        .append(invoke("integer", path, "\"childRunning\"", "1")).append(";\n")
        .append("} else {\n").append(invoke("integer", path, "\"childRunning\"", "0")).append(";\n")
        .append("if (").append(completed("?result")).append(") {\n")
        .append(reset(node.children.get(0))).append("}\n}\n")
        .append("if (").append(equal("?result", "FAILURE")).append(") {\n")
        .append(reset(node)).append("return;\n}\n").append(result("RUNNING"));
  }

  private void emitSwitch(CompiledNode node, StringBuilder body) {
    int cases = node.children.size() - 1;
    body.append("!selected = op:+(").append(cases).append(", 0);\n");
    if (node.xml.hasAttribute("variable")) {
      for (int i = 1; i <= cases; i++) {
        if (node.xml.hasAttribute("case_" + i)) {
          body.append("if (op:==(!selected, ").append(cases).append(")) {\n")
              .append("if (").append(invoke("switchMatches",
                  encoded(node.xml.getAttribute("variable")), encoded(node.xml.getAttribute("case_" + i))))
              .append(") { !selected = op:+(").append(i - 1).append(", 0); }\n}\n");
        }
      }
    }
    body.append("!index = ").append(invoke("integer", quote(node.path), "\"selected\"")).append(";\n");
    for (int i = 0; i <= cases; i++) {
      body.append("if (~op:==(!selected, ").append(i).append(")) {\n")
          .append(halt(node.children.get(i))).append("}\n");
    }
    body.append(invoke("integer", quote(node.path), "\"selected\"", "!selected")).append(";\n");
    for (int i = 0; i <= cases; i++) {
      body.append("if (op:==(!selected, ").append(i).append(")) {\n")
          .append(child(node.children.get(i))).append("}\n");
    }
    body.append("if (").append(completed("?result")).append(") {\n").append(reset(node)).append("}\n");
  }

  private void emitParallelAll(CompiledNode node, StringBuilder body) {
    int count = node.children.size();
    body.append("!failureLimit = ").append(parameter(node, "max_failures", "1")).append(";\n")
        .append("if (op:lt(!failureLimit, 0)) {\n!failureLimit = op:+(!failureLimit, ")
        .append(count + 1).append(");\nif (op:lt(!failureLimit, 0)) { !failureLimit = op:+(0, 0); }\n}\n")
        .append("if (op:gt(!failureLimit, ").append(count)
        .append(")) { exit(FAIL, invalidParallelThreshold()); }\n");
    for (CompiledNode child : node.children) {
      body.append("!cached = ").append(invoke("text", quote(child.path))).append(";\n")
          .append("if (").append(equal("!cached", "IDLE")).append(") {\n").append(child(child))
          .append("if (").append(completed("?result")).append(") {\n")
          .append(invoke("text", quote(child.path), "?result")).append(";\n}\n")
          .append("} else { ?result = op:bind(!cached); }\n")
          .append("if (").append(equal("?result", "SKIPPED"))
          .append(") { !skipped = op:+(!skipped, 1); }\n")
          .append("if (").append(equal("?result", "SUCCESS"))
          .append(") { !successes = op:+(!successes, 1); }\n")
          .append("if (").append(equal("?result", "FAILURE"))
          .append(") { !failures = op:+(!failures, 1); }\n");
    }
    body.append("if (op:==(!skipped, ").append(count).append(")) {\n")
        .append(result("SKIPPED")).append("return;\n}\n")
        .append("!count = op:+(!skipped, !successes);\n!count = op:+(!count, !failures);\n")
        .append("if (op:ge(!count, ").append(count).append(")) {\n").append(reset(node))
        .append("if (op:ge(!failures, !failureLimit)) {\n").append(result("FAILURE"))
        .append("} else {\n").append(result("SUCCESS")).append("}\n} else {\n")
        .append(result("RUNNING")).append("}\n");
  }

  private static List<Element> children(Element parent) {
    List<Element> result = new ArrayList<>();
    for (Node node = parent.getFirstChild(); node != null; node = node.getNextSibling()) {
      if (node instanceof Element) {
        result.add((Element) node);
      } else if (node.getNodeType() == Node.TEXT_NODE && !node.getTextContent().isBlank()) {
        throw error(parent, "Unexpected executable text");
      }
    }
    return result;
  }

  private static void attributes(Element element, Set<String> allowed) {
    for (int i = 0; i < element.getAttributes().getLength(); i++) {
      String name = element.getAttributes().item(i).getNodeName();
      if (!allowed.contains(name)) {
        throw error(element, "Unsupported attribute: " + name);
      }
    }
  }

  private static IllegalArgumentException error(Element element, String message) {
    return new IllegalArgumentException("<" + element.getTagName() + " ID=\""
        + element.getAttribute("ID") + "\">: " + message);
  }

  /** Convert a file without starting DIARC: input.xml bindings.json actionName output.asl. */
  public static void main(String[] args) throws IOException {
    if (args.length != 4) {
      throw new IllegalArgumentException("Usage: BehaviorTreeConverter input.xml bindings.json actionName output.asl");
    }
    BehaviorTreeBindings bindings;
    try (java.io.Reader reader = Files.newBufferedReader(Path.of(args[1]))) {
      bindings = BehaviorTreeBindings.read(reader);
    }
    String asl = new BehaviorTreeConverter(bindings).convert(Path.of(args[0]), args[2]);
    Files.writeString(Path.of(args[3]), asl);
  }
}

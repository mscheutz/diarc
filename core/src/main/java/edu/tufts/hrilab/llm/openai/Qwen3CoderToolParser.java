package edu.tufts.hrilab.llm.openai;
import com.google.gson.Gson;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

// (Manually modified AI generated language conversion - found a lot of things even in the original file were defined
//  but not used anywhere. Proceed with caution)

// Java port of vllm/tool_parsers/qwen3coder_tool_parser.py (Qwen3CoderToolParser).
// Parses Qwen3's XML-tag tool-call format into OpenAI-spec DeltaToolCall deltas.
// Wire format from the model: <tool_call><function=NAME><parameter=KEY>VAL</parameter>...</function></tool_call>
//
// One deliberate divergence from the Python: the Python parser detects tag boundaries via
// BOTH text matching AND tokenizer token-id checks (it runs inside vLLM with vocab access).
// This Java port runs client-side, sees only the SSE wire `delta.content` text, and has no
// tokenizer. It therefore uses text matching only. In normal streaming this works because
// <tool_call>/<function=>/<parameter=> are single special tokens in Qwen3's vocab and arrive
// intact in their own SSE deltas. If the tags ever get split across deltas (e.g. a proxy
// re-chunks the stream, or a non-Qwen model is used) the text-only check will miss the
// boundary and the fragment falls through as plain `content`.
public class Qwen3CoderToolParser {

  public static final class DeltaFunctionCall {
    public String name;
    public String arguments;
    public DeltaFunctionCall() {}
    public DeltaFunctionCall(String name, String arguments) { this.name = name; this.arguments = arguments; }
    public String toString() {
      StringBuilder sb = new StringBuilder("DeltaFunctionCall(");
      if (name != null) sb.append("name='").append(name).append("'");
      if (arguments != null) { if (name != null) sb.append(", "); sb.append("arguments='").append(arguments).append("'"); }
      return sb.append(")").toString();
    }
  }

  public static final class DeltaToolCall {
    public Integer index;
    public String id;
    public String type;
    public DeltaFunctionCall function;
    public DeltaToolCall() {}
    public String toString() {
      StringBuilder sb = new StringBuilder("DeltaToolCall(index=").append(index);
      if (id != null) sb.append(", id='").append(id).append("'");
      if (type != null) sb.append(", type='").append(type).append("'");
      if (function != null) sb.append(", function=").append(function);
      return sb.append(")").toString();
    }
  }

  public static final class DeltaMessage {
    public String role;
    public String content;
    public String reasoning;
    public List<DeltaToolCall> tool_calls;
    public DeltaMessage() {}
    public static DeltaMessage content(String c) { DeltaMessage m = new DeltaMessage(); m.content = c; return m; }
    public static DeltaMessage withToolCall(DeltaToolCall tc) {
      DeltaMessage m = new DeltaMessage();
      m.tool_calls = new ArrayList<>();
      m.tool_calls.add(tc);
      return m;
    }
    public String toString() {
      StringBuilder sb = new StringBuilder("DeltaMessage(");
      boolean first = true;
      if (role != null)      { sb.append("role='").append(role).append("'"); first = false; }
      if (content != null)   { if (!first) sb.append(", "); sb.append("content='").append(content).append("'"); first = false; }
      if (reasoning != null) { if (!first) sb.append(", "); sb.append("reasoning='").append(reasoning).append("'"); first = false; }
      if (tool_calls != null && !tool_calls.isEmpty()) { if (!first) sb.append(", "); sb.append("tool_calls=").append(tool_calls); }
      return sb.append(")").toString();
    }
  }

  public static final class FunctionCall {
    public String name;
    public String arguments;
    public FunctionCall() {}
    public FunctionCall(String name, String arguments) { this.name = name; this.arguments = arguments; }
  }

  public static final class ToolCall {
    public String id;
    public String type = "function";
    public FunctionCall function;
    public ToolCall() {}
    public ToolCall(String type, FunctionCall function) { this.type = type; this.function = function; }
  }

  public static final class ExtractedToolCallInformation {
    public boolean tools_called;
    public List<ToolCall> tool_calls = new ArrayList<>();
    public String content;
  }

  public static final class Tool {
    public String type = "function";
    public FunctionDefinition function;
  }

  public static final class FunctionDefinition {
    public String name;
    public String description;
    public Map<String, Object> parameters;
  }

  static Map<String, Object> findToolProperties(List<Tool> tools, String functionName) {
    if (tools == null || functionName == null) return Collections.emptyMap();
    for (Tool t : tools) {
      if (t.function != null && functionName.equals(t.function.name)) {
        if (t.function.parameters != null) {
          Object props = t.function.parameters.get("properties");
          if (props instanceof Map) {
            @SuppressWarnings("unchecked")
            Map<String, Object> m = (Map<String, Object>) props;
            return m;
          }
        }
      }
    }
    return Collections.emptyMap();
  }

  static String jsonEncode(Object value) {
    if (value == null) return "null";
    if (value instanceof Boolean) return value.toString();
    if (value instanceof Number) {
      // Gson `fromJson(s, Object.class)` parses every JSON number as Double, losing the
      // Python `json.loads` int/float distinction. Re-emit whole-valued Doubles as ints
      // so `{"a": 1}` round-trips as `{"a": 1}` rather than `{"a": 1.0}`.
      if (value instanceof Long || value instanceof Integer || value instanceof Short || value instanceof Byte) {
        return value.toString();
      }
      double d = ((Number) value).doubleValue();
      if (d == Math.floor(d) && !Double.isInfinite(d) && Math.abs(d) < 1e16) {
        return Long.toString((long) d);
      }
      return value.toString();
    }
    if (value instanceof String) return "\"" + escapeJsonString((String) value) + "\"";
    if (value instanceof List) {
      StringBuilder sb = new StringBuilder("[");
      boolean first = true;
      for (Object o : (List<?>) value) {
        if (!first) sb.append(", ");
        sb.append(jsonEncode(o));
        first = false;
      }
      return sb.append("]").toString();
    }
    if (value instanceof Map) {
      StringBuilder sb = new StringBuilder("{");
      boolean first = true;
      for (Map.Entry<?, ?> e : ((Map<?, ?>) value).entrySet()) {
        if (!first) sb.append(", ");
        sb.append("\"").append(escapeJsonString(String.valueOf(e.getKey()))).append("\": ").append(jsonEncode(e.getValue()));
        first = false;
      }
      return sb.append("}").toString();
    }
    return "\"" + escapeJsonString(value.toString()) + "\"";
  }

  static String escapeJsonString(String s) {
    StringBuilder sb = new StringBuilder(s.length() + 8);
    for (int i = 0; i < s.length(); i++) {
      char c = s.charAt(i);
      switch (c) {
        case '\\': sb.append("\\\\"); break;
        case '"':  sb.append("\\\""); break;
        case '\b': sb.append("\\b"); break;
        case '\f': sb.append("\\f"); break;
        case '\n': sb.append("\\n"); break;
        case '\r': sb.append("\\r"); break;
        case '\t': sb.append("\\t"); break;
        default:
          if (c < 0x20) sb.append(String.format("\\u%04x", (int) c));
          else sb.append(c);
      }
    }
    return sb.toString();
  }

  private static final Gson GSON = new Gson();

  // Mirrors Python `json.loads`. Returns Map/List/Double/Boolean/String/null per Gson defaults.
  static Object jsonLoads(String s) {
    return GSON.fromJson(s, Object.class);
  }

  // Mirrors Python `ast.literal_eval` for common cases: single-quoted strings/keys,
  // True/False/None capitalisation. Best-effort; falls back to caller on parse failure.
  static Object astLiteralEval(String s) {
    String normalized = s
            .replace("True", "true")
            .replace("False", "false")
            .replace("None", "null");
    StringBuilder sb = new StringBuilder(normalized.length());
    boolean inDouble = false;
    for (int i = 0; i < normalized.length(); i++) {
      char c = normalized.charAt(i);
      if (c == '\\' && i + 1 < normalized.length()) { sb.append(c).append(normalized.charAt(++i)); continue; }
      if (c == '"') inDouble = !inDouble;
      if (c == '\'' && !inDouble) { sb.append('"'); continue; }
      sb.append(c);
    }
    return GSON.fromJson(sb.toString(), Object.class);
  }

  private boolean currentToolNameSent = false;
  private final List<Map<String, String>> prevToolCallArr = new ArrayList<>();
  private final List<String> streamedArgsForTool = new ArrayList<>();

  private final String toolCallStartToken = "<tool_call>";
  private final String toolCallEndToken = "</tool_call>";
  private final String toolCallPrefix = "<function=";
  private final String functionEndToken = "</function>";
  private final String parameterPrefix = "<parameter=";
  private final String parameterEndToken = "</parameter>";

  private boolean isToolCallStarted = false;
  private int failedCount = 0;

  public int currentToolIndex = 0;
  public boolean headerSent = false;
  public String currentToolId = null;
  public String currentFunctionName = null;
  public String currentParamName = null;
  public String currentParamValue = "";
  public String currentParamValueDelta = "";
  public String currentParamValueBuffer = "";
  public boolean completedParam = false;
  public String lastParamName = null;
  public String lastParamValue = "";
  public int paramCount = 0;
  public boolean inParam = false;
  public boolean inFunction = false;
  public String accumulatedText = "";
  public boolean jsonStarted = false;
  public boolean jsonClosed = false;
  public final Map<String, Object> accumulatedParams = new LinkedHashMap<>();
  public String nlgText = "";
  public String nlgTextDelta = "";
  boolean nlgTextDone = false;

  private List<Tool> tools;

  private static final Pattern TOOL_CALL_COMPLETE_REGEX =
          Pattern.compile("<tool_call>(.*?)</tool_call>", Pattern.DOTALL);
  private static final Pattern TOOL_CALL_REGEX =
          Pattern.compile("<tool_call>(.*?)</tool_call>|<tool_call>(.*?)$", Pattern.DOTALL);
  private static final Pattern TOOL_CALL_FUNCTION_REGEX =
          Pattern.compile("<function=(.*?)</function>|<function=(.*)$", Pattern.DOTALL);
  private static final Pattern TOOL_CALL_PARAMETER_REGEX =
          Pattern.compile("<parameter=(.*?)(?:</parameter>|(?=<parameter=)|(?=</function>)|$)", Pattern.DOTALL);

  public Qwen3CoderToolParser() { this(null); }
  public Qwen3CoderToolParser(List<Tool> tools) {
    this.tools = tools;
    resetStreamingState();
  }

  public void setTools(List<Tool> tools) { this.tools = tools; }

  private String generateToolCallId() {
    return "call_" + UUID.randomUUID().toString().replace("-", "").substring(0, 24);
  }

  private void resetStreamingState() {
    currentToolIndex = 0;
    isToolCallStarted = false;
    headerSent = false;
    currentToolId = null;
    currentFunctionName = null;
    currentParamName = null;
    currentParamValue = "";
    paramCount = 0;
    inParam = false;
    inFunction = false;
    accumulatedText = "";
    jsonStarted = false;
    jsonClosed = false;
    accumulatedParams.clear();
    prevToolCallArr.clear();
    streamedArgsForTool.clear();
  }

  // Mirrors Python `_convert_param_value(param_value, param_name, param_config, func_name)`.
  Object convertParamValue(String paramValue, String paramName, Map<String, Object> paramConfig, String funcName) {
    if ("null".equals(paramValue.toLowerCase())) return null;

    if (!paramConfig.containsKey(paramName)) {
      return paramValue;
    }

    String paramType;
    Object cfg = paramConfig.get(paramName);
    if (cfg instanceof Map) {
      @SuppressWarnings("unchecked")
      Map<String, Object> cfgMap = (Map<String, Object>) cfg;
      if (cfgMap.containsKey("type")) {
        paramType = String.valueOf(cfgMap.get("type")).trim().toLowerCase();
      } else if (cfgMap.containsKey("anyOf")) {
        paramType = "object";
      } else {
        paramType = "string";
      }
    } else {
      paramType = "string";
    }

    if (paramType.equals("string") || paramType.equals("str") || paramType.equals("text")
            || paramType.equals("varchar") || paramType.equals("char") || paramType.equals("enum")) {
      return paramValue;
    } else if (paramType.startsWith("int") || paramType.startsWith("uint")
            || paramType.startsWith("long") || paramType.startsWith("short")
            || paramType.startsWith("unsigned")) {
      try {
        return Long.parseLong(paramValue.trim());
      } catch (NumberFormatException e) {
        return paramValue;
      }
    } else if (paramType.startsWith("num") || paramType.startsWith("float")) {
      try {
        double f = Double.parseDouble(paramValue.trim());
        if (f - (long) f != 0.0) return f;
        return (long) f;
      } catch (NumberFormatException e) {
        return paramValue;
      }
    } else if (paramType.equals("boolean") || paramType.equals("bool") || paramType.equals("binary")) {
      String pv = paramValue.toLowerCase();
      return pv.equals("true");
    } else {
      Object converted = paramValue;
      if (paramType.equals("object") || paramType.equals("array") || paramType.equals("arr")
              || paramType.startsWith("dict") || paramType.startsWith("list")) {
        try {
          converted = jsonLoads(paramValue);
          return converted;
        } catch (Exception ignored) {}
      }
      try {
        converted = astLiteralEval(paramValue);
      } catch (Exception ignored) {}
      return converted;
    }
  }

  // Mirrors Python `_parse_xml_function_call`.
  ToolCall parseXmlFunctionCall(String functionCallStr) {
    int endIndex = functionCallStr.indexOf('>');
    if (endIndex == -1) return null;
    String functionName = functionCallStr.substring(0, endIndex);
    Map<String, Object> paramConfig = findToolProperties(tools, functionName);
    String parameters = functionCallStr.substring(endIndex + 1);
    Map<String, Object> paramDict = new LinkedHashMap<>();
    Matcher pm = TOOL_CALL_PARAMETER_REGEX.matcher(parameters);
    while (pm.find()) {
      String matchText = pm.group(1);
      if (matchText == null) continue;
      int idx = matchText.indexOf('>');
      if (idx == -1) continue;
      String paramName = matchText.substring(0, idx);
      String paramValue = matchText.substring(idx + 1);
      if (paramValue.startsWith("\n")) paramValue = paramValue.substring(1);
      if (paramValue.endsWith("\n")) paramValue = paramValue.substring(0, paramValue.length() - 1);
      paramDict.put(paramName, paramValue);
      //paramDict.put(paramName, convertParamValue(paramValue, paramName, paramConfig, functionName));
    }
    ToolCall tc = new ToolCall();
    tc.id = generateToolCallId();
    tc.type = "function";
    tc.function = new FunctionCall(functionName, jsonEncode(paramDict));
    return tc;
  }

  // Mirrors Python `_get_function_calls`.
  List<String> getFunctionCalls(String modelOutput) {
    List<String> rawToolCalls = new ArrayList<>();
    Matcher m = TOOL_CALL_REGEX.matcher(modelOutput);
    while (m.find()) {
      String g1 = m.group(1);
      String g2 = m.group(2);
      rawToolCalls.add(g1 != null ? g1 : g2);
    }
    if (rawToolCalls.isEmpty()) rawToolCalls.add(modelOutput);
    List<String> functionCalls = new ArrayList<>();
    for (String tc : rawToolCalls) {
      Matcher fm = TOOL_CALL_FUNCTION_REGEX.matcher(tc);
      while (fm.find()) {
        String g1 = fm.group(1);
        String g2 = fm.group(2);
        functionCalls.add(g1 != null ? g1 : g2);
      }
    }
    return functionCalls;
  }

  // Mirrors Python `extract_tool_calls`.
  public ExtractedToolCallInformation extractToolCalls(String modelOutput) {
    ExtractedToolCallInformation out = new ExtractedToolCallInformation();
    if (!modelOutput.contains(toolCallPrefix)) {
      out.tools_called = false;
      out.content = modelOutput;
      return out;
    }
    try {
      List<String> functionCalls = getFunctionCalls(modelOutput);
      if (functionCalls.isEmpty()) {
        out.tools_called = false;
        out.content = modelOutput;
        return out;
      }
      List<ToolCall> tool_calls = new ArrayList<>();
      for (String fc : functionCalls) {
        ToolCall tc = parseXmlFunctionCall(fc);
        if (tc != null) tool_calls.add(tc);
      }
      prevToolCallArr.clear();
      for (ToolCall tc : tool_calls) {
        Map<String, String> m = new HashMap<>();
        m.put("name", tc.function.name);
        m.put("arguments", tc.function.arguments);
        prevToolCallArr.add(m);
      }
      int contentIndex = modelOutput.indexOf(toolCallStartToken);
      int idx = modelOutput.indexOf(toolCallPrefix);
      if (contentIndex < 0) contentIndex = idx;
      String content = (contentIndex >= 0) ? modelOutput.substring(0, contentIndex) : "";
      out.tools_called = !tool_calls.isEmpty();
      out.tool_calls = tool_calls;
      out.content = content.isEmpty() ? null : content;
      return out;
    } catch (Exception e) {
      out.tools_called = false;
      out.content = modelOutput;
      return out;
    }
  }

  // Mirrors Python `extract_tool_calls_streaming(previous_text, current_text, delta_text, ...)`.
  // The Python signature also takes token-id sequences; see the file header for why this port
  // omits them and what the practical implication is.
  public DeltaMessage extractToolCallsStreaming(String previousText, String currentText, String deltaText) {
    currentParamValueDelta = "";
    completedParam = false;
    if (previousText == null || previousText.isEmpty()) {
      resetStreamingState();
    }

    if (deltaText == null || deltaText.isEmpty()) {
      int completeCalls = countMatches(TOOL_CALL_COMPLETE_REGEX, currentText);
      if (completeCalls > 0 && !prevToolCallArr.isEmpty()) {
        int openCalls = countOccurrences(currentText, toolCallStartToken)
                - countOccurrences(currentText, toolCallEndToken);
        if (openCalls == 0) return DeltaMessage.content("");
      } else if (!isToolCallStarted && currentText != null && !currentText.isEmpty()) {
        return DeltaMessage.content("");
      }
      return null;
    }

    accumulatedText = currentText;

    if (jsonClosed && !inFunction) {
      int toolEnds = countOccurrences(currentText, toolCallEndToken);
      if (toolEnds > currentToolIndex) {
        currentToolIndex++;
        headerSent = false;
        paramCount = 0;
        jsonStarted = false;
        jsonClosed = false;
        accumulatedParams.clear();
        int toolStarts = countOccurrences(currentText, toolCallStartToken);
        if (currentToolIndex >= toolStarts) isToolCallStarted = false;
        return null;
      }
    }

    if (!isToolCallStarted) {
      if (deltaText.contains(toolCallStartToken)) {
        isToolCallStarted = true;
        String contentBefore = deltaText.substring(0, deltaText.indexOf(toolCallStartToken));
        if (!contentBefore.isEmpty()) return DeltaMessage.content(contentBefore);
        return null;
      } else {
        if (rstrip(currentText).endsWith(toolCallEndToken) && deltaText.trim().isEmpty()) return null;
        return DeltaMessage.content(deltaText);
      }
    }

    int toolStartsCount = countOccurrences(currentText, toolCallStartToken);
    if (currentToolIndex >= toolStartsCount) return null;

    List<Integer> toolStartPositions = new ArrayList<>();
    int idx = 0;
    while (true) {
      int pos = currentText.indexOf(toolCallStartToken, idx);
      if (pos < 0) break;
      toolStartPositions.add(pos);
      idx = pos + toolCallStartToken.length();
    }
    if (currentToolIndex >= toolStartPositions.size()) return null;

    int toolStartIdx = toolStartPositions.get(currentToolIndex);
    int toolEndIdx = currentText.indexOf(toolCallEndToken, toolStartIdx);
    String toolText = (toolEndIdx == -1)
            ? currentText.substring(toolStartIdx)
            : currentText.substring(toolStartIdx, toolEndIdx + toolCallEndToken.length());

    if (!headerSent) {
      if (toolText.contains(toolCallPrefix)) {
        int funcStart = toolText.indexOf(toolCallPrefix) + toolCallPrefix.length();
        int funcEnd = toolText.indexOf('>', funcStart);
        if (funcEnd != -1) {
          currentFunctionName = toolText.substring(funcStart, funcEnd);
          currentToolId = generateToolCallId();
          headerSent = true;
          inFunction = true;

          Map<String, String> entry = new HashMap<>();
          entry.put("name", currentFunctionName);
          entry.put("arguments", "{}");
          prevToolCallArr.add(entry);
          streamedArgsForTool.add("");

          DeltaToolCall dtc = new DeltaToolCall();
          dtc.index = currentToolIndex;
          dtc.id = currentToolId;
          dtc.type = "function";
          dtc.function = new DeltaFunctionCall(currentFunctionName, "");
          return DeltaMessage.withToolCall(dtc);
        }
      }
      return null;
    }

    if (inFunction) {
      if (!jsonStarted) {
        jsonStarted = true;
        streamedArgsForTool.set(currentToolIndex, streamedArgsForTool.get(currentToolIndex) + "{");
        DeltaToolCall dtc = new DeltaToolCall();
        dtc.index = currentToolIndex;
        dtc.function = new DeltaFunctionCall(null, "{");
        return DeltaMessage.withToolCall(dtc);
      }

      List<Integer> paramStarts = new ArrayList<>();
      int searchIdx = 0;
      while (true) {
        int pos = toolText.indexOf(parameterPrefix, searchIdx);
        if (pos < 0) break;
        paramStarts.add(pos);
        inParam = true;
        searchIdx = pos + parameterPrefix.length();
      }

      List<String> jsonFragments = new ArrayList<>();
      while (inParam && paramCount < paramStarts.size()) {
        int paramIdx = paramStarts.get(paramCount);
        int paramStart = paramIdx + parameterPrefix.length();
        String remaining = toolText.substring(paramStart);
        int nameEnd = remaining.indexOf('>');
        if (nameEnd == -1) break;
        String pName = remaining.substring(0, nameEnd);
        currentParamName = pName;
        int valueStart = paramStart + nameEnd + 1;
        String valueText = toolText.substring(valueStart);
        if (valueText.startsWith("\n")) valueText = valueText.substring(1);

        int paramEndIdx = valueText.indexOf(parameterEndToken);
        if (paramEndIdx == -1) {
          int nextParamIdx = valueText.indexOf(parameterPrefix);
          int funcEndIdx = valueText.indexOf(functionEndToken);
          if (nextParamIdx != -1 && (funcEndIdx == -1 || nextParamIdx < funcEndIdx)) {
            paramEndIdx = nextParamIdx;
          } else if (funcEndIdx != -1) {
            paramEndIdx = funcEndIdx;
          } else {
            int toolEndInValue = valueText.indexOf(toolCallEndToken);
            if (toolEndInValue != -1) {
              paramEndIdx = toolEndInValue;
            } else {
              if (valueText.isEmpty()) {
                break;
              }
              // Don't include partial end token in results until sure it is not part of an end token
              if (deltaText.contains("<") || !currentParamValueBuffer.isEmpty()) {
                // Store partial results in buffer until sure it is safe or not
                currentParamValueBuffer += deltaText;

                // Whitespace or close character without seeing paramEndToken (checked above) means we are safe. Append entire buffer
                if (deltaText.contains(">") || deltaText.matches(".*\\s.*")) {
                  currentParamValue += currentParamValueBuffer;
                  currentParamValueBuffer = "";
                }
              } else {
                currentParamValueDelta = deltaText;
                currentParamValue = valueText;
              }
              break;
            }
          }
        }

        completedParam = true;
        lastParamName = currentParamName;
        lastParamValue = currentParamValue;
        inParam = false;
        currentParamName = "";
        currentParamValue = "";

        String paramValue = valueText.substring(0, paramEndIdx);
        if (paramValue.endsWith("\n")) paramValue = paramValue.substring(0, paramValue.length() - 1);

        accumulatedParams.put(pName, paramValue);

        //Map<String, Object> paramConfig = findToolProperties(tools, currentFunctionName == null ? "" : currentFunctionName);
        //Object converted = convertParamValue(paramValue, pName, paramConfig, currentFunctionName == null ? "" : currentFunctionName);
        Object converted = paramValue;
        String serialized = jsonEncode(converted);

        String fragment;
        if (paramCount == 0) fragment = "\"" + pName + "\": " + serialized;
        else fragment = ", \"" + pName + "\": " + serialized;

        paramCount++;
        jsonFragments.add(fragment);
      }

      if (!jsonFragments.isEmpty()) {
        StringBuilder combined = new StringBuilder();
        for (String f : jsonFragments) combined.append(f);
        String combinedStr = combined.toString();
        if (currentToolIndex < streamedArgsForTool.size()) {
          streamedArgsForTool.set(currentToolIndex, streamedArgsForTool.get(currentToolIndex) + combinedStr);
        }
        DeltaToolCall dtc = new DeltaToolCall();
        dtc.index = currentToolIndex;
        dtc.function = new DeltaFunctionCall(null, combinedStr);
        return DeltaMessage.withToolCall(dtc);
      }

      if (!jsonClosed && toolText.contains(functionEndToken)) {
        jsonClosed = true;
        int funcStart2 = toolText.indexOf(toolCallPrefix) + toolCallPrefix.length();
        int funcContentEnd = toolText.indexOf(functionEndToken, funcStart2);
        if (funcContentEnd != -1) {
          String funcContent = toolText.substring(funcStart2, funcContentEnd);
          try {
            ToolCall parsed = parseXmlFunctionCall(funcContent);
            if (parsed != null && currentToolIndex < prevToolCallArr.size()) {
              prevToolCallArr.get(currentToolIndex).put("arguments", parsed.function.arguments);
            }
          } catch (Exception ignored) {}
        }
        if (currentToolIndex < streamedArgsForTool.size()) {
          streamedArgsForTool.set(currentToolIndex, streamedArgsForTool.get(currentToolIndex) + "}");
        }
        DeltaToolCall dtc = new DeltaToolCall();
        dtc.index = currentToolIndex;
        dtc.function = new DeltaFunctionCall(null, "}");
        DeltaMessage result = DeltaMessage.withToolCall(dtc);
        inFunction = false;
        accumulatedParams.clear();
        return result;
      }
    }
    return null;
  }

  // Stateful wrapper over the (previous_text, current_text, delta_text) Python signature.
  // Tracks accumulated text internally so callers only pass each fragment as it arrives.

  private String streamingPrev = "";

  public DeltaMessage feed(String deltaText) {
    String d = deltaText == null ? "" : deltaText;
    String cur = streamingPrev + d;
    DeltaMessage result = extractToolCallsStreaming(streamingPrev, cur, d);
    streamingPrev = cur;
    return result;
  }

  public DeltaMessage finishStream() {
    return extractToolCallsStreaming(streamingPrev, streamingPrev, "");
  }

  public void resetStream() {
    streamingPrev = "";
    resetStreamingState();
  }

  List<String> debugStreamedArgsForTool() { return streamedArgsForTool; }
  List<Map<String, String>> debugPrevToolCallArr() { return prevToolCallArr; }

  static int countOccurrences(String text, String substr) {
    if (text == null || substr.isEmpty()) return 0;
    int count = 0, idx = 0;
    while ((idx = text.indexOf(substr, idx)) != -1) { count++; idx += substr.length(); }
    return count;
  }

  static int countMatches(Pattern p, String text) {
    if (text == null) return 0;
    Matcher m = p.matcher(text);
    int n = 0;
    while (m.find()) n++;
    return n;
  }

  static String rstrip(String s) {
    int end = s.length();
    while (end > 0 && Character.isWhitespace(s.charAt(end - 1))) end--;
    return s.substring(0, end);
  }
}
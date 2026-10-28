/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.asl;

import edu.tufts.hrilab.action.EventSpec;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.regex.Pattern;

/**
 * Builder/editor for ASL EventSpec sequences with structured control-flow operations.
 */
public final class AslBlockBuilder {
  public record BlockHandle(int startIndex, long version) {
  }

  public record OpenBlock(
      AslBlockStructure.BlockType type,
      int startIndex,
      Integer parentStartIndex,
      String activeBranch,
      boolean hasElse,
      Collection<String> declaredLocals,
      int insertionIndex
  ) {
  }

  private static final class OpenState {
    private final AslBlockStructure.BlockType type;
    private final int startIndex;
    private final Integer parentStartIndex;
    private String activeBranch;
    private boolean hasElse;
    private final LinkedHashSet<String> declaredLocals = new LinkedHashSet<>();

    private OpenState(AslBlockStructure.BlockType type, int startIndex, Integer parentStartIndex) {
      this.type = type;
      this.startIndex = startIndex;
      this.parentStartIndex = parentStartIndex;
    }
  }

  private final List<EventSpec> specs;
  private final ArrayDeque<OpenState> open = new ArrayDeque<>();
  private long mutationVersion = 0L;

  public AslBlockBuilder() {
    this(List.of());
  }

  public AslBlockBuilder(List<EventSpec> existing) {
    this.specs = new ArrayList<>(Objects.requireNonNull(existing, "existing"));
    AslBlockStructure.validateControlFlow(this.specs);
  }

  public List<EventSpec> build() {
    if (!open.isEmpty()) {
      OpenState top = open.peek();
      throw new IllegalStateException("Unclosed block: " + top.type + " at index " + top.startIndex);
    }
    AslBlockStructure.validateControlFlow(specs);
    AslScopeValidator.validate(specs);
    return List.copyOf(specs);
  }

  public AslBlockStructure.Snapshot snapshot() {
    return AslBlockStructure.validateControlFlow(specs);
  }

  public long version() {
    return mutationVersion;
  }

  public BlockHandle blockHandle(int blockStartIndex) {
    AslBlockStructure.validateControlFlow(specs).byStartIndex(blockStartIndex)
        .orElseThrow(() -> new IllegalArgumentException("No block starts at index " + blockStartIndex));
    return new BlockHandle(blockStartIndex, mutationVersion);
  }

  public OpenBlock getCurrentBlock() {
    if (open.isEmpty()) {
      return null;
    }
    return toOpenBlock(open.peek());
  }

  public List<OpenBlock> getOpenBlocks() {
    List<OpenState> states = new ArrayList<>(open);
    Collections.reverse(states);
    List<OpenBlock> result = new ArrayList<>();
    for (OpenState state : states) {
      result.add(toOpenBlock(state));
    }
    return result;
  }

  private OpenBlock toOpenBlock(OpenState state) {
    return new OpenBlock(
        state.type,
        state.startIndex,
        state.parentStartIndex,
        state.activeBranch,
        state.hasElse,
        List.copyOf(state.declaredLocals),
        specs.size()
    );
  }

  public AslBlockBuilder add(EventSpec spec) {
    specs.add(Objects.requireNonNull(spec, "spec"));
    touch();
    return this;
  }

  public AslBlockBuilder addAll(List<EventSpec> toAdd) {
    Objects.requireNonNull(toAdd, "toAdd");
    specs.addAll(toAdd);
    touch();
    return this;
  }

  public AslBlockBuilder beginBlock() {
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.BLOCK));
    push(AslBlockStructure.BlockType.BLOCK);
    touch();
    return this;
  }

  public AslBlockBuilder endBlock() {
    ensureTop(AslBlockStructure.BlockType.BLOCK, "endblock");
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDBLOCK));
    open.pop();
    touch();
    return this;
  }

  public AslBlockBuilder beginIf(List<EventSpec> conditionSpecs) {
    int startIndex = specs.size();
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.IF));
    if (conditionSpecs != null) {
      specs.addAll(conditionSpecs);
    }
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.THEN));
    OpenState state = push(AslBlockStructure.BlockType.IF, startIndex);
    state.activeBranch = "then";
    touch();
    return this;
  }

  public AslBlockBuilder beginElseIf(List<EventSpec> conditionSpecs) {
    OpenState state = ensureTop(AslBlockStructure.BlockType.IF, "elseif");
    if (state.hasElse) {
      throw new IllegalStateException("Cannot add elseif after else");
    }
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ELSEIF));
    if (conditionSpecs != null) {
      specs.addAll(conditionSpecs);
    }
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.THEN));
    state.activeBranch = "elseif";
    touch();
    return this;
  }

  public AslBlockBuilder beginElse() {
    OpenState state = ensureTop(AslBlockStructure.BlockType.IF, "else");
    if (state.hasElse) {
      throw new IllegalStateException("Cannot add duplicate else branch");
    }
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ELSE));
    state.hasElse = true;
    state.activeBranch = "else";
    touch();
    return this;
  }

  public AslBlockBuilder endIf() {
    ensureTop(AslBlockStructure.BlockType.IF, "endif");
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDIF));
    open.pop();
    touch();
    return this;
  }

  public AslBlockBuilder beginWhile(List<EventSpec> conditionSpecs) {
    int startIndex = specs.size();
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.WHILE));
    if (conditionSpecs != null) {
      specs.addAll(conditionSpecs);
    }
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.DO));
    push(AslBlockStructure.BlockType.WHILE, startIndex);
    touch();
    return this;
  }

  public AslBlockBuilder endWhile() {
    ensureTop(AslBlockStructure.BlockType.WHILE, "endwhile");
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDWHILE));
    open.pop();
    touch();
    return this;
  }

  public AslBlockBuilder beginFor(String loopVar, String initialValue, String compareOperator,
                                  String compareValue, String update) {
    int startIndex = specs.size();
    List<String> args = new ArrayList<>();
    args.add(Objects.requireNonNull(loopVar, "loopVar"));
    if (initialValue != null) {
      args.add(initialValue);
    }
    args.add(Objects.requireNonNull(compareOperator, "compareOperator"));
    args.add(Objects.requireNonNull(compareValue, "compareValue"));
    args.add(Objects.requireNonNull(update, "update"));
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.FOR, args, List.of()));
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.BLOCK));
    push(AslBlockStructure.BlockType.FOR, startIndex);
    touch();
    return this;
  }

  public AslBlockBuilder endFor() {
    ensureTop(AslBlockStructure.BlockType.FOR, "endfor");
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDBLOCK));
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDFOR));
    open.pop();
    touch();
    return this;
  }

  public AslBlockBuilder beginForeach(String elementVar, String collectionVar) {
    int startIndex = specs.size();
    List<String> args = List.of(
        Objects.requireNonNull(elementVar, "elementVar"),
        Objects.requireNonNull(collectionVar, "collectionVar")
    );
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.FOREACH, args, List.of()));
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.BLOCK));
    push(AslBlockStructure.BlockType.FOREACH, startIndex);
    touch();
    return this;
  }

  public AslBlockBuilder endForeach() {
    ensureTop(AslBlockStructure.BlockType.FOREACH, "endfor");
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDBLOCK));
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDFOR));
    open.pop();
    touch();
    return this;
  }

  public AslBlockBuilder beginTry() {
    int startIndex = specs.size();
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.TRY));
    push(AslBlockStructure.BlockType.TRY, startIndex);
    touch();
    return this;
  }

  public AslBlockBuilder beginCatch(List<String> catchArgs) {
    ensureTop(AslBlockStructure.BlockType.TRY, "catch");
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.CATCH,
        catchArgs == null ? List.of() : catchArgs, List.of()));
    touch();
    return this;
  }

  public AslBlockBuilder beginFinally() {
    ensureTop(AslBlockStructure.BlockType.TRY, "finally");
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.FINALLY));
    touch();
    return this;
  }

  public AslBlockBuilder endTry() {
    ensureTop(AslBlockStructure.BlockType.TRY, "endtry");
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDTRY));
    open.pop();
    touch();
    return this;
  }

  public AslBlockBuilder beginAsync(List<String> inputArgs, List<String> returnArgs) {
    int startIndex = specs.size();
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ASYNC,
        inputArgs == null ? List.of() : inputArgs,
        returnArgs == null ? List.of() : returnArgs));
    push(AslBlockStructure.BlockType.ASYNC, startIndex);
    touch();
    return this;
  }

  public AslBlockBuilder endAsync() {
    ensureTop(AslBlockStructure.BlockType.ASYNC, "endasync");
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDASYNC));
    open.pop();
    touch();
    return this;
  }

  public AslBlockBuilder beginNoInterrupt() {
    int startIndex = specs.size();
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.NOINTERRUPT));
    push(AslBlockStructure.BlockType.NOINTERRUPT, startIndex);
    touch();
    return this;
  }

  public AslBlockBuilder endNoInterrupt() {
    ensureTop(AslBlockStructure.BlockType.NOINTERRUPT, "endnointerrupt");
    specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDNOINTERRUPT));
    open.pop();
    touch();
    return this;
  }

  public AslBlockBuilder recordLocalDeclaration(String localName) {
    if (open.isEmpty()) {
      return this;
    }
    open.peek().declaredLocals.add(Objects.requireNonNull(localName, "localName"));
    touch();
    return this;
  }

  public List<EventSpec> specs() {
    return Collections.unmodifiableList(specs);
  }

  public AslBlockBuilder validate() {
    AslBlockStructure.validateControlFlow(specs);
    AslScopeValidator.validate(specs);
    return this;
  }

  public AslBlockBuilder insert(int index, EventSpec spec) {
    mutateTransactionally(copy -> copy.add(index, Objects.requireNonNull(spec, "spec")));
    return this;
  }

  public AslBlockBuilder replaceRange(int fromInclusive, int toExclusive, List<EventSpec> replacement) {
    mutateTransactionally(copy -> {
      checkRange(copy, fromInclusive, toExclusive);
      copy.subList(fromInclusive, toExclusive).clear();
      if (replacement != null && !replacement.isEmpty()) {
        copy.addAll(fromInclusive, replacement);
      }
    });
    return this;
  }

  public AslBlockBuilder removeRange(int fromInclusive, int toExclusive) {
    return replaceRange(fromInclusive, toExclusive, List.of());
  }

  public AslBlockBuilder removeCompleteBlock(int blockStartIndex) {
    AslBlockStructure.Snapshot snapshot = AslBlockStructure.validateControlFlow(specs);
    AslBlockStructure.BlockDescriptor block = snapshot.byStartIndex(blockStartIndex)
        .orElseThrow(() -> new IllegalArgumentException("No block starts at index " + blockStartIndex));
    return removeRange(block.startIndex(), block.endIndex() + 1);
  }

  public AslBlockBuilder replaceCompleteBlock(int blockStartIndex, List<EventSpec> replacement) {
    AslBlockStructure.Snapshot snapshot = AslBlockStructure.validateControlFlow(specs);
    AslBlockStructure.BlockDescriptor block = snapshot.byStartIndex(blockStartIndex)
        .orElseThrow(() -> new IllegalArgumentException("No block starts at index " + blockStartIndex));
    return replaceRange(block.startIndex(), block.endIndex() + 1, replacement);
  }

  public AslBlockBuilder addBlockLocalDeclaration(int insertIndex, String varName, String javaType, String semanticType, String initializerExpression) {
    Objects.requireNonNull(varName, "varName");
    Objects.requireNonNull(javaType, "javaType");
    if (!open.isEmpty()) {
      checkRange(specs, insertIndex, insertIndex);
      List<EventSpec> insertion = new ArrayList<>();
      insertion.add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare")
          .addInputArg(varName).addInputArg(javaType).addInputArg(semanticType == null ? "" : semanticType).build());
      if (initializerExpression != null && !initializerExpression.isEmpty()) {
        insertion.add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign")
            .addInputArg(initializerExpression).addReturnArg(varName).build());
      }
      specs.addAll(insertIndex, insertion);
      touch();
      return this;
    }
    mutateTransactionally(copy -> {
      checkRange(copy, insertIndex, insertIndex);
      List<EventSpec> insertion = new ArrayList<>();
      insertion.add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare")
          .addInputArg(varName).addInputArg(javaType).addInputArg(semanticType == null ? "" : semanticType).build());
      if (initializerExpression != null && !initializerExpression.isEmpty()) {
        insertion.add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign")
            .addInputArg(initializerExpression).addReturnArg(varName).build());
      }
      copy.addAll(insertIndex, insertion);
    });
    return this;
  }

  public AslBlockBuilder removeBlockLocalDeclaration(int declarationIndex) {
    mutateTransactionally(copy -> {
      checkRange(copy, declarationIndex, declarationIndex + 1);
      EventSpec declaration = copy.get(declarationIndex);
      if (declaration.getType() != EventSpec.EventType.CONTROL || !"declare".equals(declaration.getCommand())
          || declaration.getInputArgs().isEmpty()) {
        throw new IllegalArgumentException("No declaration EventSpec at index " + declarationIndex);
      }
      String variable = declaration.getInputArgs().get(0);
      AslBlockStructure.Snapshot snapshot = AslBlockStructure.validateControlFlow(copy);
      int scopeEnd = Math.min(declarationScopeEnd(snapshot, declarationIndex), copy.size() - 1);
      List<Integer> refs = collectReferences(snapshot, copy, variable, declarationIndex + 1, scopeEnd);
      if (!refs.isEmpty()) {
        throw new IllegalArgumentException("Cannot remove declaration for " + variable
            + "; references remain at EventSpec indices " + refs);
      }
      copy.remove(declarationIndex);
    });
    return this;
  }

  public AslBlockBuilder renameBlockLocalVariable(int declarationIndex, String newName) {
    Objects.requireNonNull(newName, "newName");
    mutateTransactionally(copy -> {
      checkRange(copy, declarationIndex, declarationIndex + 1);
      EventSpec declaration = copy.get(declarationIndex);
      if (declaration.getType() != EventSpec.EventType.CONTROL || !"declare".equals(declaration.getCommand())
          || declaration.getInputArgs().isEmpty()) {
        throw new IllegalArgumentException("No declaration EventSpec at index " + declarationIndex);
      }
      String oldName = declaration.getInputArgs().get(0);
      if (oldName.equals(newName)) {
        return;
      }
      AslBlockStructure.Snapshot snapshot = AslBlockStructure.validateControlFlow(copy);
      List<String> declArgs = new ArrayList<>(declaration.getInputArgs());
      declArgs.set(0, newName);
      copy.set(declarationIndex, new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare")
          .setInputArgs(declArgs).build());
      int declarationScopeEnd = Math.min(declarationScopeEnd(snapshot, declarationIndex), copy.size() - 1);
      if (declarationScopeEnd < declarationIndex + 1) {
        return;
      }
      Map<Integer, Integer> shadowScopes = declarationScopeEnds(snapshot, copy, oldName, declarationIndex + 1, declarationScopeEnd);
      int skipShadowUntil = -1;
      for (int i = declarationIndex + 1; i <= declarationScopeEnd; i++) {
        if (i <= skipShadowUntil) {
          continue;
        }
        Integer shadowEnd = shadowScopes.get(i);
        if (shadowEnd != null) {
          skipShadowUntil = shadowEnd;
          continue;
        }
        EventSpec spec = copy.get(i);
        copy.set(i, renameInSpec(spec, oldName, newName));
      }
    });
    return this;
  }

  public AslBlockBuilder appendToBlockBody(int blockStartIndex, EventSpec spec) {
    Objects.requireNonNull(spec, "spec");
    mutateTransactionally(copy -> {
      AslBlockStructure.BlockDescriptor block = blockByStart(AslBlockStructure.validateControlFlow(copy), blockStartIndex);
      int insertionIndex = block.bodyRange().endInclusive() + 1;
      checkRange(copy, insertionIndex, insertionIndex);
      copy.add(insertionIndex, spec);
    });
    return this;
  }

  public AslBlockBuilder appendToBlockBody(BlockHandle handle, EventSpec spec) {
    Objects.requireNonNull(handle, "handle");
    if (handle.version() != mutationVersion) {
      throw new IllegalStateException("Stale block handle: expected version " + mutationVersion + " but got " + handle.version());
    }
    return appendToBlockBody(handle.startIndex(), spec);
  }

  public AslBlockBuilder insertIntoBlockBody(int blockStartIndex, int offset, EventSpec spec) {
    Objects.requireNonNull(spec, "spec");
    mutateTransactionally(copy -> {
      AslBlockStructure.BlockDescriptor block = blockByStart(AslBlockStructure.validateControlFlow(copy), blockStartIndex);
      int start = block.bodyRange().startInclusive();
      int endExclusive = block.bodyRange().endInclusive() + 1;
      int insertionIndex = start + offset;
      if (insertionIndex < start || insertionIndex > endExclusive) {
        throw new IllegalArgumentException("Offset " + offset + " is outside block body range " + start + ".." + endExclusive);
      }
      copy.add(insertionIndex, spec);
    });
    return this;
  }

  public AslBlockBuilder appendToBranch(int ifBlockStartIndex, int branchIndex, EventSpec spec) {
    Objects.requireNonNull(spec, "spec");
    mutateTransactionally(copy -> {
      AslBlockStructure.BlockDescriptor block = blockByStart(AslBlockStructure.validateControlFlow(copy), ifBlockStartIndex);
      if (block.type() != AslBlockStructure.BlockType.IF) {
        throw new IllegalArgumentException("Block at index " + ifBlockStartIndex + " is not an IF block");
      }
      if (branchIndex < 0 || branchIndex >= block.branches().size()) {
        throw new IllegalArgumentException("Invalid branch index " + branchIndex + " for IF block at " + ifBlockStartIndex);
      }
      AslBlockStructure.Range range = block.branches().get(branchIndex).bodyRange();
      int insertionIndex = range.endInclusive() + 1;
      copy.add(insertionIndex, spec);
    });
    return this;
  }

  public AslBlockBuilder replaceActionAt(int index, EventSpec replacement) {
    Objects.requireNonNull(replacement, "replacement");
    mutateTransactionally(copy -> {
      checkRange(copy, index, index + 1);
      if (copy.get(index).getType() == EventSpec.EventType.CONTROL) {
        throw new IllegalArgumentException("Index " + index + " is a control marker; use block-aware replacement");
      }
      copy.set(index, replacement);
    });
    return this;
  }

  public AslBlockBuilder removeActionAt(int index) {
    mutateTransactionally(copy -> {
      checkRange(copy, index, index + 1);
      if (copy.get(index).getType() == EventSpec.EventType.CONTROL) {
        throw new IllegalArgumentException("Index " + index + " is a control marker; use block-aware removal");
      }
      copy.remove(index);
    });
    return this;
  }

  private static void checkRange(List<EventSpec> list, int fromInclusive, int toExclusive) {
    if (fromInclusive < 0 || toExclusive < fromInclusive || toExclusive > list.size()) {
      throw new IllegalArgumentException("invalid range: " + fromInclusive + ".." + toExclusive);
    }
  }

  private interface Mutator {
    void apply(List<EventSpec> copy);
  }

  private void mutateTransactionally(Mutator mutator) {
    List<EventSpec> candidate = new ArrayList<>(specs);
    mutator.apply(candidate);
    AslBlockStructure.validateControlFlow(candidate);
    AslScopeValidator.validate(candidate);
    specs.clear();
    specs.addAll(candidate);
    mutationVersion++;
  }

  private static AslBlockStructure.BlockDescriptor blockByStart(AslBlockStructure.Snapshot snapshot, int blockStartIndex) {
    return snapshot.byStartIndex(blockStartIndex)
        .orElseThrow(() -> new IllegalArgumentException("No block starts at index " + blockStartIndex));
  }

  private static EventSpec renameInSpec(EventSpec spec, String oldName, String newName) {
    if (spec.getType() == EventSpec.EventType.CONTROL && "declare".equals(spec.getCommand())
        && !spec.getInputArgs().isEmpty() && spec.getInputArgs().get(0).equals(oldName)) {
      List<String> args = new ArrayList<>(spec.getInputArgs());
      args.set(0, newName);
      return new EventSpec.Builder(spec.getType()).setActor(spec.getActor()).setCommand(spec.getCommand())
          .setInputArgs(args).setReturnArgs(spec.getReturnArgs()).build();
    }

    List<String> inputArgs = new ArrayList<>(spec.getInputArgs());
    List<String> returnArgs = new ArrayList<>(spec.getReturnArgs());
    boolean changed = false;

    for (int i = 0; i < inputArgs.size(); i++) {
      String before = inputArgs.get(i);
      String after = replaceToken(before, oldName, newName);
      if (!before.equals(after)) {
        inputArgs.set(i, after);
        changed = true;
      }
    }
    for (int i = 0; i < returnArgs.size(); i++) {
      String before = returnArgs.get(i);
      String after = replaceToken(before, oldName, newName);
      if (!before.equals(after)) {
        returnArgs.set(i, after);
        changed = true;
      }
    }

    if (!changed) {
      return spec;
    }
    return new EventSpec.Builder(spec.getType()).setActor(spec.getActor()).setCommand(spec.getCommand())
        .setInputArgs(inputArgs).setReturnArgs(returnArgs).build();
  }

  private static String replaceToken(String value, String oldName, String newName) {
    if (value == null || value.isEmpty()) {
      return value;
    }
    if (value.equals(oldName)) {
      return newName;
    }
    String pattern = "(?<![A-Za-z0-9_])" + Pattern.quote(oldName) + "(?![A-Za-z0-9_])";
    return value.replaceAll(pattern, java.util.regex.Matcher.quoteReplacement(newName));
  }

  private static Map<Integer, Integer> declarationScopeEnds(AslBlockStructure.Snapshot snapshot, List<EventSpec> specs,
                                                             String name, int fromInclusive, int toInclusive) {
    Map<Integer, Integer> result = new HashMap<>();
    for (int i = fromInclusive; i <= toInclusive; i++) {
      EventSpec spec = specs.get(i);
      if (spec.getType() == EventSpec.EventType.CONTROL
          && "declare".equals(spec.getCommand())
          && !spec.getInputArgs().isEmpty()
          && name.equals(spec.getInputArgs().get(0))) {
        result.put(i, declarationScopeEnd(snapshot, i));
      }
    }
    return result;
  }

  private static int declarationScopeEnd(AslBlockStructure.Snapshot snapshot, int declarationIndex) {
    return snapshot.findEnclosingBlock(declarationIndex)
        .map(block -> {
          if (block.type() == AslBlockStructure.BlockType.IF || block.type() == AslBlockStructure.BlockType.TRY) {
            for (AslBlockStructure.Branch branch : block.branches()) {
              AslBlockStructure.Range range = branch.bodyRange();
              if (range.startInclusive() <= declarationIndex && declarationIndex <= range.endInclusive()) {
                return range.endInclusive();
              }
            }
          }
          return block.bodyRange().endInclusive();
        })
        .orElse(Integer.MAX_VALUE);
  }

  private static List<Integer> collectReferences(AslBlockStructure.Snapshot snapshot, List<EventSpec> specs, String variable,
                                                 int fromInclusive, int toInclusive) {
    List<Integer> refs = new ArrayList<>();
    if (toInclusive < fromInclusive) {
      return refs;
    }
    Map<Integer, Integer> shadowScopes = declarationScopeEnds(snapshot, specs, variable, fromInclusive, toInclusive);
    int skipShadowUntil = -1;
    for (int i = fromInclusive; i <= toInclusive; i++) {
      if (i <= skipShadowUntil) {
        continue;
      }
      Integer shadowEnd = shadowScopes.get(i);
      if (shadowEnd != null) {
        skipShadowUntil = shadowEnd;
        continue;
      }
      EventSpec spec = specs.get(i);
      if (references(spec, variable)) {
        refs.add(i);
      }
    }
    return refs;
  }

  private static boolean references(EventSpec spec, String variable) {
    for (String input : spec.getInputArgs()) {
      if (input != null && !replaceToken(input, variable, "__x__").equals(input)) {
        return true;
      }
    }
    for (String output : spec.getReturnArgs()) {
      if (variable.equals(output)) {
        return true;
      }
    }
    return false;
  }

  private OpenState push(AslBlockStructure.BlockType type) {
    return push(type, specs.size() - 1);
  }

  private OpenState push(AslBlockStructure.BlockType type, int startIndex) {
    OpenState state = new OpenState(type, startIndex, open.isEmpty() ? null : open.peek().startIndex);
    open.push(state);
    return state;
  }

  private OpenState ensureTop(AslBlockStructure.BlockType expected, String operation) {
    if (open.isEmpty() || open.peek().type != expected) {
      throw new IllegalStateException("Cannot " + operation + " without open " + expected + " block");
    }
    return open.peek();
  }

  private void touch() {
    mutationVersion++;
  }
}

/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.asl;

import edu.tufts.hrilab.action.EventSpec;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/**
 * Structural view and validation for ASL control flow represented as List&lt;EventSpec&gt;.
 */
public final class AslBlockStructure {
  private AslBlockStructure() {
  }

  public enum BlockType {
    IF, WHILE, FOR, FOREACH, TRY, ASYNC, NOINTERRUPT, BLOCK
  }

  public record Range(int startInclusive, int endInclusive) {
    public Range {
      if (startInclusive < 0 || endInclusive < startInclusive - 1) {
        throw new IllegalArgumentException("invalid range: " + startInclusive + ".." + endInclusive);
      }
    }
  }

  public record Branch(String kind, Range bodyRange) {
    public Branch {
      Objects.requireNonNull(kind, "kind");
      Objects.requireNonNull(bodyRange, "bodyRange");
    }
  }

  public record BlockDescriptor(
      BlockType type,
      int startIndex,
      int endIndex,
      Integer parentStartIndex,
      Range bodyRange,
      List<Branch> branches
  ) {
    public BlockDescriptor {
      Objects.requireNonNull(type, "type");
      if (startIndex < 0 || endIndex < startIndex) {
        throw new IllegalArgumentException("invalid block bounds: " + startIndex + ".." + endIndex);
      }
      Objects.requireNonNull(bodyRange, "bodyRange");
      branches = branches == null ? List.of() : List.copyOf(branches);
    }
  }

  public static final class Snapshot {
    private final List<EventSpec> specs;
    private final List<BlockDescriptor> blocks;
    private final Map<Integer, BlockDescriptor> byStart;

    private Snapshot(List<EventSpec> specs, List<BlockDescriptor> blocks) {
      this.specs = List.copyOf(specs);
      this.blocks = List.copyOf(blocks);
      Map<Integer, BlockDescriptor> tmp = new HashMap<>();
      for (BlockDescriptor block : blocks) {
        tmp.put(block.startIndex(), block);
      }
      this.byStart = Map.copyOf(tmp);
    }

    public List<EventSpec> specs() {
      return specs;
    }

    public List<BlockDescriptor> blocks() {
      return blocks;
    }

    public Optional<BlockDescriptor> byStartIndex(int startIndex) {
      return Optional.ofNullable(byStart.get(startIndex));
    }

    public OptionalInt findMatchingEnd(int startIndex) {
      return byStartIndex(startIndex).map(block -> OptionalInt.of(block.endIndex())).orElse(OptionalInt.empty());
    }

    public Optional<BlockDescriptor> findEnclosingBlock(int index) {
      BlockDescriptor best = null;
      for (BlockDescriptor block : blocks) {
        if (block.startIndex() < index && index < block.endIndex()) {
          if (best == null || block.startIndex() > best.startIndex()) {
            best = block;
          }
        }
      }
      return Optional.ofNullable(best);
    }

    public Optional<BlockDescriptor> findParentBlock(int blockStartIndex) {
      BlockDescriptor block = byStart.get(blockStartIndex);
      if (block == null || block.parentStartIndex() == null) {
        return Optional.empty();
      }
      return Optional.ofNullable(byStart.get(block.parentStartIndex()));
    }
  }

  public static final class OptionalInt {
    private static final OptionalInt EMPTY = new OptionalInt(null);
    private final Integer value;

    private OptionalInt(Integer value) {
      this.value = value;
    }

    public static OptionalInt of(int value) {
      return new OptionalInt(value);
    }

    public static OptionalInt empty() {
      return EMPTY;
    }

    public boolean isPresent() {
      return value != null;
    }

    public int getAsInt() {
      if (value == null) {
        throw new IllegalStateException("OptionalInt is empty");
      }
      return value;
    }
  }

  private static final class Frame {
    private final BlockType type;
    private final int startIndex;
    private final Integer parentStartIndex;
    private int bodyStart = -1;
    private int bodyEnd = -1;
    private int endIndex = -1;
    private final List<Branch> branches = new ArrayList<>();
    private String activeBranchKind;
    private int activeBranchStart = -1;
    private boolean hasElse = false;
    private boolean forBodyOpened = false;
    private boolean forBodyClosed = false;
    private boolean tryBodyStarted = false;
    private boolean sawFinally = false;

    private Frame(BlockType type, int startIndex, Integer parentStartIndex) {
      this.type = type;
      this.startIndex = startIndex;
      this.parentStartIndex = parentStartIndex;
    }

    private BlockDescriptor finish() {
      Range body = new Range(bodyStart < 0 ? startIndex : bodyStart, bodyEnd < bodyStart ? endIndex - 1 : bodyEnd);
      return new BlockDescriptor(type, startIndex, endIndex, parentStartIndex, body, branches);
    }
  }

  public static Snapshot validateControlFlow(List<EventSpec> specs) {
    Objects.requireNonNull(specs, "specs");
    ArrayDeque<Frame> stack = new ArrayDeque<>();
    List<BlockDescriptor> blocks = new ArrayList<>();

    for (int i = 0; i < specs.size(); i++) {
      EventSpec spec = specs.get(i);
      if (spec.getType() != EventSpec.EventType.CONTROL) {
        continue;
      }
      AslControlSpecFactory.Marker marker = AslControlSpecFactory.Marker.fromCommand(spec.getCommand());
      if (marker == null) {
        continue;
      }

      switch (marker) {
        case IF -> stack.push(new Frame(BlockType.IF, i, parentStart(stack)));
        case THEN -> {
          Frame ifFrame = top(stack, BlockType.IF, marker, i);
          if (ifFrame.activeBranchKind != null) {
            throw fail(marker, i, "duplicate then/branch delimiter for if");
          }
          ifFrame.activeBranchKind = ifFrame.branches.isEmpty() ? "then" : "elseif";
          ifFrame.activeBranchStart = i + 1;
          ifFrame.bodyStart = Math.max(ifFrame.bodyStart, i + 1);
        }
        case ELSEIF -> {
          Frame ifFrame = top(stack, BlockType.IF, marker, i);
          if (ifFrame.hasElse) {
            throw fail(marker, i, "elseif appears after else");
          }
          closeIfBranch(ifFrame, i - 1, marker, i);
          ifFrame.activeBranchKind = null;
          ifFrame.activeBranchStart = -1;
        }
        case ELSE -> {
          Frame ifFrame = top(stack, BlockType.IF, marker, i);
          if (ifFrame.hasElse) {
            throw fail(marker, i, "duplicate else");
          }
          closeIfBranch(ifFrame, i - 1, marker, i);
          ifFrame.hasElse = true;
          ifFrame.activeBranchKind = "else";
          ifFrame.activeBranchStart = i + 1;
          ifFrame.bodyStart = Math.max(ifFrame.bodyStart, i + 1);
        }
        case ENDIF -> {
          Frame ifFrame = top(stack, BlockType.IF, marker, i);
          closeIfBranch(ifFrame, i - 1, marker, i);
          ifFrame.endIndex = i;
          ifFrame.bodyEnd = i - 1;
          stack.pop();
          blocks.add(ifFrame.finish());
        }
        case WHILE -> stack.push(new Frame(BlockType.WHILE, i, parentStart(stack)));
        case DO -> {
          Frame whileFrame = top(stack, BlockType.WHILE, marker, i);
          whileFrame.bodyStart = i + 1;
        }
        case ENDWHILE -> {
          Frame whileFrame = top(stack, BlockType.WHILE, marker, i);
          if (whileFrame.bodyStart < 0) {
            throw fail(marker, i, "while is missing do");
          }
          whileFrame.bodyEnd = i - 1;
          whileFrame.endIndex = i;
          stack.pop();
          blocks.add(whileFrame.finish());
        }
        case FOR -> stack.push(new Frame(BlockType.FOR, i, parentStart(stack)));
        case FOREACH -> stack.push(new Frame(BlockType.FOREACH, i, parentStart(stack)));
        case BLOCK -> {
          if (!stack.isEmpty() && (stack.peek().type == BlockType.FOR || stack.peek().type == BlockType.FOREACH)
              && !stack.peek().forBodyOpened) {
            Frame loopFrame = stack.peek();
            loopFrame.forBodyOpened = true;
            loopFrame.bodyStart = i + 1;
          } else {
            Frame blockFrame = new Frame(BlockType.BLOCK, i, parentStart(stack));
            blockFrame.bodyStart = i + 1;
            stack.push(blockFrame);
          }
        }
        case ENDBLOCK -> {
          if (!stack.isEmpty() && stack.peek().type == BlockType.BLOCK) {
            Frame blockFrame = stack.pop();
            blockFrame.bodyEnd = i - 1;
            blockFrame.endIndex = i;
            blocks.add(blockFrame.finish());
          } else if (!stack.isEmpty() && (stack.peek().type == BlockType.FOR || stack.peek().type == BlockType.FOREACH)
              && stack.peek().forBodyOpened && !stack.peek().forBodyClosed) {
            stack.peek().forBodyClosed = true;
            stack.peek().bodyEnd = i - 1;
          } else {
            throw fail(marker, i, "endblock without matching block");
          }
        }
        case ENDFOR -> {
          Frame loop = top(stack, List.of(BlockType.FOR, BlockType.FOREACH), marker, i);
          if (!loop.forBodyOpened || !loop.forBodyClosed) {
            throw fail(marker, i, "for/foreach must contain block/endblock before endfor");
          }
          loop.endIndex = i;
          stack.pop();
          blocks.add(loop.finish());
        }
        case TRY -> {
          Frame tryFrame = new Frame(BlockType.TRY, i, parentStart(stack));
          tryFrame.tryBodyStarted = true;
          tryFrame.activeBranchKind = "try";
          tryFrame.activeBranchStart = i + 1;
          tryFrame.bodyStart = i + 1;
          stack.push(tryFrame);
        }
        case CATCH -> {
          Frame tryFrame = top(stack, BlockType.TRY, marker, i);
          if (tryFrame.sawFinally) {
            throw fail(marker, i, "catch cannot appear after finally");
          }
          closeTryBranch(tryFrame, i - 1);
          tryFrame.activeBranchKind = "catch";
          tryFrame.activeBranchStart = i + 1;
        }
        case FINALLY -> {
          Frame tryFrame = top(stack, BlockType.TRY, marker, i);
          if (tryFrame.sawFinally) {
            throw fail(marker, i, "duplicate finally");
          }
          closeTryBranch(tryFrame, i - 1);
          tryFrame.sawFinally = true;
          tryFrame.activeBranchKind = "finally";
          tryFrame.activeBranchStart = i + 1;
        }
        case ENDTRY -> {
          Frame tryFrame = top(stack, BlockType.TRY, marker, i);
          closeTryBranch(tryFrame, i - 1);
          tryFrame.endIndex = i;
          tryFrame.bodyEnd = i - 1;
          stack.pop();
          blocks.add(tryFrame.finish());
        }
        case ASYNC -> {
          Frame frame = new Frame(BlockType.ASYNC, i, parentStart(stack));
          frame.bodyStart = i + 1;
          stack.push(frame);
        }
        case ENDASYNC -> closeSimple(stack, blocks, BlockType.ASYNC, marker, i);
        case NOINTERRUPT -> {
          Frame frame = new Frame(BlockType.NOINTERRUPT, i, parentStart(stack));
          frame.bodyStart = i + 1;
          stack.push(frame);
        }
        case ENDNOINTERRUPT -> closeSimple(stack, blocks, BlockType.NOINTERRUPT, marker, i);
        case TRUE, FALSE, DECLARE -> {
          // leaf condition controls
        }
      }
    }

    if (!stack.isEmpty()) {
      Frame frame = stack.peek();
      throw new IllegalArgumentException("Unclosed block " + frame.type + " starting at index " + frame.startIndex);
    }
    Collections.sort(blocks, java.util.Comparator.comparingInt(BlockDescriptor::startIndex));
    return new Snapshot(specs, blocks);
  }

  private static void closeSimple(ArrayDeque<Frame> stack, List<BlockDescriptor> blocks, BlockType type,
                                  AslControlSpecFactory.Marker marker, int index) {
    Frame frame = top(stack, type, marker, index);
    frame.bodyEnd = index - 1;
    frame.endIndex = index;
    stack.pop();
    blocks.add(frame.finish());
  }

  private static void closeTryBranch(Frame tryFrame, int endInclusive) {
    if (tryFrame.activeBranchKind == null || tryFrame.activeBranchStart < 0) {
      return;
    }
    int rangeEnd = Math.max(tryFrame.activeBranchStart - 1, endInclusive);
    tryFrame.branches.add(new Branch(tryFrame.activeBranchKind, new Range(tryFrame.activeBranchStart, rangeEnd)));
    tryFrame.activeBranchKind = null;
    tryFrame.activeBranchStart = -1;
  }

  private static void closeIfBranch(Frame ifFrame, int endInclusive, AslControlSpecFactory.Marker marker, int markerIndex) {
    if (ifFrame.activeBranchKind == null || ifFrame.activeBranchStart < 0) {
      throw fail(marker, markerIndex, "if is missing then branch");
    }
    if (endInclusive < ifFrame.activeBranchStart - 1) {
      throw fail(marker, markerIndex, "empty branch before " + marker.command());
    }
    int rangeStart = ifFrame.activeBranchStart;
    int rangeEnd = Math.max(rangeStart - 1, endInclusive);
    ifFrame.branches.add(new Branch(ifFrame.activeBranchKind, new Range(rangeStart, rangeEnd)));
    ifFrame.activeBranchKind = null;
    ifFrame.activeBranchStart = -1;
  }

  private static Integer parentStart(ArrayDeque<Frame> stack) {
    return stack.isEmpty() ? null : stack.peek().startIndex;
  }

  private static Frame top(ArrayDeque<Frame> stack, BlockType type, AslControlSpecFactory.Marker marker, int index) {
    if (stack.isEmpty() || stack.peek().type != type) {
      throw fail(marker, index, "unexpected " + marker.command());
    }
    return stack.peek();
  }

  private static Frame top(ArrayDeque<Frame> stack, List<BlockType> types, AslControlSpecFactory.Marker marker, int index) {
    if (stack.isEmpty() || !types.contains(stack.peek().type)) {
      throw fail(marker, index, "unexpected " + marker.command());
    }
    return stack.peek();
  }

  private static IllegalArgumentException fail(AslControlSpecFactory.Marker marker, int index, String reason) {
    return new IllegalArgumentException("Invalid control flow at EventSpec index " + index
        + " (" + marker.command() + "): " + reason);
  }
}

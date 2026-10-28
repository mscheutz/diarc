/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.asl.SourceLocation;
import java.io.Serializable;
import java.util.List;
import java.util.Map;

/** Snapshot produced on explicit introspection, not automatically logged argument data. */
public record ExecutionDiagnostic(SourceLocation source, String step, ActionStatus status,
                                  String reason, Map<String, String> arguments,
                                  List<String> callStack) implements Serializable { }

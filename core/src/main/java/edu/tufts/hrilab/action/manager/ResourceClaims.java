package edu.tufts.hrilab.action.manager;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/** A snapshot of the resources a single goal execution requires and holds. */
final class ResourceClaims {
  final Set<Resource> required;
  final Set<Resource> held;
  final Set<Resource> used;

  ResourceClaims(Set<Resource> required, Set<Resource> held) {
    this.required = Set.copyOf(required);
    this.held = Set.copyOf(held);
    Set<Resource> resources = new HashSet<>(required);
    resources.addAll(held);
    used = Set.copyOf(resources);
  }

  boolean conflictsWith(ResourceClaims other) {
    return !Collections.disjoint(held, other.used)
            || !Collections.disjoint(other.held, used);
  }
}

/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;
import java.util.function.Predicate;

public class ChildContexts {
  private final List<Context> children = new ArrayList<>();
  private volatile int nextIndex = 0;
  private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();
  private final ReentrantReadWriteLock.ReadLock readLock = lock.readLock();
  private final ReentrantReadWriteLock.WriteLock writeLock = lock.writeLock();

  /**
   * Get shallow copy. Underlying Contexts are not shared in copy.
   *
   * @return
   */
  public ChildContexts getCopy() {
    readLock.lock();
    try {
      ChildContexts copy = new ChildContexts();
      copy.children.addAll(this.children);
      copy.nextIndex = this.nextIndex;
      return copy;
    } finally {
      readLock.unlock();
    }
  }

  public void add(Context context) {
    writeLock.lock();
    try {
      children.add(context);
    } finally {
      writeLock.unlock();
    }
  }

  public void setChildren(ChildContexts childContexts) {
    writeLock.lock();
    try {
      children.addAll(childContexts.getChildrenContexts());
      nextIndex = childContexts.nextIndex;
    } finally {
      writeLock.unlock();
    }
  }

  public boolean remove(Context context) {
    writeLock.lock();
    try {
      return children.remove(context);
    } finally {
      writeLock.unlock();
    }
  }

  public int getNextIndex() {
    readLock.lock();
    try {
      return nextIndex;
    } finally {
      readLock.unlock();
    }
  }

  public void setNextIndex(int index) {
    writeLock.lock();
    try {
      nextIndex = index;
    } finally {
      writeLock.unlock();
    }
  }

  public Context get(int index) {
    readLock.lock();
    try {
      if (index >= 0 && index < children.size()) {
        return children.get(index);
      } else {
        return null;
      }
    } finally {
      readLock.unlock();
    }
  }


  public Context getCurrent() {
    return get(nextIndex - 1);
  }

  public Context getPrevious() {
    return get(nextIndex - 2);
  }

  public Context getNext() {
    return get(nextIndex);
  }

  public Context getNextAndIncrement() {
    writeLock.lock();
    try {
      if (nextIndex >= 0 && nextIndex < children.size()) {
        Context next = children.get(nextIndex);
        if (next != null) {
          nextIndex++;
        }
        return next;
      } else {
        return null;
      }
    } finally {
      writeLock.unlock();
    }
  }

  public int size() {
    readLock.lock();
    try {
      return children.size();
    } finally {
      readLock.unlock();
    }
  }

  public boolean isEmpty() {
    readLock.lock();
    try {
      return children.isEmpty();
    } finally {
      readLock.unlock();
    }
  }

  /**
   * Get all children contexts in a copied List (shallow copy).
   *
   * @return
   */
  public List<Context> getChildrenContexts() {
    readLock.lock();
    try {
      return new ArrayList<>(children);
    } finally {
      readLock.unlock();
    }
  }

  public void clear() {
    writeLock.lock();
    try {
      nextIndex = 0;
      children.clear();
    } finally {
      writeLock.unlock();
    }
  }

  public void reset() {
    writeLock.lock();
    try {
      nextIndex = 0;
      children.forEach(Context::resetContext);
    } finally {
      writeLock.unlock();
    }
  }

  // reset the child context at and after index
  // TODO: probably need to rename this method
  public void resetRemaining(int index) {
    writeLock.lock();
    try {
      children.stream().filter(child ->(children.indexOf(child) >= index)).forEach(Context::resetContext);
      //nextIndex = index;
    } finally {
      writeLock.unlock();
    }
  }

  public void forEach(Consumer<? super Context> consumer) {
    readLock.lock();
    try {
      children.forEach(consumer);
    } finally {
      readLock.unlock();
    }
  }

  public boolean allMatch(Predicate<? super Context> predicate) {
    readLock.lock();
    try {
      return children.stream().allMatch(predicate);
    } finally {
      readLock.unlock();
    }
  }

  public boolean anyMatch(Predicate<? super Context> predicate) {
    readLock.lock();
    try {
      return children.stream().anyMatch(predicate);
    } finally {
      readLock.unlock();
    }
  }

  // TODO: should this also set the nextIndex or maybe have a separate method that finds first and sets index??
  public Optional<Context> findFirst(Predicate<? super Context> predicate) {
    readLock.lock();
    try {
      return children.stream().filter(predicate).findFirst();
    } finally {
      readLock.unlock();
    }
  }

  /**
   * Remove matching children. Note that this does not change the nextIndex counter.
   *
   * @param predicate
   * @return
   */
  public boolean removeIf(Predicate<? super Context> predicate) {
    writeLock.lock();
    try {
      return children.removeIf(predicate);
    } finally {
      writeLock.unlock();
    }
  }

  public void insert(Context newContext) {
    insert(newContext, nextIndex);
  }

  public void insert(Context newContext, int location) {
    writeLock.lock();
    try {
      children.add(location, newContext);
    } finally {
      writeLock.unlock();
    }

  }

  public int findFirstIndexOf(Predicate<? super Context> predicate) {
    readLock.lock();
    try {
      Optional<Context> child = children.stream().filter(predicate).findFirst();
      return child.map(children::indexOf).orElse(-1);
    } finally {
      readLock.unlock();
    }
  }

  public int indexOf(Context context) {
    readLock.lock();
    try {
      return children.indexOf(context);
    } finally {
      readLock.unlock();
    }

  }

}

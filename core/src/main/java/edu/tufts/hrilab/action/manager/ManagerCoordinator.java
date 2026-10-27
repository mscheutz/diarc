package edu.tufts.hrilab.action.manager;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Supplier;

/**
 * Serializes state decisions. Effects run on the calling thread only after the
 * state command has finished, so an interpreter or callback can reenter the manager.
 */
final class ManagerCoordinator implements AutoCloseable {
  private volatile Thread owner;
  private List<Runnable> effects;
  private final ExecutorService executor = Executors.newSingleThreadExecutor(task -> {
    Thread thread = new Thread(task, "goal-manager-coordinator");
    thread.setDaemon(true);
    owner = thread;
    return thread;
  });

  boolean isOwner() {
    return Thread.currentThread() == owner;
  }

  <T> T call(Supplier<T> command) {
    if (isOwner()) {
      return command.get();
    }
    Future<Transition<T>> future = executor.submit(() -> {
      effects = new ArrayList<>();
      try {
        return new Transition<>(command.get(), effects, null);
      } catch (RuntimeException | Error failure) {
        return new Transition<>(null, effects, failure);
      } finally {
        effects = null;
      }
    });
    boolean interrupted = false;
    Transition<T> transition;
    try {
      while (true) {
        try {
          transition = future.get();
          break;
        } catch (InterruptedException e) {
          // A submitted command may already be committed; finish its effects before returning.
          interrupted = true;
        } catch (ExecutionException e) {
          throw new IllegalStateException("Goal manager coordinator failed", e.getCause());
        }
      }
    } finally {
      if (interrupted) {
        Thread.currentThread().interrupt();
      }
    }
    Throwable failure = transition.failure;
    for (Runnable effect : transition.effects) {
      try {
        effect.run();
      } catch (RuntimeException | Error e) {
        if (failure == null) {
          failure = e;
        } else if (failure != e) {
          failure.addSuppressed(e);
        }
      }
    }
    if (failure instanceof RuntimeException) {
      throw (RuntimeException) failure;
    }
    if (failure instanceof Error) {
      throw (Error) failure;
    }
    return transition.value;
  }

  void run(Runnable command) {
    call(() -> {
      command.run();
      return null;
    });
  }

  void defer(Runnable effect) {
    if (!isOwner() || effects == null) {
      throw new IllegalStateException("Effects must be registered by a coordinator command");
    }
    effects.add(effect);
  }

  @Override
  public void close() {
    executor.shutdown();
  }

  private static final class Transition<T> {
    final T value;
    final List<Runnable> effects;
    final Throwable failure;

    Transition(T value, List<Runnable> effects, Throwable failure) {
      this.value = value;
      this.effects = effects;
      this.failure = failure;
    }
  }
}

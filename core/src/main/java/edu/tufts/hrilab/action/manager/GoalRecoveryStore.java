package edu.tufts.hrilab.action.manager;

import com.google.gson.Gson;
import com.google.gson.JsonParseException;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import edu.tufts.hrilab.action.goal.GoalRecoveryRecord;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.channels.FileLock;
import java.nio.channels.OverlappingFileLockException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.UUID;
import java.util.function.Supplier;

/** Serialize snapshot capture and atomic replacement, so an older writer cannot overwrite newer intent. */
final class GoalRecoveryStore implements AutoCloseable {
  private static final long MAX_BYTES = 16 * 1024 * 1024;
  private final Path file;
  private final Gson gson = new Gson();
  private FileChannel lockChannel;
  private FileLock writerLock;

  private static final class Snapshot {
    int version = 1;
    boolean cleanShutdown;
    List<GoalRecoveryRecord> records = new ArrayList<>();
  }

  GoalRecoveryStore(Path file) {
    this.file = file.toAbsolutePath();
  }

  synchronized void acquireWriter() throws IOException {
    Files.createDirectories(file.getParent());
    lockChannel = FileChannel.open(file.resolveSibling(file.getFileName() + ".lock"),
            StandardOpenOption.CREATE, StandardOpenOption.WRITE);
    try {
      writerLock = lockChannel.tryLock();
      if (writerLock == null) {
        throw new IOException("Goal recovery checkpoint already has a writer");
      }
    } catch (IOException | OverlappingFileLockException e) {
      lockChannel.close();
      throw new IOException("Cannot acquire goal recovery writer lock", e);
    }
  }

  synchronized List<GoalRecoveryRecord> load() throws IOException {
    if (!Files.exists(file)) {
      return List.of();
    }
    if (Files.size(file) > MAX_BYTES) {
      throw new IOException("Goal recovery checkpoint exceeds 16 MiB");
    }
    try {
      byte[] bytes;
      try (java.io.InputStream input = Files.newInputStream(file)) {
        bytes = input.readNBytes((int) MAX_BYTES + 1);
      }
      if (bytes.length > MAX_BYTES) {
        throw new IOException("Goal recovery checkpoint exceeds 16 MiB");
      }
      JsonObject object = JsonParser.parseString(new String(bytes, StandardCharsets.UTF_8)).getAsJsonObject();
      if (!object.has("version") || !object.has("records") || !object.has("cleanShutdown")) {
        throw new IOException("Incomplete goal recovery checkpoint");
      }
      Snapshot snapshot = gson.fromJson(object, Snapshot.class);
      if (snapshot == null || snapshot.version != 1 || snapshot.records == null) {
        throw new IOException("Unsupported or incomplete goal recovery checkpoint");
      }
      Set<String> identifiers = new HashSet<>();
      List<GoalRecoveryRecord> records = new ArrayList<>();
      for (GoalRecoveryRecord record : snapshot.records) {
        if (record == null || record.sessionId == null || record.goalId < 0
                || record.predicate == null || record.predicate.isBlank()
                || record.status == null || record.status.isTerminated() || record.claims == null
                || record.claims.stream().anyMatch(claim -> claim == null || claim.resource == null)
                || !identifiers.add(record.sessionId + ":" + record.goalId)) {
          throw new IOException("Invalid or duplicate goal recovery record");
        }
        UUID.fromString(record.sessionId);
        records.add(new GoalRecoveryRecord(record.sessionId, record.goalId, record.predicate,
                record.status, record.claims));
      }
      return List.copyOf(records);
    } catch (JsonParseException | IllegalArgumentException | IllegalStateException e) {
      throw new IOException("Malformed goal recovery checkpoint", e);
    }
  }

  synchronized void save(Supplier<List<GoalRecoveryRecord>> capture, boolean cleanShutdown) throws IOException {
    if (writerLock == null || !writerLock.isValid()) {
      throw new IOException("Goal recovery writer lock is not held");
    }
    Snapshot snapshot = new Snapshot();
    snapshot.cleanShutdown = cleanShutdown;
    snapshot.records = capture.get();
    byte[] data = gson.toJson(snapshot).getBytes(StandardCharsets.UTF_8);
    if (data.length > MAX_BYTES) {
      throw new IOException("Goal recovery checkpoint exceeds 16 MiB");
    }
    Path parent = file.getParent();
    Files.createDirectories(parent);
    Path temporary = Files.createTempFile(parent, ".goal-recovery-", ".json");
    try {
      try (FileChannel channel = FileChannel.open(temporary, StandardOpenOption.WRITE)) {
        ByteBuffer bytes = ByteBuffer.wrap(data);
        while (bytes.hasRemaining()) {
          channel.write(bytes);
        }
        channel.force(true);
      }
      Files.move(temporary, file, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
      try (FileChannel directory = FileChannel.open(parent, StandardOpenOption.READ)) {
        directory.force(true);
      }
    } finally {
      Files.deleteIfExists(temporary);
    }
  }

  @Override
  public synchronized void close() throws IOException {
    if (lockChannel != null) {
      lockChannel.close();
    }
  }
}

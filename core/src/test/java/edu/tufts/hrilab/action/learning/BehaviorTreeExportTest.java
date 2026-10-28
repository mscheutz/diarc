/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.learning;

import edu.tufts.hrilab.action.bt.BehaviorTreeBindings;
import edu.tufts.hrilab.action.bt.BehaviorTreeConverter;
import edu.tufts.hrilab.fol.Factory;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.Assert.*;
import static org.junit.Assume.assumeTrue;

public class BehaviorTreeExportTest {
  @Rule public TemporaryFolder temporary = new TemporaryFolder();
  private Path directory;
  private Path xml;
  private Path bindings;
  private String oldXml;
  private String oldBindings;
  private BehaviorTreeLearning replacement;

  private static BehaviorTreeLearning learner(String leaf, String action) {
    BehaviorTreeLearning learning = new BehaviorTreeLearning();
    learning.begin("exportTree", Factory.createSymbol("self:agent"), "Sequence",
        "{\"" + leaf + "\":{\"action\":\"" + action + "\",\"synchronous\":true}}");
    learning.addLeaf("exportTree", "root", "step", leaf, "{}", 0);
    return learning;
  }

  @Before
  public void prepare() throws IOException {
    directory = temporary.newFolder().toPath();
    xml = directory.resolve("tree.xml");
    bindings = directory.resolve("bindings.json");
    learner("Old", "oldAction").export("exportTree", xml.toString(), bindings.toString());
    oldXml = Files.readString(xml);
    oldBindings = Files.readString(bindings);
    replacement = learner("New", "newAction");
  }

  private void assertPreviousPairIsUsable() throws IOException {
    assertEquals(oldXml, Files.readString(xml));
    assertEquals(oldBindings, Files.readString(bindings));
    assertLoadable("oldAction");
  }

  private void assertLoadable(String action) throws IOException {
    try (var reader = Files.newBufferedReader(bindings)) {
      String asl = new BehaviorTreeConverter(BehaviorTreeBindings.read(reader)).convert(xml, "reloadedTree");
      assertTrue(asl.contains("act:" + action));
      assertFalse(BehaviorTreeConverter.parse(asl).isEmpty());
    }
  }

  private void assertNoTemporaryFiles() throws IOException {
    try (var files = Files.list(directory)) {
      assertEquals(List.of(bindings, xml).stream().sorted().toList(), files.sorted().toList());
    }
  }

  private interface ExportOperation {
    void run() throws IOException;
  }

  private static IOException exportFails(ExportOperation operation) {
    try {
      operation.run();
    } catch (IOException expected) {
      return expected;
    }
    throw new AssertionError("Expected export to report its I/O failure");
  }

  @Test
  public void xmlReplacementFailureRestoresThePreviousUsablePair() throws IOException {
    AtomicInteger moves = new AtomicInteger();
    IOException error = exportFails(() -> replacement.export("exportTree",
        xml.toString(), bindings.toString(), (source, destination) -> {
          if (moves.incrementAndGet() == 2) {
            throw new IOException("injected XML replacement failure");
          }
          Files.move(source, destination, StandardCopyOption.REPLACE_EXISTING);
        }));
    assertEquals("injected XML replacement failure", error.getMessage());
    assertPreviousPairIsUsable();
    assertNoTemporaryFiles();
  }

  @Test
  public void failureAfterReplacingXmlAlsoRestoresBothFiles() throws IOException {
    AtomicInteger moves = new AtomicInteger();
    exportFails(() -> replacement.export("exportTree",
        xml.toString(), bindings.toString(), (source, destination) -> {
          Files.move(source, destination, StandardCopyOption.REPLACE_EXISTING);
          if (moves.incrementAndGet() == 2) {
            throw new IOException("injected failure after XML replacement");
          }
        }));
    assertPreviousPairIsUsable();
    assertNoTemporaryFiles();
  }

  @Test
  public void xmlStagingFailureCannotOverwriteExistingBindings() throws IOException {
    exportFails(() -> replacement.export("exportTree",
        xml.resolve("not-a-directory.xml").toString(), bindings.toString()));
    assertPreviousPairIsUsable();
    assertNoTemporaryFiles();
  }

  @Test
  public void failedFirstExportRemovesNewlyPublishedFiles() throws IOException {
    Path newXml = directory.resolve("new.xml");
    Path newBindings = directory.resolve("new.json");
    AtomicInteger moves = new AtomicInteger();
    exportFails(() -> replacement.export("exportTree",
        newXml.toString(), newBindings.toString(), (source, destination) -> {
          if (moves.incrementAndGet() == 2) {
            throw new IOException("injected XML replacement failure");
          }
          Files.move(source, destination, StandardCopyOption.REPLACE_EXISTING);
        }));
    assertFalse(Files.exists(newXml));
    assertFalse(Files.exists(newBindings));
    assertPreviousPairIsUsable();
    assertNoTemporaryFiles();
  }

  @Test
  public void recoveryFailureReportsAndRetainsTheOriginalXmlBackup() throws IOException {
    AtomicInteger moves = new AtomicInteger();
    IOException error = exportFails(() -> replacement.export("exportTree",
        xml.toString(), bindings.toString(), (source, destination) -> {
          int move = moves.incrementAndGet();
          if (move == 3) {
            throw new IOException("injected XML recovery failure");
          }
          Files.move(source, destination, StandardCopyOption.REPLACE_EXISTING);
          if (move == 2) {
            throw new IOException("injected failure after XML replacement");
          }
        }));
    assertEquals(1, error.getSuppressed().length);
    Path backup;
    try (var files = Files.list(directory)) {
      var backups = files.filter(path -> path.getFileName().toString().startsWith(".diarc-bt-backup-")).toList();
      assertEquals(1, backups.size());
      backup = backups.get(0);
    }
    assertTrue(error.getSuppressed()[0].getMessage().contains(backup.toString()));
    assertEquals(oldXml, Files.readString(backup));
    assertEquals(oldBindings, Files.readString(bindings));
    Files.move(backup, xml, StandardCopyOption.REPLACE_EXISTING);
    assertPreviousPairIsUsable();
    assertNoTemporaryFiles();
  }

  @Test
  public void successfulReplacementPublishesALoadablePairAndRemovesBackups() throws IOException {
    assertTrue(replacement.export("exportTree", xml.toString(), bindings.toString()));
    assertEquals(replacement.getXml("exportTree"), Files.readString(xml));
    assertEquals(replacement.getBindingsJson("exportTree"), Files.readString(bindings));
    assertLoadable("newAction");
    assertNoTemporaryFiles();
  }

  @Test
  public void replacingExistingFilesPreservesTheirPosixPermissions() throws IOException {
    assumeTrue(Files.getFileStore(directory).supportsFileAttributeView("posix"));
    var permissions = PosixFilePermissions.fromString("rw-r-----");
    Files.setPosixFilePermissions(xml, permissions);
    Files.setPosixFilePermissions(bindings, permissions);
    replacement.export("exportTree", xml.toString(), bindings.toString());
    assertEquals(permissions, Files.getPosixFilePermissions(xml));
    assertEquals(permissions, Files.getPosixFilePermissions(bindings));
    assertLoadable("newAction");
    assertNoTemporaryFiles();
  }
}

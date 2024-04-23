/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.gui;

/**
 * The only purpose of this class is to store a file path alongside the file name,
 * for use in a JTree, as user object.
 */
public class FileWrapper {
  private String name;
  private String path;
  private boolean isDirectory;

  public FileWrapper(String n, String p) {
    name = n;
    path = p;
    isDirectory = false;
  }

  public FileWrapper(String n, String p, boolean directory) {
    name = n;
    path = p;
    isDirectory = directory;
  }

  public String getPath() {
    return path;
  }

  public boolean isDirectory() {
    return isDirectory;
  }

  @Override
  public String toString() {
    return name;
  }
}

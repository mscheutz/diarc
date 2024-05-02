/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.cache;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.sqlite.SQLiteConfig;
import org.sqlite.SQLiteOpenMode;

import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.sql.PreparedStatement;
import java.sql.Connection;
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.util.Map;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.io.IOException;

import java.net.URL;
import java.net.URI;
import java.net.URISyntaxException;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.util.resource.Resources;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;

public class CacheDatabase {
  static private Logger log = LoggerFactory.getLogger(CacheDatabase.class);
  static private String cacheDirectoryPath = "config/edu/tufts/hrilab/slug/parsing/cache";
  private String tableName = "cache";
  private String defaultCacheName = "diarc-parser-cache";
  private String file = null;

  private Connection connection;
  private Statement statement;
  private PreparedStatement getStatement;
  private PreparedStatement insertStatement;

  public CacheDatabase (String name, String[] loadPaths, boolean persist) {
    String homeDirPath;

    if (persist) {
      homeDirPath = System.getProperty("user.home");
      try {
        homeDirPath = Paths.get(homeDirPath, ".diarc").toString();
        Files.createDirectories(Paths.get(homeDirPath));
      } catch (IOException ex) {
        log.error("Couldn't create .diarc directory in user home directory", ex);
        return;
      }

      if (name!= null && name.equals(defaultCacheName)) {
        log.warn("The cache name \"" + defaultCacheName + "\" matches is the default cache name. This may overwrite existing cache data.");
      }

      if (name != null) {
        file = Paths.get(homeDirPath, name + ".sqlite").toString();
      } else {
        file = Paths.get(homeDirPath, defaultCacheName + ".sqlite").toString();
      }
    }

    initializeDatabase();
    initializeTable();
    initializeStatements();
    if (loadPaths != null && loadPaths.length > 0) {
      loadCache(loadPaths);
    }
  }

  /**
   * Initializes a SQLite database connection.
   * If a file path is provided, it initializes the database at that path.
   * If no file path is provided, it initializes an in-memory database.
   */
  private void initializeDatabase () {
    String connectionString = file == null ? "jdbc:sqlite::memory:" : "jdbc:sqlite:" + file;

    try {
      SQLiteConfig config = new SQLiteConfig();
      if (file != null) {
        config.setOpenMode(SQLiteOpenMode.FULLMUTEX);
        log.info("Initializing parser cache database with file: " + file);
      } else {
        log.info("Initializing parser cache database in memory");
      }
      connection = DriverManager.getConnection(connectionString, config.toProperties());
      statement = connection.createStatement();
    } catch (SQLException ex) {
      log.error("Couldn't connect to database.", ex);
    }
  }

  /**
   * Retrieves the absolute path of the specified database file.
   * If the file is not found in the cache directory, returns null.
   *
   * @param file The name of the database file.
   * @return The absolute path of the database file if found; otherwise, null.
   */
  private static String getDatabasePath (String file) {
    String cachePath = Resources.createFilepath(CacheDatabase.cacheDirectoryPath, file);
    URL u = CacheDatabase.class.getResource(cachePath);
    URI uri = null;
    if (u == null) {
      log.warn("No pre-existing " + cachePath + " exists, skipping...");
      return null;
    }
    try {
      uri = u.toURI();
    } catch (URISyntaxException ex) {
      log.error("Error converting URL to URI for " + cachePath, ex);
      return null;
    }
    Path path = Paths.get(uri);
    return path.toString();
  }

  /**
   * Initializes a table in the SQLite database if it does not already exist.
   * Creates an index on the words column of it does not already exist.
   * The table consists of columns: words (TEXT), utterance (JSONB), and created (BIGINT).
   */
  private void initializeTable () {
    String tableQuery = "CREATE TABLE IF NOT EXISTS " + tableName + " ( words TEXT NOT NULL, utterance JSONB NOT NULL, created BIGINT NOT NULL);";
    String indexQuery = "CREATE INDEX IF NOT EXISTS cache_words_idx ON " + tableName + " (words);";
    boolean success = false;
    boolean indexSuccess = false;

    try {
      success = statement.execute(tableQuery);
    } catch (SQLException ex) {
      log.error("Couldn't create table", ex);
    } catch (Exception ex) {
      log.error("Error creating table", ex);
    }

    try {
      indexSuccess = statement.execute(indexQuery);
    } catch (SQLException ex) {
      log.error("Couldn't create index on table", ex);
    } catch (Exception ex) {
      log.error("Error creating index on table", ex);
    }

    if (success && indexSuccess) {
      log.debug("Created " + tableName + "  table and index");
    }
  }

  /**
   * Initializes PreparedStatements for retrieving and inserting data into the SQLite database.
   * PreparedStatement for retrieving data selects the latest utterance for the given words.
   * PreparedStatement for inserting data ignores insertion if a conflicting entry already exists.
   */
  private void initializeStatements () {
    String getQuery = "SELECT utterance FROM " + tableName + " WHERE words = ? ORDER BY created DESC LIMIT 1;";
    String insertQuery = "INSERT OR IGNORE INTO " + tableName + " ( words, utterance, created) VALUES (?, ?, ?);";
    try {
      getStatement = connection.prepareStatement(getQuery);
    } catch (SQLException ex) {
      log.error("Couldn't create PreparedStatement for get query", ex);
    }
    try {
      insertStatement = connection.prepareStatement(insertQuery);
    } catch (SQLException ex) {
      log.error("Couldn't create PreparedStatement for insert query", ex);
    }
  }

  /**
   * If cache paths are provided, the persistent cache database will be emptied
   * and populated by the rows in the loadPaths databases. In the case of a non-
   * persistent cache, this will simply load rows into the in-memory database.
   *
   * @param loadPaths Cache database files to load entries from
   */
  private void loadCache(String[] loadPaths) {
    String eraseQuery = "DELETE FROM " + tableName + " WHERE words IS NOT NULL;";
    String selectQuery = "SELECT words, utterance, created FROM " + tableName + " WHERE words IS NOT NULL;";
    boolean eraseSuccess = false;
    String dbPath;

    //wipe persistent database and load from loadPaths
    try {
      eraseSuccess = statement.execute(eraseQuery);
    } catch (SQLException ex) {
      log.error("Couldn't delete all rows in table", ex);
    } catch (Exception ex) {
      log.error("Error deleting all rows in table", ex);
    }
    if (eraseSuccess) {
      log.warn("Deleted all rows in cache file " + file);
    }
    for (String loadPath : loadPaths) {
      dbPath = CacheDatabase.getDatabasePath(loadPath);
      if (dbPath == null) {
        continue;
      }
      String connectionString = "jdbc:sqlite:" + dbPath;
      Statement loadStatement = null;
      try {
        SQLiteConfig config = new SQLiteConfig();
        config.setOpenMode(SQLiteOpenMode.FULLMUTEX);
        log.debug("Loading cache database with file: " + dbPath);
        Connection loadConnection = DriverManager.getConnection(connectionString, config.toProperties());
        loadStatement = loadConnection.createStatement();
      } catch (SQLException ex) {
        log.error("Couldn't connect to database.", ex);
        continue;
      }
      ResultSet results = null;
      int count = 0;
      try {
        results = loadStatement.executeQuery(selectQuery);
      } catch (SQLException ex) {
        log.error("Couldn't execute SELECT query while loading from database.", ex);
        continue;
      }
      try {
        while (results.next()) {
          String words = results.getString("words");
          String utterance = results.getString("utterance");
          long created = results.getLong("created");
          boolean success = false;
          insertStatement.setString(1, words);
          insertStatement.setString(2, utterance);
          insertStatement.setLong(3, created);
          success = insertStatement.execute();
          if (success) {
            log.debug("Added cache entry for [" + words + "] = " + utterance);
          }
          count++;
        }
      } catch (SQLException ex) {
        log.error("Couldn't add utterance entry to database", ex);
      }
      log.debug("Loaded " + count + " cache " + (count == 1 ? "entry" : "entries") + " from " + dbPath);
    }
  }

  /**
   * Adds an Utterance entry to the cache database.
   *
   * @param entry The Utterance object to add to the cache.
   */
  public void add (Utterance entry) {
    boolean success = false;
    String words = entry.getWordsAsString();
    List<String> wordsList = entry.getWords();
    UtteranceType type = entry.getType();
    Symbol semantics = entry.getSemantics();
    List<Term> indirectSemantics = entry.getIndirectSemantics();
    List<Map<Variable, Symbol>> bindings = entry.getBindings();
    CachedUtterance cached = new CachedUtterance(wordsList, type, semantics, indirectSemantics, bindings);

    try {
      insertStatement.setString(1, words);
      insertStatement.setString(2, cached.toJSON());
      insertStatement.setLong(3, System.currentTimeMillis());
      success = insertStatement.execute();
    } catch (SQLException ex) {
      log.error("Couldn't add utterance entry to database", ex);
    }
    if (success) {
      log.debug("Entry added to cache for [" + words + "] = " + cached.toJSON());
    }
  }

  /**
   * Retrieves an Utterance entry from the cache database based on the words in the incoming Utterance.
   * If the entry exists in the cache, updates the incoming Utterance with cached values.
   *
   * @param incoming The Utterance object containing the words to retrieve from the cache.
   * @return The Utterance object with updated values if found in the cache; otherwise, returns the incoming Utterance unchanged.
   */
  public Utterance get (Utterance incoming) {
    String words = incoming.getWordsAsString();
    ResultSet results = null;
    CachedUtterance cachedUtterance = null;
    String json;
    Predicate semantics;

    try {
      getStatement.setString(1, words);
      results = getStatement.executeQuery();
    } catch (SQLException ex) {
      log.error("Couldn't execute SELECT query on cache database.", ex);
    }

    try {
      if (results.next()) {
        json = results.getString("utterance");
        cachedUtterance = CachedUtterance.fromJSON(json);
        log.debug("Cache hit for [" + words + "] = " + json);
      }
    } catch (SQLException ex) {
      log.error("Couldn't get utterance from ResultSet", ex);
    }

    if (cachedUtterance != null) {
      incoming.setType(cachedUtterance.getType());
      semantics = cachedUtterance.getSemantics();
      semantics.set(0, incoming.getAddressee());
      incoming.setSemantics(semantics);
      incoming.setIndirectSemantics(cachedUtterance.getIndirectSemantics());
      incoming.setBindings(cachedUtterance.getBindings());
    }

    return incoming;
  }
}
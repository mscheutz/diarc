/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.belief.sql;

import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.sqlite.SQLiteConfig;
import org.sqlite.SQLiteOpenMode;

import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.ArrayList;
import java.util.List;

public class SQL {
  private Logger log = LoggerFactory.getLogger(SQL.class);
  private java.sql.Connection sqlconn;
  private Statement stmt;

  public SQL(String dbpath) {
    try {
      //loading mySQL driver
      Class.forName("org.sqlite.JDBC");
      //set up default database connection in memory
      sqlconn = DriverManager.getConnection("jdbc:sqlite::memory:");
      stmt = sqlconn.createStatement();
    } catch (SQLException | ClassNotFoundException e) {
      log.error("Couldn't connect to database.", e);
    }

    //create SQLite database linked to a specific path. If the database already exists, it will not
    //be reinitialized, so if a fresh database is required, change the name or manually delete the database first.
    if (dbpath != null) {
      String arr[] = dbpath.split("/");
      String dbname = arr[arr.length - 1];
      try {
        SQLiteConfig config = new SQLiteConfig();
        config.setOpenMode(SQLiteOpenMode.FULLMUTEX);
        sqlconn = DriverManager.getConnection("jdbc:sqlite:" + dbpath, config.toProperties());
        stmt = sqlconn.createStatement();
        log.info("Current database: " + sqlconn.getCatalog());
        log.debug("Database on disk as '" + dbname + "'");
      } catch (SQLException e) {
        log.error("Error attaching database.", e);
      }

      log.debug("Database name is " + dbname);
    }
  }

  public void assertSQLString(String str, String table, String source) {
      long startTime = System.currentTimeMillis();
      try {
          //put into database
          String query;
          str = str.replace("'", "''");

          //query = "SELECT belief FROM '" + table + "' WHERE endTime IS NULL AND belief LIKE '" + str + "'";
          //ResultSet resultSet = stmt.executeQuery(query);
          //if (!resultSet.next()) {
          log.debug("SQLString is: "+str+" - "+startTime);

          //todo: update source
          if (str.contains(":-")) {
              query = "INSERT INTO '" + table + "' (belief, startTime, endTime, type, prob, source) VALUES ('" + str + "'," + startTime + ",NULL,'rule',"+1.0+","+"'"+source+"'"+")";
          } else {
              query = "INSERT INTO '" + table + "' (belief, startTime, endTime, type, prob, source) VALUES ('" + str + "'," + startTime + ",NULL,'fact',"+1.0+","+"'"+source+"'"+")";
          }
          stmt.execute(query);
          //}

      } catch (SQLException e) {
          log.error(String.format("Belief Component String Assertion Error when asserting '%s': ", str), e);
      }
  }

  public void assertSQLString(String str, String table) {
      assertSQLString(str, table, "source()");
  }

  public void retractSQLString(String str, String table) {
    long retractTime = System.currentTimeMillis();
    try {
      String belief = str.replaceAll("\\b[A-Z][a-zA-Z]*", "%");
      log.debug(str + " : " + belief);
      String type = "fact";
      String query = "";
      if (belief.contains(":-")) {
        type = "rule";
      }

      if (table.equals(MemoryLevel.WORKING.name())) {
//        query = "UPDATE '"+MemoryLevel.UNIVERSAL+"' SET endTime = "+retractTime+" WHERE endTime IS NULL AND type = '"+type+"' AND belief LIKE '" + belief + "'";
        query = "DELETE FROM '"+MemoryLevel.UNIVERSAL+"' WHERE endTime IS NULL AND type = '" + type + "' AND belief LIKE '" + belief + "'";
        stmt.executeUpdate(query);
//        query = "UPDATE '"+MemoryLevel.EPISODIC+"' SET endTime = "+retractTime+" WHERE endTime IS NULL AND type = '"+type+"' AND belief LIKE '" + belief + "'";
        query = "DELETE FROM '"+MemoryLevel.EPISODIC+"' WHERE endTime IS NULL AND type = '" + type + "' AND belief LIKE '" + belief + "'";
        stmt.executeUpdate(query);
      } else if (table.equals(MemoryLevel.EPISODIC.name())){
//        query = "UPDATE '"+MemoryLevel.UNIVERSAL+"' SET endTime = "+retractTime+" WHERE endTime IS NULL AND type = '"+type+"' AND belief LIKE '" + belief + "'";
        query = "DELETE FROM '"+MemoryLevel.UNIVERSAL+"' WHERE endTime IS NULL AND type = '" + type + "' AND belief LIKE '" + belief + "'";
        stmt.executeUpdate(query);
      }

      //query = "UPDATE '" + table + "' SET endTime = " + retractTime + " WHERE endTime IS NULL AND type = '" + type + "' AND belief LIKE '" + belief + "'";
      query = "DELETE FROM '"+table+"' WHERE endTime IS NULL AND type = '" + type + "' AND belief LIKE '" + belief + "'";
      stmt.executeUpdate(query);
    } catch (SQLException e) {
      log.error(String.format("Belief Component String Retraction Error when retracting '%s': ", str), e);
    }
  }

  public String joinConcentricLevels(MemoryLevel memoryLevel, String filter) {
    String sql="";
    switch (memoryLevel) {
      case WORKING:
        sql += "SELECT * FROM '"+MemoryLevel.EPISODIC+"' "+filter+" UNION ";
        sql += "SELECT * FROM '"+MemoryLevel.UNIVERSAL+"' "+filter+" UNION ";
        break;
      case EPISODIC:
        sql += "SELECT * FROM '"+MemoryLevel.UNIVERSAL+"' "+filter+" UNION ";
        break;
    }
    sql += "SELECT * FROM '"+memoryLevel+"' "+filter;
    log.debug(sql);
    return sql;
  }

  public ResultSet getTableResultSet(MemoryLevel memoryLevel) {
    ResultSet resultSet = null;
    try {
      //construct statement to issue SQL query to database
      String query = joinConcentricLevels(memoryLevel,"WHERE endTime IS NULL")+" ORDER BY startTime";
      resultSet =stmt.executeQuery(query);
    } catch (SQLException e) {
      log.error("getTableResultSet Error:", e);
    }
    return resultSet;
  }


  public ResultSet getTableResultSet(String table) {
    ResultSet resultSet = null;
    try {
      //construct statement to issue SQL query to database
      String query = "SELECT * FROM '" + table + "' WHERE endTime IS NULL ORDER BY startTime";
      resultSet =stmt.executeQuery(query);
    } catch (SQLException e) {
      log.error("getTableResultSet Error:", e);
    }
    return resultSet;
  }

  public ResultSet getFacts(MemoryLevel memoryLevel) {
    ResultSet resultSet = null;
    try {
      //construct statement to issue SQL query to database
      String query = joinConcentricLevels(memoryLevel, "WHERE type='fact' AND endTime IS NULL")+" ORDER BY startTime";
      resultSet = stmt.executeQuery(query);
    } catch (SQLException e) {
      log.error("[facts] Error.", e);
    }
    return resultSet;
  }

  public ResultSet getRules(MemoryLevel memoryLevel) {
    ResultSet resultSet = null;
    try {
      //construct statement to issue SQL query to database
      String query = joinConcentricLevels(memoryLevel,"WHERE type='rule' AND endTime IS NULL")+" ORDER BY startTime";
      resultSet = stmt.executeQuery(query);
    } catch (SQLException e) {
      log.error("[rules] Error.", e);
    }
    return resultSet;
  }

  public ResultSet getHistory(MemoryLevel memoryLevel) {
    ResultSet resultSet = null;
    try {
      //construct statement to issue SQL query to database
      String query = joinConcentricLevels(memoryLevel,"WHERE type='fact'")+" ORDER BY startTime";
      resultSet = stmt.executeQuery(query);
    } catch (SQLException e) {
      log.error("[history] Error.", e);
    }
    return resultSet;
  }

  public ResultSet querySql(String sqlStatement) {
    ResultSet resultSet = null;
    try {
      resultSet = stmt.executeQuery(sqlStatement);
    } catch (SQLException e) {
      log.error("Error getting executing SQL query: " + sqlStatement, e);
    }
    return resultSet;
  }

  public int updateSql(String sqlStatement) {
    int result = 0;
    try {
      result = stmt.executeUpdate(sqlStatement);
    } catch (SQLException e) {
      log.error("Error getting executing SQL query: " + sqlStatement, e);
    }
    return result;
  }

  public List<Term> translateResultSet(ResultSet resultSet) {
    List<Term> beliefSet = new ArrayList<>();

    try {
      while (resultSet.next()) {
        beliefSet.add(Factory.createPredicate(resultSet.getString(1)));
      }
    } catch (SQLException e) {
      log.error("[translateResultSet] Error.", e);
    }
    return beliefSet;
  }

  public List<Pair<Term, List<Term>>> translateRuleSet(ResultSet resultSet) {
    List<Pair<Term, List<Term>>> beliefSet = new ArrayList<>();

    try {
      while (resultSet.next()) {
        String[] rule = resultSet.getString(1).split(":-");
        Term head = Factory.createPredicate(rule[0]);
        Predicate premises;
        if (rule[1].startsWith("(")) {
          premises = Factory.createPredicate("tmp" + rule[1]);
        } else {
          premises = Factory.createPredicate("tmp("+rule[1]+")");
        }
        List<Term> body = new ArrayList(premises.getArgs());
        Pair<Term, List<Term>> pair = Pair.of(head, body);
        log.debug("rule: "+pair+" "+resultSet.getString(2));
        beliefSet.add(pair);
      }
    } catch (SQLException e) {
      log.error("translateResultSet Error: ", e);
    }
    return beliefSet;
  }

  public void loadTable(ResultSet resultSet, String tblname) {
    try {
      String query = "CREATE TABLE IF NONE EXISTS " + tblname +" (belief TEXT NOT NULL, startTime BIGINT, endTime BIGINT, type TEXT NOT NULL, prob REAL, source TEXT NOT NULL)";
      stmt.executeUpdate(query);

      while (resultSet.next()) {
        query = "INSERT OR REPLACE INTO " + tblname + " (belief,startTime,endTime,type,source) VALUES ('"+resultSet.getString(1)+"',"+resultSet.getString(2)+","+resultSet.getString(3)+",'"+resultSet.getString(4)+"',"+resultSet.getString(5)+"','"+resultSet.getString(6)+"')";
        stmt.executeUpdate(query);
      }
    } catch (SQLException e) {
      log.error("Error loading SQL table: ", e);
    }
  }

  public void closeConnection(){
    try {
      sqlconn.close();
    } catch (SQLException e) {
      log.error("[closeConnection]",e);
    }
  }
  
  public List<Pair<Long,Long>> queryRecency(Term query) {
    long currentTime = System.currentTimeMillis();
    String sqlStatement = joinConcentricLevels(MemoryLevel.WORKING, "WHERE belief LIKE '"+query.toString()+"' AND type='fact'");
    ResultSet resultSet = querySql(sqlStatement);
    List<Pair<Long,Long>> recencies = new ArrayList<>();
    try {
      while (resultSet.next()) {
        Long startTime = resultSet.getLong(2);
        Long endTime = resultSet.getLong(3);
        if (endTime==0) {
          endTime=currentTime;
        }
        recencies.add(Pair.of(currentTime-startTime,currentTime-endTime));
      }
    } catch (SQLException e) {
      log.error("queryRecency Error: ",e);
    }
    return recencies;
  }
}

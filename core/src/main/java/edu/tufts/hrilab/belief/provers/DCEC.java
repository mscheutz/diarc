/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Interface for the Talos/DCEC logical proof generator.
 * @author luca
 */

package edu.tufts.hrilab.belief.provers;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.net.ssl.HttpsURLConnection;
import java.io.*;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLEncoder;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Interface for the Talos/DCEC logical proof generator
 * Contains the following functionality:
 *  - Solve query via online Talos/DCEC server
 *  - Knowledge database in DCEC syntax
 *  - Prolog to DCEC syntax converter
 * Mimics the way the prolog is used in DIARC/Belief:
 *  - Add facts to the database through the assert asserta, assertz commands
 *  - Remove facts from the database through the retract, retractall commands
 *  - Prove conjecture
 * Features from prolog that are missing:
 *  - Get bindings
 *  - Probably some more stuff...
 */
public class DCEC implements Prover {
  private static Logger log = LoggerFactory.getLogger(DCEC.class);

  /**
   * Knowledge database that is kept in this DCEC Instance.
   */
  private KDB kb;

  /**
   * Constructor
   */
  public DCEC() {
    kb = new KDB();
  }

  /**
   * Solve prolog statement
   * @param statement prolog statement
   * @return true if successful (e.g. proof found, fact added to knowledge base, ...)
   */

  public boolean solve(String statement) {
    if(statement.endsWith(".")) {
      Prolog2DCEC t = new Prolog2DCEC();

      /**
       * Assert = add fact to the knowledge database
       */
      if (statement.startsWith("assert(")) {
        KDB facts = t.translate(statement.substring(7, statement.length()-2)); // Remove assert()
        kb.add(facts);
        return true;
      } else if (statement.startsWith("asserta(")) {
        KDB facts = t.translate(statement.substring(9, statement.length()-2)); // Remove asserta()
        kb.addBefore(facts);
        return true;
      } else if (statement.startsWith("assertz(")) {
        KDB facts = t.translate(statement.substring(8, statement.length()-2)); // Remove assertz()
        kb.add(facts);
        return true;
      }

      /**
       * Retract = remove fact from the knowledge database
       */
      else if (statement.startsWith("retract(")) {
        KDB facts = t.translate(statement.substring(8, statement.length()-2)); // Remove retract()
        kb.remove(facts);
        return true;
      } else if (statement.startsWith("retractall(")) {
        KDB facts = t.translate(statement.substring(11, statement.length()-2));  // Remove retractall()
        kb.remove(facts);
        return true;
      }

      /**
       * Query = use DCEC to prove a conjecture
       */
      else {
        KDB conjecture = t.translate(statement.substring(0, statement.length()-1));
        if(conjecture.axioms.size() == 1) {
          kb.add(conjecture.prototypes);
          return DCEC_WebInterface.solveQuery(kb.prototypesToString(), kb.axiomsToString(),
              conjecture.axioms.iterator().next());
        } else {
          log.error("Cannot handle multiple statements in conjecture.");
          return false;
        }
      }
    } else {
      log.error("No end for statement: " + statement);
      return false;
    }
  }


  public List<Map<Variable, Symbol>> solveBindings(String statement) {
    if(solve(statement)) {
      // Return list with empty map --> found a solution, but no bindings available.
      return Collections.singletonList(new HashMap<>());
    }
    return new ArrayList<>(); // Return empty list --> no solution found.
  }

    @Override
    public Boolean querySupport(Term query) {
    throw new UnsupportedOperationException("Method not implemented.");
    }

  @Override
  synchronized public Pair<Boolean, Map<Term, Boolean>> querySupportWithExplanation(Term query) {
    throw new UnsupportedOperationException("Method not implemented.");
  }

  @Override
  public List<Map<Variable, Symbol>> queryBelief(Term query) {
    throw new UnsupportedOperationException("Method not implemented.");
  }

  @Override
  public void assertBelief(Term belief) {
    throw new UnsupportedOperationException("Method not implemented.");
  }

  @Override
  public void assertBelief(String belief) {
    throw new UnsupportedOperationException("Method not implemented.");
  }

  @Override
  public void retractBelief(Term belief) {
    throw new UnsupportedOperationException("Method not implemented.");
  }

  @Override
  public void assertRule(Term head, List<Term> body) {
    throw new UnsupportedOperationException("Method not implemented.");
  }

  @Override
  public void retractRule(Term head, List<Term> body) {
    throw new UnsupportedOperationException("Method not implemented.");
  }

  @Override
  public void initializeFromFiles(List<String> filenames) throws IOException {
    throw new UnsupportedOperationException("Method not implemented.");
  }

  @Override
  public String getTheory() {
    log.error("DCEC prover doesn't support this functionality. Returning empty theory.");
    return "";
  }

  @Override
  public Object getInternalTheory() {
    return kb;
  }

  @Override
  public boolean setTheory(String theory) {
    log.error("DCEC prover doesn't support this functionality. Ignoring setTheory call.");
    return false;
  }

  @Override
  public void assertRule(String rule) {
    throw new UnsupportedOperationException("Method not implemented.");
  }

  @Override
  public void retractRule(String rule) {
    throw new UnsupportedOperationException("Method not implemented.");
  }

  @Override
  public String sanitize(Term term) {
    return null;
  }

  /**
   * Knowledge database in DCEC syntax
   */
  private class KDB {
    /**
     * Prototypes
     */
    public Set<Prototype> prototypes = new HashSet<>();

    /**
     * Axioms
     */
    public Set<String> axioms = new LinkedHashSet<>();

    /**
     * Add entries to the database. (Added at the end because of LinkedHashSet).
     * @param other entries to add
     */
    void add(KDB other) {
      log.debug("Adding: "  + other.axiomsToString());
      this.prototypes.addAll(other.prototypes);
      this.axioms.addAll(other.axioms);
    }

    /**
     * Add entries at the beginning. Since we're using sets it's a pretty terrible operation.
     * I should consider dropping sets for lists instead and make sure there are no duplicate entries.
     * @param other entries to add
     */
    void addBefore(KDB other) {
      log.debug("Adding before: "  + other.axiomsToString());
      // Convert to list to add at beginning
      List<Prototype> p = new LinkedList<>(prototypes);
      List<String> a = new LinkedList<>(axioms);

      p.addAll(0, other.prototypes);
      a.addAll(0, other.axioms);

      this.prototypes = new HashSet<>(p);
      this.axioms = new LinkedHashSet<>(a);
    }

    /**
     * Add prototypes to the database.
     * @param prototypes prototypes to add
     */
    void add(Set<Prototype> prototypes) {
      this.prototypes.addAll(prototypes);
    }

    /**
     * Remove entries from db
     * @param other entries to remove
     */
    void remove(KDB other) {
      log.debug("Removing: "  + other.axiomsToString());
      // /!\ Do not remove prototypes since there might be other axioms still needing them!
      //this.prototypes.removeAll(other.prototypes);
      this.axioms.removeAll(other.axioms);
    }

    /**
     * Get prototypes as a string, ready for use with DCEC.
     * @return prototypes
     */
    String prototypesToString() {
      String str = "";
      for(Prototype p : prototypes) {
        str = str.concat(p.toString() + "#");
      }
      return str;
    }

    /**
     * Get axioms as a string, ready for use with DCEC.
     * @return axioms
     */
    String axiomsToString() {
      String str = "";
      for(String a : axioms) {
        str = str.concat(a + "#");
      }
      return str;
    }
  }

  /**
   * Prolog to DCEC syntax translator
   */
  private class Prolog2DCEC {
    /**
     * Translates prolog file to DCEC syntax
     * @param file path to file
     * @return Knowledge extracted from file in DCEC syntax, null if error
     */
    public KDB translateFile(String file) {
      try {
        KDB kb = new KDB();
        BufferedReader br = new BufferedReader(new FileReader(file));
        String line;
        while ((line=br.readLine()) != null) {
          kb.add(translate(line));
        }
        return kb;
      } catch (FileNotFoundException e) {
        log.error("File not found.");
        log.error(e.getMessage());
        return null;
      } catch (IOException e) {
        log.error("Could not read line.");
        log.error(e.getMessage());
        return null;
      }
    }

    /**
     * Translates prolog statement to DCEC syntax
     * @param statement Prolog statement to translate
     * @return Knowledge extracted from file in DCEC syntax
     */
    public KDB translate(String statement) {
      final String comment = "#";
      final String implication = ":";

      KDB kb = new KDB();

      // Check if we should translate at all (we ignore commented-out commands)
      if(!statement.isEmpty() && !statement.startsWith(comment)) {
        // Replace "self" with "me" to avoid conflict with DCEC reserved word "self"
        statement = statement.replace("self", "me");
        // Replace "exists" with "existz" to avoid the following error message:
        // "DUE TO A QUIRK OF SPASS OUTPUT PROOFS AND GENERATED STATEMENTS
        // CANNOT BE DERIVED FROM STATEMENTS WITH exists"
        statement = statement.replace("exists", "existz");
        // Remove dash "-" (operator in DCEC)
        statement = statement.replace("-", "");
        // Flag functions (predicates with arguments) with an "_F" after the name, before the parenthesis,
        // to avoid conflicts when using the same name for objects (i.e. without arguments).
        // Example: "obstacle(present)" and "obstacle" will conflict in DCEC.
        // TODO: Currently also replaces built-in DCEC functions not, if, iff.
        // TODO: No adverse side-effects have been found (yet).
        statement = statement.replaceAll("(\\w+)\\(","$1_F\\(");

        // Add function prototypes to database
        kb.prototypes.addAll(getPrototypes(statement));

        // Extract implications
        if (statement.contains(implication)) {
          String implies[] = statement.split(implication);
          if (implies.length == 2) {
            kb.axioms.add(buildForAll("implies(" + buildAnd(implies[1]) + "," + implies[0] + ")")); // Add implies(A,B) statement
          } else {
            log.error("Found implication operator ("+ implication +") but was not able to find operands.");
          }
        }
        // Facts
        else {
          kb.axioms.add(buildForAll(statement));  // Add fact to list
        }
      }
      return kb;
    }

    /**
     * Extract function prototypes from prolog statements.
     * Assumes Boolean return type and Object arguments.
     * @param statement Prolog statement
     * @return Set of prototypes
     */
    private Set<Prototype> getPrototypes(String statement) {
      Pattern r = Pattern.compile("\\W?([\\w_]+_F)\\("); // Looks for some_string_F(
      Matcher m = r.matcher(statement);
      Set<Prototype> prototypes = new HashSet<>();
      while(m.find()) {
        int level = 0;
        int args = 1;
        boolean done = false;
        for(char c: statement.substring(m.end(1)).toCharArray()) {
          switch(c) {
            case '(':
              level++;
              break;
            case ')':
              level--;
              if(level < 1) done = true;
              break;
            case ',':
              if(level==1) args++;
              break;
          }
          if(done) break;
        }
        prototypes.add(new Prototype("Boolean", m.group(1), args));
      }
      return prototypes;
    }

    /**
     * Builds forAll[..] from free variables in prolog statement.
     * @param statement Prolog statement
     * @return command with free variables replaced by forAll[..] statement
     */
    private String buildForAll(String statement) {
      Pattern r = Pattern.compile("\\W([A-Z])\\W"); // Looks for X
      Matcher m = r.matcher(statement);
      String forAll = "";
      ArrayList<String> freeVariables = new ArrayList<>();
      if(m.find()) {
        do {
          if(!freeVariables.contains(m.group(1))) {
            forAll = forAll.concat("forAll[" + m.group(1).toLowerCase() + "] ");
            freeVariables.add(m.group(1));
          }
          statement = statement.replace(m.group(0), m.group(0).toLowerCase());
        } while(m.find(m.end()-1)); // Little trick to find overlapping matches, e.g. (A,B) -> find both A and B
      }
      return forAll.concat(statement);
    }

    /**
     * Builds and(x,y) operator from comma separated predicates.
     * @param statement Prolog statement
     * @return command with comma separated predicates converted to and(..,..)
     */
    private String buildAnd(String statement) {
      Pattern r = Pattern.compile("(\\A|\\p{Punct})(\\()\\w"); // Looks for  "(", "..((" and "...,("
      Matcher m = r.matcher(statement);
      while(m.find()) {
        int level = 0;
        int andCounter=0;
        int replacedChars = 0;
        String and = "and(";
        boolean done = false;
        for(char c: statement.substring(m.end(2)).toCharArray()) {
          switch(c) {
            case '(':
              level++;
              and += c;
              break;
            case ')':
              level--;
              and += c;
              if(level < 0) done = true;
              break;
            case ',':
              if(level == 0) {
                andCounter++;
                if (andCounter == 2) {
                  and = "and(" + and + ")";
                  andCounter = 1;
                }
              }
            default:
              and +=c;
              break;
          }
          replacedChars ++;
          if(done) {
            if(andCounter > 0) {
              statement = statement.substring(0, m.end(2) - 1) + and +
                  statement.substring(m.end(2) + replacedChars);
              m = r.matcher(statement);
            }
            break;
          }
        }
      }
      return statement;
    }
  }

  /**
   * DCEC prototype
   */
  private class Prototype {
    private String type;
    private String name;
    private int arguments;

    /**
     * Prototype constructor
     * @param t Return type
     * @param n Name
     * @param a Number of arguments
     */
    public Prototype(String t, String n, int a) {
      type = t;
      name = n;
      arguments = a;
    }

    /**
     * Returns prototype as string, ready to be used with DCEC.
     * @return prototype in string form
     */
    public String toString() {
      String str = type + " " + name;
      for(int i=0; i<arguments; i++) {
        str = str.concat(" Object");
      }
      return str;
    }

    @Override
    public boolean equals(Object obj) {
      if(obj == null || obj.getClass() != Prototype.class) {
        return false;
      }
      final Prototype that = (Prototype) obj;
      return (this.type.equals(that.type)
          && this.name.equals(that.name)
          && this.arguments == that.arguments);
    }

    @Override
    public int hashCode() {
      return Long.valueOf((type.hashCode()*31 + name.hashCode())*31 + this.arguments).hashCode();
    }
  }

  /**
   * Interface to the online version of the Talos/DCEC logic prover
   */
  private static class DCEC_WebInterface {
    private static final String POST_URL = "https://prover.cogsci.rpi.edu/DCEC_PROVER/proverPost.php";
    private static final String API_KEY = "x60aw2on76";

    /**
     * Solve query through online Talos/DCEC logic prover
     * @param prototypes Prototypes, separated by #
     * @param axioms Axioms, separated by #
     * @param conjecture Conjecture (single statement)
     * @return True if proof was found, false if not or errors happened.
     */
    private static boolean solveQuery(String prototypes, String axioms, String conjecture) {
      boolean justify = false;
      int timeout = 60;
      String options = "";
      boolean simultaneous = false;
      boolean discover = false;
      // Manual rule selection to avoid timeout when proof can't be found
      String manual =
          "[DCEC_RULE_1,DCEC_RULE_2,DCEC_RULE_3,DCEC_RULE_4,DCEC_RULE_5,DCEC_RULE_6,DCEC_RULE_7,DCEC_RULE_9," +
              "DCEC_RULE_10,DCEC_RULE_11A,DCEC_RULE_11B,DCEC_RULE_12,DCEC_RULE_13,DCEC_RULE_14,DCEC_RULE_15," +
              "MODUS_PONENS,CONJUNCTION_INTRODUCTION,WEAKENING,COMMUTIVITY_OF_OR,COMMUTIVITY_OF_XOR," +
              "DEFINITION_OF_XOR,CUT_ELIMINATION,DISJUCTION_ELIMINATION,DEFINITION_OF_IFF]";
      return solveQuery(prototypes, axioms, conjecture, justify, timeout, options, simultaneous, discover, manual);
    }

    /**
     * Solve query through online Talos/DCEC logic prover.
     * @param prototypes Prototypes, separated by #
     * @param axioms Axioms, separated by #
     * @param conjecture Conjecture (single statement)
     * @param justify Whether the prover should return the list of rules and statements that were used in its proof.
     * @param timeout The amount of time the Prover can run, defaults to the maximum time associated with your API Key.
     * @param options Any options you would like to hand to SPASS, defaults to -Auto.
     * @param simultaneous Whether the prover should run in simultaneous mode, where all DCEC rules require that times be the same, or normal mode. Defaults to false, i.d. normal.
     * @param discover Whether the prover should return new statements it finds while working on the proof.
     * @param manual Set to "-" to include all DCEC*, basic logical rules, and their Commonly Known equivalents, otherwise a list of rules in a string forming a bracketed list.
     * @return True if proof was found, false if not or errors happened.
     */
    private static boolean solveQuery(String prototypes, String axioms, String conjecture,
                                      boolean justify, int timeout, String options,
                                      boolean simultaneous, boolean discover, String manual) {
      long startTime = System.currentTimeMillis();
      HttpsURLConnection con;
      try {
        URL obj = new URL(POST_URL);
        con = (HttpsURLConnection) obj.openConnection();
        con.setRequestMethod("POST");
      } catch (MalformedURLException e) {
        log.error("Malformed url!");
        log.error(e.getMessage());
        return false;
      } catch (IOException e) {
        log.error("IOException while setting up URL connection.");
        log.error(e.getMessage());
        return false;
      }

      // Build POST request
      Map<String, String> request = new HashMap<>();
      request.put("apiKey", API_KEY);
      request.put("prototypes", prototypes);
      request.put("axioms", axioms);
      request.put("conjecture", conjecture);
      request.put("justify", (justify ? "true" : "false"));
      request.put("timeout", Integer.toString(timeout));
      request.put("options", options);
      request.put("simultaneous", (simultaneous ? "true" : "false"));
      request.put("discover", (discover ? "true" : "false"));
      request.put("manual", manual);

      StringBuilder sb = new StringBuilder();
      try {
        for (Map.Entry<String, String> entry : request.entrySet())
          sb.append(URLEncoder.encode(entry.getKey(), "UTF-8") + "=" + URLEncoder.encode(entry.getValue(), "UTF-8") + "&");
      } catch (UnsupportedEncodingException e) {
        log.error("Error while encoding data.");
        log.error(e.getMessage());
        return false;
      }

      // Remove last &
      sb.deleteCharAt(sb.lastIndexOf("&"));

      // Send POST request
      try {
        con.setDoOutput(true);
        DataOutputStream wr = new DataOutputStream(con.getOutputStream());
        wr.writeBytes(sb.toString());
        wr.flush();
        wr.close();
      } catch (IOException e) {
        log.error("IOException while sending POST request.");
        log.error(e.getMessage());
        return false;
      }

      // Get response
      StringBuffer response;
      try {
        BufferedReader in = new BufferedReader(
            new InputStreamReader(con.getInputStream()));
        String inputLine;
        response = new StringBuffer();

        while ((inputLine = in.readLine()) != null) {
          response.append(inputLine);
        }
        in.close();
      } catch (IOException e) {
        log.error("IOException while reading response.");
        log.error(e.getMessage());
        return false;
      }
      
      boolean proof = response.toString().contains("Proof Found");
      boolean noproof = response.toString().contains("Completion Found");

      if(!proof && !noproof) {
        log.error("DCEC returned an error!");
        Pattern r = Pattern.compile("\\Q<body>\\E(.+)\\Q</body>\\E");
        Matcher m = r.matcher(response.toString());
        if(m.find()) {
          String message[] = m.group(1).split("\\Q<br>\\E");
          for(String msg : message) {
            System.out.println("\t"+msg);
          }
        }
        System.out.println("Prototypes:");
        String[] protos = prototypes.split("#");
        for(String p : protos) {
          System.out.println(p);
        }
        System.out.println("Axioms:");
        String[] ax = axioms.split("#");
        for(String a : ax) {
          System.out.println(a);
        }
      }

      log.info("Solved query: " + conjecture + " is " + proof + ". " +
          "Time to solve: " + (System.currentTimeMillis() - startTime) + "ms");
      return proof;
    }
  }

  public List<Map<Variable, Symbol>> queryString(String query) {
    return  null;
  }


  public Boolean queryBelief(String query){
    return null;
  }


  public void addBelief(String belief) {

  }

  public void retractBelief(String belief) {

  }
}

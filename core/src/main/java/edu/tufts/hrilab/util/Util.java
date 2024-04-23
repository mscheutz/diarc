/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.util;

import java.util.*;
import java.util.Map.Entry;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Util {

  private static Logger log = LoggerFactory.getLogger(Util.class);

  // Global heading from (angle between) one point to another
  public static double getHeadingFrom(double x1, double y1, double x2, double y2) {
    double h = Math.atan2((y2 - y1), (x2 - x1));
    return h;
  }

  // Heading to point relative to given heading
  public static double getHeadingFromRel(double x1, double y1, double x2, double y2, double h) {
    double xtmp = x2 - x1;
    double ytmp = y2 - y1;
    // First rotate the destination so heading is 0
    double xdif = Math.cos(h) * xtmp - Math.sin(h) * ytmp;
    double ydif = Math.cos(h) * ytmp + Math.sin(h) * xtmp;
    // Then calculate the angle -- negated because of Y-axis switch
    h = -Math.atan2(ydif, xdif);
    return h;
  }

  // Distance between two points
  public static double getDistanceFrom(double x1, double y1, double x2, double y2) {
    double d;

    d = Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    return d;
  }

  /**
   * Longitude-latitude distance. This is needed for pos servers that return the
   * pose in longitude-latitude form instead of x,y meters (e.g., Cart). See
   * http://en.wikipedia.org/wiki/Great-circle_distance.
   *
   * @param x1 starting longitude
   * @param y1 starting latitude
   * @param x2 ending longitude
   * @param y2 ending latitude
   * @return distance between the two locations
   */
  public static double lldist(double x1, double y1, double x2, double y2) {
    double R = 6372795; // Average radius of earth
    double xcurr = Math.toRadians(x1);
    double xdest = Math.toRadians(x2);
    double ycurr = Math.toRadians(y1);
    double ydest = Math.toRadians(y2);
    double dx = xdest - xcurr;
    double dy = ydest - ycurr;
    double dist;

    dist = Math.pow(Math.sin(dy / 2), 2) + Math.cos(ycurr) * Math.cos(ydest) * Math.pow(Math.sin(dx / 2), 2);
    //dist = 2 * Math.atan2(Math.sqrt(dist), Math.sqrt(1 - dist));
    dist = 2 * Math.asin(Math.min(1, Math.sqrt(dist)));
    dist = R * dist;
    return dist;
  }

  /**
   * Sleep. This is a convenience method to obviate the need to catch the
   * InterruptedException any time you want to sleep.
   *
   * @param millis the sleep duration
   */
  public static void Sleep(long millis) {
    if (millis < 0) {
      return;
    }
    try {
      Thread.sleep(millis);
    } catch (InterruptedException ie) {
      log.info("Sleep interrupted!", ie);
    }
  }

  public static int calcParenBalance(String str, int bal) {
    int opar = str.indexOf('(');
    int cpar = str.indexOf(')');
    //System.err.println("calcParenBalance('" + str + "', " + bal + ");");
    if (opar < 0 && cpar < 0) {
      return bal;
    } else if (opar < 0 && cpar >= 0) {
      return calcParenBalance(str.substring(cpar + 1), bal - 1);
    } else if (opar >= 0 && cpar < 0) {
      return calcParenBalance(str.substring(opar + 1), bal + 1);
    } else if (cpar > opar) {
      return calcParenBalance(str.substring(opar + 1), bal + 1);
    } else {
      return calcParenBalance(str.substring(cpar + 1), bal - 1);
    }
  }

  public static int calcBracketBalance(String str, int bal) {
    int opar = str.indexOf('{');
    int cpar = str.indexOf('}');
    //System.err.println("calcBracketBalance('" + str + "', " + bal + ");");
    if (opar < 0 && cpar < 0) {
      return bal;
    } else if (opar < 0 && cpar >= 0) {
      return calcBracketBalance(str.substring(cpar + 1), bal - 1);
    } else if (opar >= 0 && cpar < 0) {
      return calcBracketBalance(str.substring(opar + 1), bal + 1);
    } else if (cpar > opar) {
      return calcBracketBalance(str.substring(opar + 1), bal + 1);
    } else {
      return calcBracketBalance(str.substring(cpar + 1), bal - 1);
    }
  }

  public static int calcQuoteBalance(String str, int bal) {
    //Call with bal=0
    //Returns the number of single quotes
    int qpar = str.indexOf('\'');
    //System.err.println("calcQuoteBalance('" + str + "', " + bal + ");");
    if (qpar < 0) {
      return bal;
    } else {
      return calcQuoteBalance(str.substring(qpar + 1), bal + 1);
    }
  }

  /**
   * Brad: trying out this implementation vs previous one to see what happens
   * @param s string to be tokenized
   * @param delim delimiter
   * @return tokens
   */
  public static List<String> tokenizeArgs(String s, char delim) {
//    return tokenizeArgsHelper(s, delim, true);
    return tokenizeArgsHelper(s, new ArrayList<String>(), 0, 0, 0, 0, true);
  }

  //TODO: currently only set up for , as the delim character
  //TODO: For arbitrary strings, it is impossible to differentiate between certain cases.
  //  e.g. "te","st" -> 1. "te","st" or 1. "te" 2. "st"
  public static List<String> tokenizeArgsHelper(String s, List<String> results, int parenDepth, int singleQuoteDepth, int bracketDepth, int doubleQuotesOpen, boolean first) {
    if (s.isEmpty()) {
      //DO we need to check doubleQuotesOpen here as well?
      return (doubleQuotesOpen == 0 && parenDepth == 0 && singleQuoteDepth % 2 == 0 && bracketDepth == 0) ? results : null;
    }
    int resultIdx;

    //Continuation of potential escaped string, remain on same token index
    if (doubleQuotesOpen > 0) {
      resultIdx = results.size()-1;
    }
    //New token
    else {
      doubleQuotesOpen = s.startsWith("\"") ? 1 : 0;
      resultIdx = results.size();
      results.add("");
    }

    //TokenizeArgs while handling for special characters, including the potential for nested " ) and , chars
    // which are parts of arbitrary string arguments
    //Assumption: well formed args - first '"' should only ever be the first character or directly after a ',' (start of escaped string arg)
    //            Otherwise is internal to some other arg and should not be considered a candidate for splitting
    char c;
    char cPrev = ' ';
    for (int i=0;i<s.length();i++) {
      c = s.charAt(i);
      //Original logic: balance paren,bracket,singleQuote depth while splitting on delim
      if (doubleQuotesOpen == 0) {
        if (c == '(') parenDepth++;
        else if (c == ')') parenDepth--;
        else if (c == '\'') singleQuoteDepth++;
        else if (c == '{') bracketDepth++;
        else if (c == '}') bracketDepth--;
        else if (c == ',') {
          if (results.get(resultIdx).length() > 0) {
            //valid token split w/o special characters
            if (parenDepth == 0 && singleQuoteDepth % 2 == 0 && bracketDepth == 0) {
              resultIdx++;
              results.add("");
              cPrev = c;
              continue;
            }
          } else {
            //successive delims indicates error
            return null;
          }
        } else if (c == '"') {
          //This may not necessarily mean the end/beginning of an argument, but definitely indicates start of an escaped string
          doubleQuotesOpen = 1;
        }
      }
      //Currently within an arbitrary string
      else {
        //Upon encountering potential end of arbitrary string, need to test if so
        //  Possible end when " is followed by , or )
        //Arbitrary string can be its own token, or just internal
        //   if own token, want to increment token and recursive call
        //   otherwise, need to test whether it's actually the end of the arbitrary string or not
        if (cPrev == '"' && (c == ',' || c == ')') && doubleQuotesOpen == 2) {
          //potentiall its own token, want to see if splitting here is valid
          if (parenDepth == 0 && singleQuoteDepth % 2 == 0 && bracketDepth == 0) {
            List<String> tokenizeCandidate;
            ArrayList<String> resultsCopy = new ArrayList<>(results);
            if (c == ')') {
              resultsCopy.set(resultIdx, resultsCopy.get(resultIdx) + c);
              tokenizeCandidate = tokenizeArgsHelper(s.substring(i + 1), resultsCopy, parenDepth - 1, singleQuoteDepth, bracketDepth, 0, false);
            } else {
              tokenizeCandidate = tokenizeArgsHelper(s.substring(i + 1), resultsCopy, parenDepth, singleQuoteDepth, bracketDepth, 0, false);
            }
            //if splitting here is valid, return result of recursive call, otherwise continue without splitting
            if (tokenizeCandidate != null) {
              return tokenizeCandidate;
            }
          }
          //internal arbitrary string, just want to not mistakenly count ) " ' , etc
          else {
            List<String> tokenizeCandidate;
            ArrayList<String> resultsCopy = new ArrayList<>(results);
            resultsCopy.set(resultIdx, resultsCopy.get(resultIdx) + c);
            //Test whether extending arbitrary string returns a valid result, otherwise consider this the end of the escaped string
            tokenizeCandidate = tokenizeArgsHelper(s.substring(i + 1), resultsCopy, parenDepth, singleQuoteDepth, bracketDepth, doubleQuotesOpen, false);
            if (tokenizeCandidate != null) {
              return tokenizeCandidate;
            } else {
              doubleQuotesOpen = 0;
              if (c == ')') {
                parenDepth--;
              }
            }
          }
        }
        //Prevent " followed by , or ) from being split as its own token
        if (doubleQuotesOpen == 1) {
          doubleQuotesOpen = 2;
        }
      }
      results.set(resultIdx, results.get(resultIdx) + c);
      cPrev = c;
    }

    //If string cannot be properly split into tokens return null to indicate parent call should continue and not split
    // on escaped delim. Otherwise return valid results.
    if ((doubleQuotesOpen > 0 && !(results.get(resultIdx).startsWith("\"") && results.get(resultIdx).endsWith("\""))) ||
         !(parenDepth == 0 && singleQuoteDepth % 2 == 0 && bracketDepth == 0)) {
      return first ? results : null;
    } else {
      return results;
    }
  }

  /**
   * Overloaded version of tokenizeArgs, using default ',' delimiter
   * @param s string to be tokenized
   * @return tokens
   */

  public static List<String> tokenizeArgs(String s) {
    List<String> tokens = tokenizeArgs(s,',');
    tokens=tokens.stream().map(String::trim).collect(Collectors.toList());
    return tokens;
  }

  /**
   * Checks if value's toString is an integer, long, float, or double.
   *
   * @param value
   * @return
   */
  public static boolean isNumeric(Object value) {
    return value.toString().matches("[-+]?\\d*\\.?\\d+[fFlLdD]?");
  }

  //TODO:brad:compare this with how things work in the parser now. and/or delete it if it isn't used anywhere
  /**
   * Converts text numbers into digits.
   *
   * @author mheilman
   */
  public static String numbersTextToDigits(String in) {
    final String[] numberStrings = {"zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten",
            "eleven", "twelve", "thirteen", "fourteen", "fifteen", "sixteen", "seventeen", "eighteen", "nineteen"};
    final String[] tensStrings = {"zero", "ten", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"};
    String result = in;
    int idx1, idx2;
    int i, j, k;
    String token;
    boolean test = false;
    int tmpidx = 0;
    int number;
    int tmpnumber;
    idx1 = -1;
    boolean numberstart;
    idx2 = -1;
    boolean firstloop = true;
    //System.out.println("numbersTextToDigits: in == " + in);
    while (tmpidx != -1) {
      number = 0;
      k = 0;
      firstloop = true;
      tmpnumber = 0;
      numberstart = true;
      while ((tmpidx = k) != -1 || firstloop) {
        k = result.indexOf(" ", tmpidx + 1);
        if (firstloop && k != -1) {
          token = result.substring(tmpidx, k);
          tmpidx--;
          firstloop = false;
        } else if (firstloop && k == -1) {
          token = result.substring(tmpidx);
          tmpidx--;
          firstloop = false;
        } else if (k != -1) {
          token = result.substring(tmpidx + 1, k);
        } else {
          token = result.substring(tmpidx + 1);
        }
        //System.out.println("token == " + token);
        test = false;
        if (numberstart) {
          for (j = 0; j < numberStrings.length; j++) {
            if (numberStrings[j].compareTo(token) == 0) {
              numberstart = false;
              tmpnumber = j;
              idx1 = tmpidx + 1;
              break;
            }
          }
          for (j = 0; j < tensStrings.length; j++) {
            if (tensStrings[j].compareTo(token) == 0) {
              tmpnumber = j * 10;
              numberstart = false;
              idx1 = tmpidx + 1;
              break;
            }
          }
          if (token.compareTo("hundred") == 0) {
            numberstart = false;
            idx1 = tmpidx + 1;
            tmpnumber = 100;
          } else if (token.compareTo("thousand") == 0) {
            numberstart = false;
            idx1 = tmpidx + 1;
            tmpnumber = 1000;
          } else if (token.compareTo("million") == 0) {
            numberstart = false;
            idx1 = tmpidx + 1;
            tmpnumber = 1000000;
          } else if (token.compareTo("billion") == 0) {
            numberstart = false;
            idx1 = tmpidx + 1;
            tmpnumber = 1000000000;
          }
        } else {
          idx2 = tmpidx;
          for (j = 0; j < numberStrings.length; j++) {
            if (numberStrings[j].compareTo(token) == 0) {
              if (tmpnumber > 999) {
                number += tmpnumber;
                tmpnumber = 0;
              }
              test = true;
              tmpnumber += j;
            }
          }
          for (j = 0; j < tensStrings.length; j++) {
            if (tensStrings[j].compareTo(token) == 0) {
              if (tmpnumber > 999) {
                number += tmpnumber;
                tmpnumber = 0;
              }
              test = true;
              tmpnumber += j * 10;
            }
          }
          if (token.compareTo("hundred") == 0) {
            test = true;
            tmpnumber *= 100;
          } else if (token.compareTo("thousand") == 0) {
            test = true;
            tmpnumber *= 1000;
          } else if (token.compareTo("million") == 0) {
            test = true;
            tmpnumber *= 1000000;
          } else if (token.compareTo("billion") == 0) {
            test = true;
            tmpnumber *= 1000000000;
          } else if (token.compareTo("and") == 0) {
            test = true;
            number += tmpnumber;
            tmpnumber = 0;
          }
          if (!test) {
            number += tmpnumber;
            result = result.substring(0, idx1) + number + result.substring(idx2);
            break;
          }
        }

      }
      if (test) {
        //number ended the string
        number += tmpnumber;
        result = result.substring(0, idx1) + number;
      }

    }
    return result;
  }

  /**
   * Check whether next arg is another parameter (i.e., starts with '-').
   *
   * @return true if it's a parameter
   */
  public static boolean checkNextArg(String[] args, int i) {
    int j = i + 1;
    return ((j >= args.length)
            || (args[j].length() == 0)
            || (args[j].charAt(0) == '-'));
  }

  public static int signedInt(byte b1, byte b2) {
    int sint = (int) b1 << 8 | (int) b2;
    return sint;
  }

  public static int unsignedInt(byte b1, byte b2) {
    int uint = ((0xFF & (int) b1) << 8
            | (0xFF & (int) b2)) & 0xFFFFFFFF;
    return uint;
  }

  public static int unsignedInt(byte b1, byte b2, byte b3) {
    int uint = ((0xFF & (int) b1) << 16
            | (0xFF & (int) b2) << 8
            | (0xFF & (int) b3)) & 0xFFFFFFFF;
    return uint;
  }

  public static long unsignedInt(byte b1, byte b2, byte b3, byte b4) {
    long uint = ((long) ((0xFF & (int) b1) << 24
            | (0xFF & (int) b2) << 16
            | (0xFF & (int) b3) << 8
            | (0xFF & (int) b4))) & 0xFFFFFFFFL;
    return uint;
  }

  /**
   * method for joining objects (more properly, their .toString()) with a
   * specific deliminator to produce a string of the form "a, b, c". based on
   * http://snippets.dzone.com/posts/show/91
   */
  public static String join(Iterable<? extends Object> objectCollection, String separator) {
    Iterator<? extends Object> iter = objectCollection.iterator();
    if (!iter.hasNext()) {
      return "";
    }
    StringBuilder out = new StringBuilder(String.valueOf(iter.next()));
    while (iter.hasNext()) {
      out.append(separator).append(iter.next());
    }
    return out.toString();
  }

  /**
   * searches through a collection (ArrayList, HashSet, etc) and determines if
   * the collection contains a particular searchFor string, IGNORING case.
   */
  public static boolean containsIgnoreCase(Collection<String> searchIn, String searchFor) {
    for (String eachElement : searchIn) {
      if (eachElement.equalsIgnoreCase(searchFor)) {
        return true;
      }
    }
    // if haven't already quit with true
    return false;
  }

  /**
   * searches through a {@code Map<String, Object>} (can be a hashmap, treemap, etc) and
   * returns the value associated with the key, IGNORING case. If not key
   * matched, returns null.
   */
  public static <T> T getIgnoreCase(Set<Entry<String, T>> entrySet, String searchForKey) {
    for (Entry<String, T> eachEntry : entrySet) {
      if (eachEntry.getKey().equalsIgnoreCase(searchForKey)) {
        return eachEntry.getValue();
      }
    }
    // if found nothing:
    return null;
  }
}

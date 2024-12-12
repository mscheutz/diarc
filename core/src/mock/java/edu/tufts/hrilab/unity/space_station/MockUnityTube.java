/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity.space_station;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;

import java.util.HashMap;
import java.util.Map;

public class MockUnityTube {
  private String name;
  private String simName;
  private boolean isDamaged = false;
  private boolean isBroken = false;
  private boolean isRepaired = false;
  private boolean isOn = true;
  private float health = 100;
  private float decayWhileOn;
  private float decayWhileOff;

  private static Map<String, String> wordToDigitNumbers = new HashMap<>();

  static {
    wordToDigitNumbers.put("one", "1");
    wordToDigitNumbers.put("two", "2");
    wordToDigitNumbers.put("three", "3");
    wordToDigitNumbers.put("four", "4");
    wordToDigitNumbers.put("five", "5");
    wordToDigitNumbers.put("six", "6");
    wordToDigitNumbers.put("seven", "7");
    wordToDigitNumbers.put("eight", "8");
    wordToDigitNumbers.put("nine", "9");
    wordToDigitNumbers.put("ten", "10");
    wordToDigitNumbers.put("eleven", "11");
    wordToDigitNumbers.put("twelve", "12");
  }

  private String area;
  private String side;
  private String number;

  public MockUnityTube(String area, String side, String number, float decayWhileOn, float decayWhileOff) {
    this.name = area + side + number;
    this.area = area;
    this.side = side;
    this.number = number;
    this.simName = area.substring(0, 1).toUpperCase() + area.substring(1) + ":" + side.substring(0, 1).toUpperCase() + ":" + wordToDigitNumbers.get(number.toLowerCase());
    this.decayWhileOn = decayWhileOn;
    this.decayWhileOff = decayWhileOff;
  }

  public void updateStep() {
    if (isDamaged && !isBroken) {
      if (health >= 0) {
        if (isOn) {
          health -= decayWhileOn;
        } else {
          health -= decayWhileOff;
        }

        if (health <= 0) {
          isBroken = true;
        }
      }
    }
  }


  public void startTubeDamage() {
    isDamaged = true;
  }

  public float getHealth() {
    return health;
  }

  public boolean isDamaged() {
    return isDamaged;
  }

  public boolean isBroken() {
    return isBroken;
  }

  public boolean repairTube() {
    if (isBroken || isOn || !isDamaged) {
      return false;
    }

    isDamaged = false;
    health = 100;
    return true;
  }

  public boolean turnOff() {
    if (isBroken) {
      return false;
    }

    isOn = false;
    return true;
  }

  public boolean turnOn() {
    if (isBroken) {
      return false;
    }

    isOn = true;
    return true;
  }

  public String getSimName() {
    return simName;
  }

  public String getName() {
    return name;
  }

  public Symbol getNameAsSymbol() {
    return Factory.createSymbol(name);
  }

  public String getArea() {
    return area;
  }

  public boolean inArea(Symbol area) {
    return this.area.equals(area.getName());
  }

  public boolean isOn() {
    return isOn;
  }

  public boolean isOff() {
    return !isOn;
  }
}

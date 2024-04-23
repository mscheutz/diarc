/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity;

import java.util.HashMap;

public class MockUnityTube {
    private String name;
    private String simName;
    private boolean isBroken;
    private boolean isRepaired;
    private boolean isOn = true;//todo: don't know how this is supposed to be modeled for toggling. assuming always on for now
    private double health;
    private double decayOn;
    private double decayOff;

    private static HashMap<String, String> wordToDigitNumbers = new HashMap<>();
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

    private String wing;
    private String side;
    private String number;

    public MockUnityTube(String wing, String side, String number, boolean isBroken, double health, double decayOn, double decayOff) {
        this.name = wing + side + number;
        this.simName = wing.substring(0,1).toUpperCase() + wing.substring(1) + ":" + side.substring(0,1).toUpperCase() + ":" + wordToDigitNumbers.get(number.toLowerCase());
        this.isBroken = isBroken;
        this.isRepaired = false;
        this.wing = wing;
    }

    public MockUnityTube(String wing, String side, String number, boolean isBroken, double decayOn, double decayOff) {
        this.name = wing + side + number;
        this.simName = wing.substring(0,1).toUpperCase() + wing.substring(1) + ":" + side.substring(0,1).toUpperCase() + ":" + wordToDigitNumbers.get(number.toLowerCase());
        this.isBroken = isBroken;
        this.health = 100;
        this.isRepaired = false;
        this.decayOn = decayOn;
        this.decayOff = decayOff;
        this.wing = wing;
    }

    public void updateStep() {
        if (isBroken) {
            if (health >= 0) {
                if (isOn) {
                    health -= decayOn;
                } else {
                    health -= decayOff;
                }
            }
        } else if (isRepaired && health >= 100) {
            isRepaired = false;
        }
    }


    public void breakTube() {
        isBroken = true;
    }

    public double getHealth() {
        return health;
    }

    public boolean getIsBroken() {
        return isBroken;
    }

    public void repairTube() {
        isBroken = false;
        isRepaired = true;
    }

    public String getSimName() { return simName; };

    public String getName() {
        return name;
    }

    public String getWing() { return wing; }
}

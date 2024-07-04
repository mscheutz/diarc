/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.temi;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.temiv3.TemiV3Interface;
import edu.tufts.hrilab.temiv3.TemiV3PoseConsultant;
import edu.tufts.hrilab.temiv3.TemiV3PoseReference;
import edu.tufts.hrilab.util.Util;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.*;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.regex.Matcher;
import java.util.regex.Pattern;


public class MockTemiV3Component extends DiarcComponent implements TemiV3Interface {

    private final static Pattern alphaNumericPattern = Pattern.compile("\\w+");

    String currentRoom;
    private TemiV3PoseConsultant consultant;
    private Lock consultantLock = new ReentrantLock();
    private long escortWaitDuration;
    private int escortWaitAttempts;
    private boolean sayTextBlocks;
    private final Lock freezeLock = new ReentrantLock();
    private final Condition freezeCondition = freezeLock.newCondition();

    //TODO: this kbname/type should likely correspond to name of map it is associated with
    private String mapDescriptor = "location";

    public MockTemiV3Component() {
        super();
        shouldRunExecutionLoop = true;
        executionLoopCycleTime = 10000;
        escortWaitDuration = 5;
        escortWaitAttempts = 3;
    }


    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = new ArrayList<>();
        options.add(Option.builder("mapDescriptor").hasArg().argName("file").desc("map name for location types and consultant kbName").build());
        options.addAll(super.additionalUsageInfo());
        return options;
    }

    @Override
    protected void parseArgs(CommandLine cmdLine) {
        if (cmdLine.hasOption("mapDescriptor")) {
            mapDescriptor = cmdLine.getOptionValue("mapDescriptor");
        }
    }

    @Override
    protected void init() {
        super.init();

        consultant = new TemiV3PoseConsultant(TemiV3PoseReference.class, mapDescriptor, new ArrayList<>());
        try {
            Collection<String> groups = this.getMyGroups();
            groups.add(consultant.getKBName());
            TRADE.registerAllServices(consultant,groups);
        } catch (TRADEException e) {
            log.error("[init] exception registering consultant ", e);
        }
    }



//    @Override
//    protected void executionLoop() {
//        log.warn("[executionLoop] in loop");
//    }

    private TemiV3PoseReference getLocationReferenceFromId(Symbol locRef) {
        log.info("[getLocationReferenceFromId] " + locRef.toString());
        if (!locRef.isTerm() && locRef.getName().startsWith(mapDescriptor)) {
            return consultant.getReference(locRef);
        } else {
            log.warn("[getLocationReferenceFromId] Argument is not a refId");
            return null;
        }
    }

    //TODO:brad: this could maybe be put in a utility method somewhere foc convenience sake.

    /**
     * Convenience method for wrapping names that should variably be in ""
     *
     * @param desc
     * @return
     */
    private static String escapeSpecialCharacters(String desc) {
        if (desc == null || desc.isEmpty()) {
            return null;
        }
        Matcher matcher = alphaNumericPattern.matcher(desc);
        if (!matcher.matches()) {
            return "\"" + desc + "\"";
        }
        return desc;
    }

    public TemiV3PoseReference getLocationReferenceFromName(String name) {
        return consultant.getLocationReferenceFromName(name);
    }

    @Override
    public String getLocationNameFromReference(Symbol locRef) {
        return consultant.getReference(locRef).getName().getName();
    }

    @Override
    public boolean goToLoc(Symbol location, Boolean escorting) {
        log.info("[goToLoc] called with location " + location);
        TemiV3PoseReference locRef;
        if (!location.isTerm() && location.getName().startsWith(mapDescriptor)) {
            locRef = getLocationReferenceFromId(location);
        } else {
            locRef = getLocationReferenceFromName(location.getName());
        }

        List<Float> loc = locRef.getPose();
        log.info("[goToLocation] in goTo with location: " + loc);
        log.info("[goToLocation] sleeping a bit to simulate navigation");
        Util.Sleep(3000);
        return true;
    }

    @Override
    public void interruptGoToLocation() {
        endRepositionLoop();
        //TRADE.callThe("answerQuestion", Factory.createSymbol(App.getTemiFirebaseComponent().getRobotName()), Factory.createSymbol(App.getTemiFirebaseComponent().getRobotName()), Factory.createPredicate("waitForAckTemi", "X"));
        try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("answerQuestion")).call(void.class, Factory.createSymbol("self:agent"), Factory.createSymbol("self:agent"), Factory.createPredicate("waitForAckTemi", "X"));
        } catch (TRADEException e) {
            log.error("[interruptGoToLocation] error calling answerQuestion", e);
        }
        stopMoving();
    }

    @Override
    public boolean goToPos(List<Float> goalCoords, Symbol destination, boolean clearScreen) {
        log.info("[goToPos] in method");
        return true;
    }

    @Override
    public void followMe() {
        log.info("[followMe]");
    }


    @OnInterrupt(onCancelServiceCall = "interruptFollowBlocking()", onSuspendServiceCall = "interruptFollowBlocking()")
    @Override
    public void followMeBlocking() {
        followMe();
        startAcknowledgeDisplay("following until interrupted");
        try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("waitForResponse").argTypes(Predicate.class)).call(Object.class, Factory.createPredicate("waitForAckTemi", "X"));
        } catch (TRADEException e) {
            log.error("[followMeBlocking] error blocking with waitForResponse", e);
        }
        stopMoving();
        endAcknowledgeDisplay();
    }

    @Override
    public void interruptFollowBlocking() {
        try {
            //TODO:brad: I don't think these are the query semantics we want, we'll see if we can come up with a way to fix them.
            TRADE.getAvailableService(new TRADEServiceConstraints().name("cancelWaitForResponse").argTypes(Predicate.class)).call(void.class, Factory.createPredicate("waitForAckTemi", "X"));
        } catch (TRADEException e) {
            log.error("[interruptFollowBlocking] error calling cancelWaitForResponse", e);
        }
        stopMoving();
    }

    @Override
    public void stopMoving() {
//        sendPositionUpdate();
    }

    @Override
    public void saveLocation(Symbol location) {
//        //TODO: Can this ever be a reference?
//        String name = getLocationNameFromReference(location);
        String name = location.getName();

        log.info("[saveLocation] " + name);
        if (name.startsWith("\"") && name.endsWith("\"")) {
            name = name.substring(1, name.length() - 1);
        }

//        if (name.contains("/")) {
//            sayTextDialogueHistory(Factory.createSymbol("Location names cannot contain a forward slash, please try again with a different name"), true);
//            return;
//        }

        //Check 2 cases:
        //  1. new location name is homophone variation of existing location name - popup prompt asking to overwrite location
        //  2. new location name is in dict but not the above case - popup prompt saying this word has other meaning, cannot be a location name
        log.debug("[saveLocation] calling checkLocationName in TLDLParserComponent");
        String conflict = null;
        try {
            conflict = TRADE.getAvailableService(new TRADEServiceConstraints().name("checkLocationName")).call(String.class, name);
        } catch (TRADEException e) {
            log.error("[saveLocation] unable to check location name for " + name, e);
        }

        if (conflict != null) {
            if (conflict.equals("")) {
                log.warn("[saveLocation] conflicts location: "+location+" with nonLoc dictionary entry");
//                MainActivity.getInstance().showConflictingLocNameAlert(loc, false);
                return;
            }
            else {
                log.debug("[saveLocation] conflicts with other location entry: " + conflict);
                //TODO:brad: what is supposed to happen here?
//                //Possible that an old location is still in the dictionary but has been deleted as a location
//                // If this is the case, we don't want to prompt the user. The old entries will be removed upon saving
//                if (locations.containsKey(conflict)) {
//                    if (!MainActivity.getInstance().showConflictingLocNameAlert(conflict, true)) {
//                        log.info("[saveLocation] user decided to cancel save");
//                        return;
//                    }
//                    log.info("[saveLocation] user decided to overwrite location");
//                    if (!deleteLocation(new Symbol(conflict), false)) {
//                        sayTextDialogueHistory("I cannot overwrite the existing location because I could not delete the existing location. Please check internet connectivity.", true);
//                        return;
//                    }
//                }
            }
        } else {
            log.debug("[saveLocation] locName not in dict");
        }
        consultantLock.lock();
        consultant.addLocation(location.getName(), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        consultantLock.unlock();

    }

    //TODO: implement this post temi action refactor
    @Override
    public void interruptSaveLocation() {

    }

    //TODO:brad: move this to consultant
    //TODO: will this always be a ref
    @Override
    public void deleteLocation(Symbol location) {
        List<Map<Variable, Symbol>> locBindings;
        try {
            locBindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Term.class)).call(List.class, Factory.createPredicate("at(?actor,X)"));
        } catch (TRADEException e) {
            log.error("Error generating object bindings");
            return;
        }

        String locName = consultant.getReference(location).getName().getName();
        if (locBindings != null && !locBindings.isEmpty()) {
            Symbol atLoc = locBindings.get(0).get(Factory.createVariable("X"));
            if (locName.equals(atLoc.getName())) {
                try {
                    TRADE.getAvailableService(new TRADEServiceConstraints().name("retractBelief").argTypes(Predicate.class)).call(void.class, Factory.createPredicate("at", locBindings.get(0).get(Factory.createVariable("?actor")), locBindings.get(0).get(Factory.createVariable("X"))));
                } catch (TRADEException e) {
                    log.error("Error generating object bindings");
                    return;
                }
            }
        }

        consultantLock.lock();
        consultant.removeReference(location);
        consultantLock.unlock();

        log.debug("[deleteLocation] removing location " + locName + " from parser dict");
        try {
        if (locName.startsWith("\"") && locName.endsWith("\"")) {
            locName = locName.substring(1,locName.length()-1);
        }
            TRADE.getAvailableService(new TRADEServiceConstraints().name("generateLocationRules").argTypes(String.class,String.class,Boolean.class)).call(void.class, locName, consultant.getKBName(), true);
        } catch (TRADEException e) {
            log.error("[deleteLocation] unable to remove dictionary entry for " + location, e);
        }

    }

    //TODO: better way to do handle both cases of refId or string?
    @Override
    public boolean knowsLocation(Symbol location) {
        TemiV3PoseReference locRef;
        if (!location.isTerm() && location.getName().startsWith(mapDescriptor)) {
            return consultant.getActivatedEntities().containsKey(location);
        } else {
            locRef = consultant.getLocationReferenceFromName(location.getName());
            if (locRef == null) {
                return false;
            }
            return consultant.getActivatedEntities().containsKey(locRef.refId);
        }

//        return getLocationReferenceFromId(location) != null;
    }

    @Override
    public List<String> getKnownLocationNames() {
        Set<Symbol> locSymbols = consultant.getActivatedEntities().keySet();
        List<String> knownLocations = new ArrayList<>();
        for (Symbol locSymbol : locSymbols) {
            knownLocations.add(getLocationReferenceFromId(locSymbol).getName().getName());
        }
        return knownLocations;
    }

    @Override
    public void setEscortWaitDuration(Symbol waitDuration) {
        log.info("[setEscortWaitDuration] string: " + waitDuration);
        int numericalValue = convertStringToInt(waitDuration.toString());
        if (numericalValue != -1) {
            setEscortWaitDuration(numericalValue);
        }
    }

    public void setEscortWaitDuration(long waitDuration) {
        log.info("[setEscortWaitDuration] int: " + waitDuration);
        escortWaitDuration = waitDuration;
    }

    @Override
    public void setEscortWaitAttempts(Symbol waitAttempts) {
        log.info("[setEscortWaitAttempts] string: " + waitAttempts);
        int numericalValue = convertStringToInt(waitAttempts.toString());
        if (numericalValue != -1) {
            setEscortWaitAttempts(numericalValue);
        }
    }

    public void setEscortWaitAttempts(int waitAttempts) {
        log.info("[setEscortWaitAttempts] int: " + waitAttempts);
        escortWaitAttempts = waitAttempts;
    }

    @Override
    public long getEscortWaitDuration() {
        return escortWaitDuration;
    }

    @Override
    public int getEscortWaitAttempts() {
        return escortWaitAttempts;
    }


    //Converts the string representation of a number to an integer
    //EW: Can reduce range of this function if we want to (probably won't ever get a number above
    //      100 for any settings)
    private int convertStringToInt(String numberString) {
        boolean isValidInput = true;
        int result = 0;
        int finalResult = 0;
        //Todo: Will: Get rid of this and use only the actions which take in ints
        try {
            return Integer.parseInt(numberString);
        } catch (NumberFormatException e) {
            log.info("Non digit passed to convertStringToInt - converting!");
        }
        List<String> allowedStrings = Arrays.asList
                (
                        "zero", "one", "two", "three", "four", "five", "six", "seven",
                        "eight", "nine", "ten", "eleven", "twelve", "thirteen", "fourteen",
                        "fifteen", "sixteen", "seventeen", "eighteen", "nineteen", "twenty",
                        "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety",
                        "hundred", "thousand", "million", "billion", "trillion"
                );

        if (numberString != null && numberString.length() > 0) {
            numberString = numberString.replaceAll("-", " ");
            numberString = numberString.toLowerCase().replaceAll(" and", " ");
            String[] splittedParts = numberString.trim().split("\\s+");

            for (String str : splittedParts) {
                if (!allowedStrings.contains(str)) {
                    isValidInput = false;
                    System.out.println("Invalid word found : " + str);
                    break;
                }
            }
            if (isValidInput) {
                for (String str : splittedParts) {
                    if (str.equalsIgnoreCase("zero")) {
                        result += 0;
                    } else if (str.equalsIgnoreCase("one")) {
                        result += 1;
                    } else if (str.equalsIgnoreCase("two")) {
                        result += 2;
                    } else if (str.equalsIgnoreCase("three")) {
                        result += 3;
                    } else if (str.equalsIgnoreCase("four")) {
                        result += 4;
                    } else if (str.equalsIgnoreCase("five")) {
                        result += 5;
                    } else if (str.equalsIgnoreCase("six")) {
                        result += 6;
                    } else if (str.equalsIgnoreCase("seven")) {
                        result += 7;
                    } else if (str.equalsIgnoreCase("eight")) {
                        result += 8;
                    } else if (str.equalsIgnoreCase("nine")) {
                        result += 9;
                    } else if (str.equalsIgnoreCase("ten")) {
                        result += 10;
                    } else if (str.equalsIgnoreCase("eleven")) {
                        result += 11;
                    } else if (str.equalsIgnoreCase("twelve")) {
                        result += 12;
                    } else if (str.equalsIgnoreCase("thirteen")) {
                        result += 13;
                    } else if (str.equalsIgnoreCase("fourteen")) {
                        result += 14;
                    } else if (str.equalsIgnoreCase("fifteen")) {
                        result += 15;
                    } else if (str.equalsIgnoreCase("sixteen")) {
                        result += 16;
                    } else if (str.equalsIgnoreCase("seventeen")) {
                        result += 17;
                    } else if (str.equalsIgnoreCase("eighteen")) {
                        result += 18;
                    } else if (str.equalsIgnoreCase("nineteen")) {
                        result += 19;
                    } else if (str.equalsIgnoreCase("twenty")) {
                        result += 20;
                    } else if (str.equalsIgnoreCase("thirty")) {
                        result += 30;
                    } else if (str.equalsIgnoreCase("forty")) {
                        result += 40;
                    } else if (str.equalsIgnoreCase("fifty")) {
                        result += 50;
                    } else if (str.equalsIgnoreCase("sixty")) {
                        result += 60;
                    } else if (str.equalsIgnoreCase("seventy")) {
                        result += 70;
                    } else if (str.equalsIgnoreCase("eighty")) {
                        result += 80;
                    } else if (str.equalsIgnoreCase("ninety")) {
                        result += 90;
                    } else if (str.equalsIgnoreCase("hundred")) {
                        result *= 100;
                    } else if (str.equalsIgnoreCase("thousand")) {
                        result *= 1000;
                        finalResult += result;
                        result = 0;
                    } else if (str.equalsIgnoreCase("million")) {
                        result *= 1000000;
                        finalResult += result;
                        result = 0;
                    } else if (str.equalsIgnoreCase("billion")) {
                        result *= 1000000000;
                        finalResult += result;
                        result = 0;
                    } else if (str.equalsIgnoreCase("trillion")) {
                        result *= 1000000000000L;
                        finalResult += result;
                        result = 0;
                    }
                }

                finalResult += result;
                log.info("[convertStringToInt] Converted from string " + numberString + " to int " + finalResult);
                return finalResult;
            }
        }
        log.warn("[convertStringToInt] Error converting string representation of integer: " + numberString);
        return -1;
    }

    @Override
    public void sayText(String message) {
        log.info("[sayText] " + message);
        if (sayTextBlocks) {
            log.info("[sayText] sleeping a bit to simulate saying message");
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            log.info("[sayText] done sleeping");
        }
    }

    @Override
    public void sayTextDialogueHistory(String message) {

    }

    @Override
    public void sayTextDialogueHistory(String message, boolean wait) {

    }

    @Override
    public void startQRCodeDisplay(String url, String header) {
        log.info("[startQRCodeDisplay] in method with url: " + url + " and header: " + header);
    }

    @Override
    public void endQRCodeDisplay() {
        log.info("[endQRCodeDisplay] out method");
    }

    @Override
    public void startAcknowledgeDisplay(String message) {
        log.info("[startAcknowledgeDisplay] in method");
    }

    @Override
    public void endAcknowledgeDisplay() {
        log.info("[endAcknowledgeDisplay] in method");
    }

    @Override
    public void setSayTextBlocks(boolean value) {
        sayTextBlocks = value;
    }

    @Override
    public void sendPositionUpdate() {

    }

    @Override
    public boolean relocalize(boolean sayText) {
        return true;
    }

    @Override
    public boolean isLocalized() {
        return true;
    }

    @Override
    public void checkInKiosk(Symbol location) {

    }

    @Override
    public void interruptCheckIn() {

    }

    @Override
    public void enableKioskMode() {

    }

    @Override
    public void disableKioskMode() {

    }

    @Override
    public void setKioskTimeout(Symbol timeout) {

    }

    @Override
    public boolean setKioskLocation(Symbol location) {
        return false;
    }

    @Override
    public void setDisplayAppointmentsSince(Symbol time) {

    }

    @Override
    public int getDisplayAppointmentsSince() {
        return 0;
    }

    @Override
    public void setAllowAppointmentsSince(Symbol time) {

    }

    @Override
    public int getAllowAppointmentsSince() {
        return 0;
    }

    @Override
    public void setLateAppointmentMessage(Symbol message) {

    }

    @Override
    public String getLateAppointmentMessage() {
        return null;
    }

    @Override
    public void setDisplayAppointmentsUntil(Symbol time) {

    }

    @Override
    public int getDisplayAppointmentsUntil() {
        return 0;
    }

    @Override
    public void setAllowAppointmentsUntil(Symbol time) {

    }

    @Override
    public int getAllowAppointmentsUntil() {
        return 0;
    }

    @Override
    public void setEarlyAppointmentMessage(Symbol message) {

    }

    @Override
    public String getEarlyAppointmentMessage() {
        return null;
    }

    @Override
    public void setSuccessfulCheckInMessage(Symbol message) {

    }

    @Override
    public String getSuccessfulCheckInMessage() {
        return null;
    }

    @Override
    public void setBirthdayMessageRange(Symbol range) {

    }

    @Override
    public int getBirthdayMessageRange() {
        return 0;
    }

    @Override
    public void setBirthdayMessage(Symbol message) {

    }

    @Override
    public String getBirthdayMessage() {
        return null;
    }

    @Override
    public void setAppointmentLoadErrorMessage(Symbol message) {

    }

    @Override
    public String getAppointmentLoadErrorMessage() {
        return null;
    }

    @Override
    public void setAppointmentListEmptyMessage(Symbol message) {

    }

    @Override
    public String getAppointmentListEmptyMessage() {
        return null;
    }

    @Override
    public void setCheckInHeaderMessage(Symbol message) {

    }

    @Override
    public String getCheckInHeaderMessage() {
        return null;
    }

    @Override
    public void displayFreezeScreen() {
        log.info("[displayFreezeScreen]");
        freezeLock.lock();
        try {
            freezeCondition.await();
        } catch (InterruptedException e) {
            log.error("[freeze]", e);
        } finally {
            freezeLock.unlock();
        }
    }

    @Override
    public void endDisplayFreezeScreen() {
        log.info("[endDisplayFreezeScreen]");
        freezeLock.lock();
        try {
            freezeCondition.signalAll();
        } finally {
            freezeLock.unlock();
        }
    }

    //TODO: rename from freeze to not be confused with agent freezing (or rename agent freezing to lock or something)
    //Should be an action script
    //TODO: Parallel updates on temi side are required
    @Override
    public void freezeTemi() {
        displayFreezeScreen();
    }

    @Override
    public void endFreezeTemi() {
        endDisplayFreezeScreen();
    }

    @Override
    public List<Float> getCurrentPosition() {
        List<Float> currentPos = new ArrayList<>();
        currentPos.add(1.0f);
        currentPos.add(1.0f);
        currentPos.add(1.0f);
        return currentPos;
    }

    ///////////////// Unnecessary in Mock /////////////////

    @Override
    public void endRepositionLoop() {
    }

    @Override
    public boolean getRepositionFailure() {
        return false;
    }

    @Override
    public void backupLocations(Symbol collectionName) {

    }

    @Override
    public void restoreLocations(Symbol collectionName) {

    }

    @Override
    public void deleteLocationBackup(Symbol collectionName) {

    }

    @Override
    public List<String> getLocationBackupCollections() {
        return null;
    }

    @Override
    public void waitForDetection() {

    }

    @Override
    public void playVideo(Symbol name) {
        log.info("[playVideo]: " + name);
    }

    @Override
    public void closeVideo() {

    }

    @Override
    public void display(Symbol message) {

    }

    @Override
    public void clearScreen() {

    }

    @Override
    public void showTopBar() {

    }

    @Override
    public void uploadMap(boolean overwrite) {

    }

    @Override
    public void updateHomeBase(boolean overwriteMap) {

    }

    @Override
    public void chargeTemi() {

    }

    @Override
    public void displayChargingAlert() {

    }

    @Override
    public void setBatteryWarnLevel(Symbol warnLevel) {

    }

    @Override
    public void setBatteryAbortLevel(Symbol abortLevel) {

    }

    @Override
    public void setBatteryChargedLevel(Symbol chargedLevel) {

    }

    @Override
    public void setVolume(Symbol volume) {

    }

    @Override
    public void setHardButtons(boolean disabled) {

    }

    @Override
    public void enablePrivacyMode() {

    }

    @Override
    public void disablePrivacyMode() {

    }

    @Override
    public void requestPermissions() {

    }

    @Override
    public void restartApp() {

    }

    @Override
    public void enableVoiceAck() {

    }

    @Override
    public void disableVoiceAck() {

    }

    @Override
    public void enableButtons() {

    }

    @Override
    public void disableButtons() {

    }

    @Override
    public void enableObstacleNotifications() {

    }

    @Override
    public void disableObstacleNotifications() {

    }

    @Override
    public void enableKidsMode() {

    }

    @Override
    public void disableKidsMode() {

    }

    @Override
    public void displayFace() {

    }

    @Override
    //Convert goal predicate to human readable form for display in goal queue GUI
    public String convertPredToReadable(String predString) {
        String[] splitRes = predString.split("[(),](?=[^\"][^\"(),]*[^\"][(),]|\".*\"(?![^,()]))");


        try {
            //Trim end parens from last arg - sloppy, do this better
            int resLen = splitRes.length;
            while (splitRes[resLen-1].endsWith(")")) {
                splitRes[resLen-1] = splitRes[resLen-1].substring(0,splitRes[resLen-1].length()-1);
            }

            String funcName = splitRes[0];
            switch (funcName) {
                case "followMe":
                case "followMeBlocking":
                    return "Follow";
                case "goToLocation":
                    return String.format("Go to location '%s'", getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
                case "saveLocation":
                    return String.format("Save location '%s'", splitRes[2]);
                case "deleteLocation":
                    return String.format("Delete location '%s'", getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
                case "display":
                    return String.format("Display message '%s'", splitRes[2]);
                case "sayText":
                    return String.format("Say message '%s'", splitRes[2]);
                case "playVideo":
                    return String.format("Play video '%s'", splitRes[2]);
                case "escort":
                    if (splitRes.length == 6) {
                        return String.format("Escort %s from %s to %s", splitRes[3], getLocationNameFromReference(Factory.createSymbol(splitRes[4])), getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
                    } else {
                        return String.format("Escort %s from current location to %s", splitRes[3], getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
                    }
                case "fetch":
                    if (splitRes.length == 5) {
                        return String.format("Fetch %s from %s", splitRes[2], getLocationNameFromReference(Factory.createSymbol(splitRes[3])));
                    } else {
                        return String.format("Fetch %s", splitRes[2]);
                    }
                case "greet":
                    return "Greet in " + splitRes[2];
                case "tour":
                    return "Tour";
                case "clearScreen":
                    return "Clearing Screen";
                case "displayFace":
                    return "Display Face";
                case "chargeTemi":
                    return "Charging";
                case "goToThenSay":
                    return String.format("Go to %s then say message", getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
                case "goToThenDisplay":
                    return String.format("Go to %s then display message", getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
                case "goToThenPlay":
                    return String.format("Go to %s then play video %s", getLocationNameFromReference(Factory.createSymbol(splitRes[2])), splitRes[3]);
                case "setVolume":
                    return String.format("Set volume to %s", splitRes[2]);
                case "displayQRCode":
                    return "Display Website QR";
                case "relocalize":
                    return "Reposition";
                case "checkInKiosk":
                    return String.format("Check in patients at location %s", getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
                default:
                    return predString;
            }
        } catch (Exception e) {

        }
        return predString;
    }

    @TRADEService
    @Action
    public void updateFirebaseDoc( Map<String,Object> goalInfo) {
        log.debug("[updateFirebaseDoc] "+goalInfo);
    }


    @TRADEService
    @Action
    public boolean ignoreTentativeAccept(Predicate goalPred){
        log.info("[ignoreTentativeAccept] goalPred: "+goalPred);
        if(goalPred != null) {
            switch (goalPred.getName()) {
                case "sayText":
                case "generateResponse":
                case "generateResponseFromString":
                case "setVolume":
                case "sendMap":
                case "sendLogs":
                case "requestPermissions":
                case "restartApp":
                case "disableButtons":
                case "enableButtons":
                case "showTopBar":
                case "enableObstacleNotifications":
                case "disableObstacleNotifications":
                case "enablePrivacyMode":
                case "disablePrivacyMode":
                case "enableVoiceAck":
                case "disableVoiceAck":
                case "setEscortWaitAttempts":
                case "setEscortWaitDuration":
                case "setBatteryAbortLevel":
                case "setBatteryWarnLevel":
                case "setBatteryChargedLevel":
                case "chargeTemi":
                case "displayFace":
                case "enableKidsMode":
                case "disableKidsMode":
                case "sendPositionUpdate":
                case "stopListening":
                case "endFreezeTemi":
                case "enableKioskMode":
                case "disableKioskMode":
                case "setKioskTimeout":
                case "setKioskLocation":
                case "setDisplayAppointmentsSince":
                case "setAllowAppointmentsSince":
                case "setLateAppointmentMessage":
                case "setDisplayAppointmentsUntil":
                case "setAllowAppointmentsUntil":
                case "setEarlyAppointmentMessage":
                case "setSuccessfulCheckInMessage":
                case "setBirthdayMessageRange":
                case "setBirthdayMessage":
                case "setAppointmentLoadErrorMessage":
                case "setAppointmentListEmptyMessage":
                case "setCheckInHeaderMessage":
                case "setStorageArea":
                    return true;
                default:
                    return false;
            }
        }
        return false;
    }

    @TRADEService
    public boolean skipsQueue(String funcName) {
        switch (funcName) {
            case "cancelSystemGoals":
            case "cancelPendingGoalByIndex":
            case "acknowledge":
            case "freeze":
            case "endFreeze":
            case "resumeGoal":
            case "suspendGoal":
            case "stopMoving":
            case "interruptSaveLocation":
            case "sendPositionUpdate":
            case "stopListening":
            case "interruptFollowBlocking":
            case "interruptGoToLocation":
            case "setVolume":
            case "sendMap":
            case "sendLogs":
            case "requestPermissions":
            case "restartApp":
            case "disableButtons":
            case "enableButtons":
            case "showTopBar":
            case "enableObstacleNotifications":
            case "disableObstacleNotifications":
            case "enablePrivacyMode":
            case "disablePrivacyMode":
            case "enableVoiceAck":
            case "disableVoiceAck":
            case "setEscortWaitAttempts":
            case "setEscortWaitDuration":
            case "setBatteryAbortLevel":
            case "setBatteryWarnLevel":
            case "setBatteryChargedLevel":
            case "enableKidsMode":
            case "disableKidsMode":
//            case "displayFace":
            case "downloadVideos":
            case "interruptCheckIn":
            case "enableKioskMode":
            case "disableKioskMode":
            case "setKioskTimeout":
            case "setKioskLocation":
            case "setDisplayAppointmentsSince":
            case "setAllowAppointmentsSince":
            case "setLateAppointmentMessage":
            case "setDisplayAppointmentsUntil":
            case "setAllowAppointmentsUntil":
            case "setEarlyAppointmentMessage":
            case "setSuccessfulCheckInMessage":
            case "setBirthdayMessageRange":
            case "setBirthdayMessage":
            case "setAppointmentLoadErrorMessage":
            case "setAppointmentListEmptyMessage":
            case "setCheckInHeaderMessage":
            case "setStorageArea":
                return true;
        }
        return false;
    }

    @Override
    protected void shutdownComponent() {
        try {
            TRADE.deregister(consultant);
        } catch (TRADEException e) {
            log.error("[shutdownComponent] exception de registering consultant", e);
        }
    }

}

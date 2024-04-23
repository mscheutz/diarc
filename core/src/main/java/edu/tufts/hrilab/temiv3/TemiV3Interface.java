/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.temiv3;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Effect;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import java.util.List;

public interface TemiV3Interface {

    /**
     * checks that location is saved then goes to that location.
     *
     * @param location label for location to go to
     */
    //TODO: move effects (and interrupt behavior) to action scripts
    @OnInterrupt(onCancelServiceCall = "interruptGoToLocation()", onSuspendServiceCall = "interruptGoToLocation()")
    @TRADEService
    @Action
    @Effect( effect={"not(at(?actor,X))"},
            type= EffectType.ALWAYS)
    @Effect( effect={"at(?actor,?location)"},
            type= EffectType.SUCCESS)
    @Effect( effect={"at(?actor,unknown)"},
            type= EffectType.FAILURE)
    @Effect( effect={"at(?actor,unknown)"},
            type= EffectType.NONPERF)
    boolean goToLoc(Symbol location, Boolean escorting);

    @TRADEService
    @Action
    void interruptGoToLocation();

    //TODO:brad: ideally, we should get rid of this. In the action we could create a temporary location, and then use the standard go to location (symbol). I think that we should aim for as many actions as possible to consume FoL classes.
    /**
     * Goes to desired coordinates.
     * <br> Used when fetch script is enacted from an unknown starting location
     *
     * @param goalCoords coordinates to go to
     */
    @TRADEService
    @Action()
    @OnInterrupt(onCancelServiceCall = "interruptGoToLocation()", onSuspendServiceCall = "interruptGoToLocation()")
    @Effect( effect={"not(at(?actor,X))"},
            type= EffectType.ALWAYS)
    @Effect( effect={"at(?actor,unknown)"},
            type= EffectType.ALWAYS)
    boolean goToPos(List<Float> goalCoords, Symbol destination, boolean clearScreen);

    @TRADEService
    @Action
    void followMe();

    /**
     * Blocking version of followMe (waits for acknowledgement)
     */
    @TRADEService
    @Action
    @OnInterrupt(onCancelServiceCall = "interruptFollowBlocking()", onSuspendServiceCall = "interruptFollowBlocking()")
    @Effect( effect={"not(at(?actor,X))"},
            type= EffectType.ALWAYS)
    @Effect( effect={"at(?actor,unknown)"},
            type= EffectType.ALWAYS)
    void followMeBlocking();

    @Action
    @TRADEService
    void interruptFollowBlocking();

    /**
     * Stop moving. Has side effect of causing goToLocationStatus and ReposeStatus to be set to ABORT. We additionally
     *  unlock the goToLoc() condition to prevent deadlock upon temi primitives failing.
     */
    @TRADEService
    @Action
    void stopMoving();

    /**
     * Utility func for interrupting goToLocation in case we are currently repositioning
     */
    @TRADEService
    @Action
    void endRepositionLoop();

    /**
     * Helper function to exit with correct failure predicate in case goToLocation fails due to repositioning errors
     * @return
     */
    @TRADEService
    @Action
    boolean getRepositionFailure();

    /**
     * Stores robots current location
     *
     * @param location label for current location
     */
    @OnInterrupt(
            onCancelServiceCall = "interruptSaveLocation()", onSuspendServiceCall = "interruptSaveLocation()"
    )
    @TRADEService
    @Action
    @Effect( effect={"not(at(?actor,X))"},
            type= EffectType.SUCCESS)
    @Effect( effect={"at(?actor,?location)"},
            type= EffectType.SUCCESS)
    void saveLocation(Symbol location);

    @TRADEService
    @Action
    public void interruptSaveLocation();
    @TRADEService
    @Action
    void backupLocations(Symbol collectionName);
    @TRADEService
    @Action
    void restoreLocations(Symbol collectionName);
    @TRADEService
    @Action
    void deleteLocationBackup(Symbol collectionName);
    @TRADEService
    @Action
    List<String> getLocationBackupCollections();

    /**
     * deletes location with label @location
     *
     * @param location label for location to be deleted
     */
    @TRADEService
    @Action
    void deleteLocation(Symbol location);

    /**
     * Gets Temi's current x and y coordinates on the map. Used in fetch script to store coordinates to return to when
     *  called from an unknown location.
     * @return Temi's x,y coordinates
     */
    @TRADEService
    @Action
    List<Float> getCurrentPosition();

    /**
     * Checks whether the temi has stored the supplied location as a known location. Used to exit goToLocation script
     * when appropriate with the correct failure predicate.
     * @param location location name to check against temi's known location
     * @return true if there exists a matching location, false otherwise
     */

    @OnInterrupt(onCancelServiceCall = "interruptFollowBlocking()", onSuspendServiceCall = "interruptFollowBlocking()")
    @TRADEService
    @Action
    boolean knowsLocation(Symbol location);

    /**
     * for control app
     * @return list of temi's known location names
     */
    @TRADEService
    @Action
    List<String> getKnownLocationNames();

    /**
     * wait for Temi to detect a person
     */
    @TRADEService
    @Action
    void waitForDetection();

    /**
     * Blocks until acknowledged is set to true
     */
    @TRADEService
    @Action
    void startAcknowledgeDisplay(String message);

    /**
     * sets acknowledged to true
     */
    @TRADEService
    @Action
    void endAcknowledgeDisplay();

    /**
     * Duplicate function necessary for scoping between freeze and underlying paused action acknowledgements
     */
    @TRADEService
    @Action
    @OnInterrupt(
            onCancelServiceCall = "endFreezeTemi()",
            onSuspendServiceCall = "endFreezeTemi()"
    )
    void freezeTemi();

    @TRADEService
    @Action
    void endFreezeTemi();

    @TRADEService
    @Action
    @OnInterrupt(
        onCancelServiceCall = "endDisplayFreezeScreen()",
        onSuspendServiceCall = "endDisplayFreezeScreen()"
    )
    void displayFreezeScreen();

    @TRADEService
    @Action
    void endDisplayFreezeScreen();

    /**
     * play video, blocking until {@link #closeVideo} is called
     */
    @TRADEService
    @Action
    @OnInterrupt(onCancelServiceCall = "closeVideo()", onSuspendServiceCall = "closeVideo()")
    void playVideo(Symbol name);

    /**
     * signals videoDone lock allowing {@link #playVideo} to end and clears the screen
     */
    @TRADEService
    @Action
    void closeVideo();

    /**
     * Linked to language "display" or "display message"
     * Navigates to display fragment with supplied message, not blocking
     * @param message message to display on screen
     */
    @TRADEService
    @Action()
    void display(Symbol message);

    /**
     * returns temi tablet to main screen
     */
    @TRADEService
    @Action
    void clearScreen();

    /**
     * show top bar on temi tablet
     */
    @TRADEService
    @Action
    void showTopBar();

    /**
     * Sends temi map data to firebase
     */
    @TRADEService
    @Action
    void uploadMap(boolean overwrite);

    @TRADEService
    @Action
    void updateHomeBase(boolean overwriteMap);

    /**
     * Linked to Language "charge"
     * <br> Behavior which is automatically enacted upon battery reaching a certain percent threshold. Informs the user that the
     * charge behavior is being enacted, then proceeds to navigate to home base and wait either for acknowledgement or to
     * reach back up to a certain battery level. The associated language command "charge" will also cause dialogue
     * to clear the current goal queue, ensuring this task is either immediately executed or first on the queue.
     */
    @TRADEService
    @Action
    @OnInterrupt(onCancelServiceCall = "interruptFollowBlocking()", onSuspendServiceCall = "interruptFollowBlocking()")
    void chargeTemi();

    /**
     * zno displays alert signalling that new tasks cannot be submitted while zno is charging
     */
    @TRADEService
    @Action
    void displayChargingAlert();

    /**
     * Sets the value stored in SharedPreferences with the key "escortWaitDuration". Used to determine the duration to
     * wait for acknowledgement between requests within the escort script.
     */
    @TRADEService
    @Action
    void setEscortWaitDuration(Symbol waitDuration);

    /**
     * Sets the value stored in SharedPreferences with the key "escortWaitAttempts". Used to determine the number
     * of times the temi calls out the patient name and waits for acknowledgement within the escort script.
     */
    @TRADEService
    @Action
    void setEscortWaitAttempts(Symbol waitAttempts);

    /**
     * Sets the value stored in SharedPreferences with the key "chargeWarningPercent". Used to determine battery level
     * upon which to perform a warning sayText to indicate the temi will clear tasks to charge soon
     */
    @TRADEService
    @Action
    void setBatteryWarnLevel(Symbol warnLevel);

    /**
     * Sets the value stored in SharedPreferences with the key "chargeActionPercent". Used to determine battery level
     * upon which the temi enacts its charge behavior
     */
    @TRADEService
    @Action
    void setBatteryAbortLevel(Symbol abortLevel);

    /**
     * Sets the value stored in SharedPreferences with the key "chargeAbortPercent". Used to determine battery level
     * upon which the temi enacts its charge behavior
     */
    @TRADEService
    @Action
    void setBatteryChargedLevel(Symbol chargedLevel);

    /**
     * Gets the value stored in SharedPreferences with the key "escortWaitDuration". Used to determine the duration to
     * wait for acknowledgement between requests within the escort script.
     */
    @TRADEService
    @Action
    long getEscortWaitDuration();

    /**
     * Gets the value stored in SharedPreferences with the key "escortWaitAttempts". Used to determine the number
     * of times the temi calls out the patient name and waits for acknowledgement within the escort script.
     */
    @TRADEService
    @Action
    int getEscortWaitAttempts();

    /**
     * Calls setVolume from temi sdk
     * @param volume value to set volume to
     */
    @TRADEService
    @Action
    public void setVolume(Symbol volume);

    /**
     * Disables/enables hardButtons on the temi. Currently only acts on the volume buttons and the main top bar, does not
     *  affect the power button
     * @param disabled true to disable the hardButtons, false to enable them
     */
    @TRADEService
    @Action
    public void setHardButtons(boolean disabled);

    @TRADEService
    @Action
    void sayText(String message);

    //TODO: remove after action reimplmentation
    @TRADEService
    @Action
    void sayTextDialogueHistory(String message);

    @TRADEService
    @Action
    void sayTextDialogueHistory(String message, boolean wait);

    @TRADEService
    @Action
    void enablePrivacyMode();

    @TRADEService
    @Action
    void disablePrivacyMode();

    @TRADEService
    @Action
    void requestPermissions();

    @TRADEService
    @Action
    void restartApp();

    @TRADEService
    @Action
    void enableVoiceAck();

    @TRADEService
    @Action
    void disableVoiceAck();

    @TRADEService
    @Action
    void enableButtons();

    @TRADEService
    @Action
    void disableButtons();

    @TRADEService
    @Action
    void enableObstacleNotifications();

    @TRADEService
    @Action
    void disableObstacleNotifications();

    @TRADEService
    @Action
    void enableKidsMode();

    @TRADEService
    @Action
    void disableKidsMode();

    @TRADEService
    @Action
    void displayFace();

    @TRADEService
    @Action
    void startQRCodeDisplay(String url, String header);

    @TRADEService
    @Action
    void endQRCodeDisplay();

    @Action
    @TRADEService
    String getLocationNameFromReference(Symbol locRef);

    @TRADEService
    String convertPredToReadable(String predString);

    @TRADEService
    @Action
    void setSayTextBlocks(boolean value);

    @Action
    @TRADEService
    void sendPositionUpdate();

    //TODO: will be relocalizeTemi post reimplementation
    @Action
    @TRADEService
    @OnInterrupt(
            onCancelServiceCall = "stopMoving()", onSuspendServiceCall = "stopMoving()"
    )
    boolean relocalize(boolean sayText);

    @Action
    @TRADEService
    boolean isLocalized();

    @Action
    @TRADEService
    @OnInterrupt(
            onCancelServiceCall = "interruptCheckIn()", onSuspendServiceCall = "interruptCheckIn()"
    )
    void checkInKiosk(Symbol location);

    @Action
    @TRADEService
    void interruptCheckIn();

    @Action
    @TRADEService
    void enableKioskMode();

    @Action
    @TRADEService
    void disableKioskMode();

    @Action
    @TRADEService
    void setKioskTimeout(Symbol timeout);

    @Action
    @TRADEService
    boolean setKioskLocation(Symbol location);

    @Action
    @TRADEService
    void setDisplayAppointmentsSince(Symbol time);

    @TRADEService
    int getDisplayAppointmentsSince();

    @Action
    @TRADEService
    void setAllowAppointmentsSince(Symbol time);

    @TRADEService
    int getAllowAppointmentsSince();

    @Action
    @TRADEService
    void setLateAppointmentMessage(Symbol message);

    @TRADEService
    String getLateAppointmentMessage();

    @Action
    @TRADEService
    void setDisplayAppointmentsUntil(Symbol time);

    @TRADEService
    int getDisplayAppointmentsUntil();

    @Action
    @TRADEService
    void setAllowAppointmentsUntil(Symbol time);

    @TRADEService
    int getAllowAppointmentsUntil();

    @Action
    @TRADEService
    void setEarlyAppointmentMessage(Symbol message);

    @TRADEService
    String getEarlyAppointmentMessage();

    @Action
    @TRADEService
    void setSuccessfulCheckInMessage(Symbol message);

    @TRADEService
    String getSuccessfulCheckInMessage();

    @Action
    @TRADEService
    void setBirthdayMessageRange(Symbol range);

    @TRADEService
    int getBirthdayMessageRange();

    @Action
    @TRADEService
    void setBirthdayMessage(Symbol message);

    @TRADEService
    String getBirthdayMessage();

    @Action
    @TRADEService
    void setAppointmentLoadErrorMessage(Symbol message);

    @TRADEService
    String getAppointmentLoadErrorMessage();

    @Action
    @TRADEService
    void setAppointmentListEmptyMessage(Symbol message);

    @TRADEService
    String getAppointmentListEmptyMessage();

    @Action
    @TRADEService
    void setCheckInHeaderMessage(Symbol message);

    @TRADEService
    String getCheckInHeaderMessage();

    //TODO: Until we figure out where to put all of this demo-specific functionality, need to have these to have temi configs work generally
    @TRADEService
    boolean ignoreTentativeAccept(Predicate goalPred);

    @TRADEService
    boolean skipsQueue(String funcName);
}
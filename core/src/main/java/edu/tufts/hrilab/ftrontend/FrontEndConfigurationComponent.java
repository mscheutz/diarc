package edu.tufts.hrilab.ftrontend;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Set;

/**
 * Class which holds intended configuration of the front end of the system and exposes TRADE services for the rest of the system to branch logic based on what
 * components and services should exist.
 *
 * TODO: should this be a standalone component which the developer needs to make sure to properly align with the associated config or should it be responsible
 *       for creating and registering other components based on the supplied configuration?
 */
public class FrontEndConfigurationComponent extends DiarcComponent
{
   //TODO: can likely merge some arguments into superset CLI args (e.g. -socialLLM)
   private boolean useStreamingTTS = false;
   private boolean useTentativeAcceptFiller = true;
   private boolean usePreValidationFiller = true;
   private boolean useReattemptParseFiller = true;
   private boolean supportExtendedParses = true;
   private boolean useWWBasedASR = false;
   private boolean useListeningIndicator = true;
   private boolean useConfirmation = true;
   private boolean usingQueueEM = true;
   private boolean usingLLM = true;
   private String language = "English";

   // Addressees the robot is physically co-present with: their replies are spoken aloud. Replies to
   // any other addressee (webapp, remote/text channels) are delivered via dialogue history only.
   public final Set<String> LOCAL_ADDRESSEES = Set.of("local_user", "local_visitor", "operator");

   @Override
   protected List<Option> additionalUsageInfo() {
      // TODO: only the two switches the ava config needs for now. the rest of the flags
      // above are still hardcoded and want a real configuration pass
      List<Option> options = new ArrayList<>();
      options.add(Option.builder("streamingTTS").desc("stream llm replies to the tts as they generate").build());
      options.add(Option.builder("wwBasedASR").desc("listening is wake word driven, rearm after replies").build());
      return options;
   }

   @Override
   protected void parseArgs(CommandLine cmdLine) {
      if (cmdLine.hasOption("streamingTTS")) {
         useStreamingTTS = true;
      }
      if (cmdLine.hasOption("wwBasedASR")) {
         useWWBasedASR = true;
      }
   }

   @Override
   protected void init() {
      //TODO: check configuration
   }

   //TODO: This doesn't really make sense, needs to be a pair of interactors. Currently this is spiritually whether the
   //      original input is from the webapp or not. Naming of everything is misleading
   @TRADEService
   public boolean isLocalAddressee(Symbol addressee) {
      return addressee != null && LOCAL_ADDRESSEES.contains(addressee.getName());
   }

   @TRADEService
   public boolean isLocalAddressee(String addresseeName) {
      return addresseeName != null && LOCAL_ADDRESSEES.contains(addresseeName);
   }

   @TRADEService
   public boolean usingStreamingTTS(Symbol agentTeam) {
      log.debug("[usingStreamingTTS] {}",agentTeam);
      return useStreamingTTS && isLocalAddressee(agentTeam);
   }
   @TRADEService
   public boolean usingWWBasedASR(Symbol agentTeam) {
      log.debug("[usingWWBasedASR] {}",agentTeam);
      return useWWBasedASR && isLocalAddressee(agentTeam);
   }
   @TRADEService
   public boolean usingListeningIndicator(Symbol agentTeam) {
      log.debug("[usingListeningIndicator] {}",agentTeam);
      return useListeningIndicator && isLocalAddressee(agentTeam);
   }

   //Don't think we need these 3 if having usingLLM (unless we want filler presence to be configurable)
   @TRADEService
   public boolean useTentativeAcceptFiller() {
      return useTentativeAcceptFiller;
   }
   @TRADEService
   public boolean usePreValidationFiller() {
      return usePreValidationFiller;
   }
   @TRADEService
   public boolean useReattemptParseFiller() {
      return useReattemptParseFiller;
   }

   @TRADEService
   public boolean usingLLM() {
      return usingLLM;
   }
   //TODO: remove supportExtendedParses and just replace with usingSocialLLM for now?
   @TRADEService
   public boolean supportExtendedParses() {
      return supportExtendedParses;
   }
   @TRADEService
   public boolean usingValidation() {
      return useConfirmation;
   }
   @TRADEService
   public boolean usingQueueEM() {
      return usingQueueEM;
   }



   @TRADEService
   public String sampleReattemptParseFillerPhrase() {
      int option = new Random().nextInt(4);
      return switch (option) {
         case 0 -> "My mistake.";
         case 1 -> "Sorry about that.";
         case 2 -> "Oh, okay.";
         case 3 -> "Apologies.";
         default -> "Okay";
      };
   }

   @TRADEService
   public String samplePreValidationFillerPhrase(Utterance unvalidatedUtterance, boolean revalidating) {
      Symbol semantics = unvalidatedUtterance.getSemantics();
      //TODO: should generally match any "focusedContext" id
      if (semantics.getName().equals("submitTaskSequenceDefinition") || semantics.getName().equals("scheduled")) {
         int option = new Random().nextInt(2);
         return switch (option) {
            case 0 -> "Let me make sure I understand";
            case 1 -> "To be sure I understood correctly";
            default -> "Okay.";
         };
      } else if (revalidating) {
         int option = new Random().nextInt(8);
         return switch (option) {
            case 0 -> "Updating";
            case 1 -> "Reconsidering";
            case 2 -> "Reevaluating";
            case 3 -> "Reassessing";
            case 4 -> "Correcting";
            case 5 -> "Reanalyzing";
            case 6 -> "Rethinking";
            case 7 -> "Revisiting";
            default -> "Okay.";
         };
      } else {
         int option = new Random().nextInt(6);
         return switch (option) {
            case 0 -> "Okay.";
            case 1 -> "Alright.";
            case 2 -> "Copy.";
            case 3 -> "Heard.";
            case 4 -> "Gotcha.";
            case 5 -> "Processing.";
            default -> "Okay.";
         };
      }
   }

   @TRADEService
   public String sampleGoalAchievedPhrase(Predicate state) {
      String name = state.getName();
      //If movement based:
      switch (name) {
         case "breakOpenDoor":
         case "escortCivilianToSafety":
            return "";
         case "goToLocation":
         case "escortFromToLocation":
         case "measureWifiAtLocations":
         case "measureWifiAtLocationsAndReportAt":
         case "measureTemperatureAtLocations":
         case "measureTemperatureAtLocationsAndReportAt":
         case "playNextVideoAtLocation":
         case "playSpecificVideoAtLocation":
         case "displayNamedImageAtLocation":
         case "displayNamedImageForDurationAtLocation":
            int option = new Random().nextInt(3);
            return switch (option) {
               case 0 -> "Made it there.";
               case 1 -> "Reached my destination.";
               case 2 -> "Arrived.";
               default -> "Okay";
            };
      }
      int option = new Random().nextInt(5);
      return switch (option) {
         case 0 -> "Done.";
         case 1 -> "Succeeded.";
         case 2 -> "Complete.";
         case 3 -> "Finished.";
         case 4 -> "Achieved.";
         default -> "Okay";
      };
   }

   @TRADEService
   public String sampleTentativeAcceptPhrase(Predicate state) {
      String name = state.getName();
      //If movement based:
      switch (name) {
         case "breakOpenDoor":
         case "escortCivilianToSafety":
            return "";
         case "goToLocation":
         case "escortFromToLocation":
         case "measureWifiAtLocations":
         case "measureWifiAtLocationsAndReportAt":
         case "measureTemperatureAtLocations":
         case "measureTemperatureAtLocationsAndReportAt":
         case "playNextVideoAtLocation":
         case "playSpecificVideoAtLocation":
         case "displayNamedImageAtLocation":
         case "displayNamedImageForDurationAtLocation":
            int option = new Random().nextInt(4);
            return switch (option) {
               case 0 -> "On my way.";
               case 1 -> "Moving.";
               case 2 -> "Heading there.";
               case 3 -> "Leaving now.";
               default -> "Okay";
            };
      }
      int option = new Random().nextInt(10);
      return switch (option) {
         case 0 -> "Understood.";
         case 1 -> "Noted.";
         case 2 -> "Sounds good.";
         case 3 -> "Will do.";
         case 4 -> "Got it.";
         case 5 -> "Confirmed.";
         case 6 -> "Proceeding.";
         case 7 -> "Submitted.";
         case 8 -> "Acknowledged.";
         case 9 -> "Accepted.";
         default -> "Okay";
      };
   }

   @TRADEService
   public String sampleGoalPlacedInQueueAcknowledgement(Predicate state)
   {
      int option = new Random().nextInt(4);
      return switch (option)
      {
         case 0 -> "Will do that when I'm done with my other tasks.";
         case 1 -> "Queueing after my other tasks.";
         case 2 -> "Adding to my list of tasks.";
         case 3 -> "I have other assigned tasks, but I'll get to that when I can.";
         default -> "Okay";
      };
   }

   @TRADEService
   public boolean ignoreValidation(Predicate goalPred){
      if (goalPred != null) {
         switch (goalPred.getName()) {
            case "suspendCurrentTask":
            case "resumeCurrentTask":
            case "suspendGoal":
            case "cancelCurrentTask":
            case "cancelGoal":
            case "dontRespond":
            //case "modifyExistingTaskStatus":
               return true;
            default:
               return false;
         }
      }
      return false;
   }

   //TODO: default load from file (domain dependent)
   @TRADEService
   public boolean ignoreTentativeAccept(Predicate goalPred) {
      if (goalPred != null)
      {
         switch (goalPred.getName())
         {
            case "describeScheduledGoals":
            case "describeCurrentLocation":
            case "describePendingGoals":
            case "describeAllKnownLocations":
            case "repeatLastQuestion":
            case "describeCurrentGoal":
            case "reportTaughtActions":
            case "reportInstructableActions":
            case "describeActionSteps":
            case "reportIfCanPerform":
            case "reportKioskSettings":
            case "listSavedVideos":
            case "reportVideoVolume":
            case "listSavedImages":
            case "reportIsVideoSaved":
            case "sayText":
            case "sayLocally":
            case "reportBatteryLevel":
            case "generateResponseFromString":
            case "getAllKnownLocations":
            case "getBatteryLevelString":
            case "getCurrentLocationString":
            case "getTaughtActionsList":
            case "getIfCanPerform":
            case "getPredefinedActions":
            case "getActionSteps":
            case "getKioskSettings":
            case "getVideoVolume":
            case "getIsVideoSaved":
            case "getSavedVideos":
            case "getSavedImages":
            case "getPreviouslyMeasuredWifiAtLocationString":
            case "getPreviouslyMeasuredTemperatureAtLocationString":
            case "getCurrentPendingScheduledTaskInformation":
            case "resetConversation":
            case "learnTaskSequence":
            case "scheduleTask":
            case "getInformationAroundTeachingNewActions":
            case "getInformationAroundHowToTeachATask":
            case "getInformationAroundWhatALearnedTaskIs":
            case "getInformationAroundWhatActionsCanBePerformedAlongALearnedTask":
            case "getAssistanceOnUsingSaveLocationAndGoToLocationProperly":
            case "getPreviouslyMeasuredWifiAtLocationsString":
            case "getPreviouslyMeasuredWifiAtCurrentLocationString":
            case "getPreviouslyMeasuredTemperatureAtLocationsString":
            case "getPreviouslyMeasuredTemperatureAtCurrentLocationString":
            case "updateDialogueHistoryFromString":
            case "checkPastInteractionHistoryForMissingInformation":
            case "dontRespond":
            case "reportAllPackingParameters":
            case "reportFullyPackedCrateConfiguration":
            case "reportMaxNumberOfPackedItemsPerLayer":
            case "reportMaxNumberOfPackedLayers":
            case "reportMaxNumberOfPackedItems":
            case "reportCurrentPackedCrateConfiguration":
            case "reportCurrentNumberOfLayersToPack":
            case "reportConfiguredNumberOfItemsToPack":
            case "reportCrateDimensions":
            case "reportCrateHeight":
            case "reportCrateWidth":
            case "reportCrateLength":
            case "reportPackedItemDimensions":
            case "reportPackedItemHeight":
            case "reportPackedItemWidth":
            case "reportPackedItemLength":
            case "setNumberOfItemsToPackPerCrate":
            case "reportNumberOfItemsInCrate":
            case "reportUnsetPackingParameters":
            case "say":
            case "generateResponse":
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
            case "learnMission":
            case "getAllKnownRooms":
            case "startMission":
            //case "cancelAllCurrentAndQueuedTasks":
            case "clearRoomForDoor":
            //
            case "follow":
            case "escortToLocation":
            case "escortFromLocationToLocation":
            case "cancelAllScheduledGoals":
            //case "scheduled":
            case "cancelSingleScheduledGoal":
            case "forgetAllTaughtActions":
            case "modifyExistingTaskStatus":
            case "submitTaskSequenceDefinition":
               return true;
            default:
               return false;
         }
      }
      return false;
   }
}

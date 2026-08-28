package edu.tufts.hrilab.slug.nlg.templated;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

//TODO: transform to run through IntelliJ
/**
 * Standalone regression test covering every semantic example in
 * "NLG Semantic Examples.docx".
 *
 * Run from the project root:
 *   javac -d target/regression $(find src/main/java -name '*.java') \
 *       src/test/java/com/example/validationnlg/AllDocumentExamplesRegression.java
 *   java -ea -cp target/regression com.example.validationnlg.AllDocumentExamplesRegression
 */
public final class AllDocumentExamplesRegression {
    private record Example(String id, String semantics, String... requiredContent) {}
    private record TranslationCase(String id, String semantics, String expected) {}

    public static void main(String[] args) {
        EntityResolver resolver = resolver();
        TemplatedNLG canonical = new TemplatedNLG(resolver, RealizationOptions.canonical());
        TemplatedNLG variedA = TemplatedNLG.varied(resolver, 17L);
        TemplatedNLG variedB = TemplatedNLG.varied(resolver, 91L);

        List<String> failures = new ArrayList<>();
        verifyGeneralTemplateTranslations(canonical, failures);
        int generated = 0;
        int visiblyVaried = 0;

        for (Example example : examples()) {
            String base = run(canonical, example, failures, "canonical");
            String a = run(variedA, example, failures, "varied seed 17");
            String b = run(variedB, example, failures, "varied seed 91");
            generated += 3;

            if (base != null && a != null && b != null &&
                    (!base.equals(a) || !base.equals(b) || !a.equals(b))) {
                visiblyVaried++;
            }
        }

        // Variation is intentionally concentrated in validation questions and sequencing.
        if (visiblyVaried < 4) {
            failures.add("Expected at least four examples to exhibit controlled variation; got " + visiblyVaried);
        }

        if (!failures.isEmpty()) {
            System.err.println("REGRESSION FAILURES (" + failures.size() + "):");
            failures.forEach(f -> System.err.println(" - " + f));
            System.exit(1);
        }

        System.out.printf(Locale.ROOT,
                "PASS: %d document examples, %d generated outputs, %d examples with visible variants.%n",
                examples().size(), generated, visiblyVaried);
    }


    /**
     * Exact canonical checks for every entry in General Templates / Modifiers,
     * followed by additional parameterized instances proving the implementation
     * is template-driven rather than hard-coded to the document's constants.
     */
    private static void verifyGeneralTemplateTranslations(TemplatedNLG nlg, List<String> failures) {
        for (TranslationCase tc : generalTemplateCases()) {
            try {
                String actual = nlg.generate(tc.semantics());
                if (!normalize(actual).equals(normalize(tc.expected()))) {
                    failures.add(tc.id() + ": expected <" + tc.expected() + "> but got <" + actual + ">");
                }
            } catch (RuntimeException ex) {
                failures.add(tc.id() + ": " + ex.getClass().getSimpleName() + ": " + ex.getMessage());
            }
        }
    }

    private static List<TranslationCase> generalTemplateCases() {
        return List.of(
            // Documented Standalone Example Forms and Translations.
            new TranslationCase("general-and-document",
                "and(learnedTourSequence(introduction_tour),canAskToPerformBySaying(pleasePerform,introduction_tour))",
                "I have learned the introduction tour sequence, and you can ask me to perform it by saying “please perform the introduction tour”."),
            new TranslationCase("general-not-document", "not(open(door_1))", "Door 1 is not open."),
            new TranslationCase("general-error-document", "error(spot,not(detectDoor))", "I did not see a door."),
            new TranslationCase("general-cannot-document",
                "cannot(goToLocation(ava,reception:location),because(not(localized)))",
                "I cannot go to reception because I am not localized."),
            new TranslationCase("general-ack-document",
                "ack(attempting(ava,searchRoomForPeople(ava,experiment_room)))",
                "Okay, I will attempt to search the experiment room for people."),
            new TranslationCase("general-greeting-document", "greeting(ava,eric,\"Hello ava\")", "Hello Eric."),
            new TranslationCase("general-mean-document", "mean(ava,goToRoom(ava,main_lab_space))",
                "Confirming you want me to go to the main lab space?"),
            new TranslationCase("general-attempting-document",
                "attempting(ava,searchRoomForPeople(ava,experiment_room))",
                "I am attempting to search the experiment room for people."),
            new TranslationCase("general-unknown-document", "doNotKnowWhat(ava,\"knob\",means)",
                "I don't know what “knob” means in the current context."),
            new TranslationCase("general-know-document", "know(ava,howTo(stand(ava)))", "I know how to stand."),
            new TranslationCase("general-urgent-document", "urgentPriority(goToLocation(ava,reception:location))",
                "I am going to reception immediately."),

            // Additional instances of every template: different predicates and arguments.
            new TranslationCase("general-and-parameterized",
                "and(learnedTourSequence(safety_round),canAskToPerformBySaying(run,safety_round))",
                "I have learned the safety round sequence, and you can ask me to perform it by saying “run the safety round”."),
            new TranslationCase("general-not-parameterized", "not(open(storage_door))", "Storage door is not open."),
            new TranslationCase("general-error-parameterized", "error(ava,not(open(storage_door)))",
                "Storage door is not open."),
            new TranslationCase("general-cannot-parameterized",
                "cannot(goToRoom(spot,back_room),because(not(open(storage_door))))",
                "I cannot go to the back room because storage door is not open."),
            new TranslationCase("general-ack-parameterized", "ack(attempting(spot,scanForDoor(spot)))",
                "Okay, I will attempt to scan the area in front of me for doors."),
            new TranslationCase("general-greeting-parameterized", "greeting(ava,maria,\"Good morning\")", "Hello Maria."),
            new TranslationCase("general-mean-parameterized", "mean(local_user,searchRoomForPeople(spot,back_room))",
                "Confirming you want me to search the back room for people?"),
            new TranslationCase("general-attempting-parameterized", "attempting(spot,goToRoom(spot,back_room))",
                "I am attempting to go to the back room."),
            new TranslationCase("general-unknown-parameterized", "doNotKnowWhat(ava,\"foyer\",means)",
                "I don't know what “foyer” means in the current context."),
            new TranslationCase("general-know-parameterized", "know(spot,howTo(scanForDoor(spot)))",
                "I know how to scan the area in front of me for doors."),
            new TranslationCase("general-urgent-parameterized", "urgentPriority(searchRoomForPeople(spot,back_room))",
                "I am going to search the back room for people immediately.")
        );
    }

    private static String run(TemplatedNLG nlg, Example example, List<String> failures, String mode) {
        try {
            String output = nlg.generate(example.semantics());
            if (output == null || output.isBlank()) {
                failures.add(example.id() + " [" + mode + "]: blank output");
                return output;
            }
            if (!(output.endsWith(".") || output.endsWith("?"))) {
                failures.add(example.id() + " [" + mode + "]: missing terminal punctuation: " + output);
            }
            String normalized = normalize(output);
            for (String required : example.requiredContent()) {
                if (!normalized.contains(normalize(required))) {
                    failures.add(example.id() + " [" + mode + "]: missing content '" + required + "' in: " + output);
                }
            }
            if (output.matches(".*\\blocation_\\d+\\b.*")) {
                failures.add(example.id() + " [" + mode + "]: leaked internal location identifier: " + output);
            }
            return output;
        } catch (RuntimeException ex) {
            failures.add(example.id() + " [" + mode + "]: " + ex.getClass().getSimpleName() + ": " + ex.getMessage());
            return null;
        }
    }

    private static String normalize(String text) {
        return text.toLowerCase(Locale.ROOT)
                .replace("wi-fi", "wifi")
                .replace("a.m.", "am")
                .replace("p.m.", "pm")
                .replaceAll("[^a-z0-9%]+", " ")
                .trim();
    }

    private static EntityResolver resolver() {
        return new EntityResolver()
                .alias("location_0:location", "chair 1")
                .alias("location_1:location", "chair 2")
                .alias("location_2:location", "chair 3")
                .alias("location_5:location", "pavilion")
                .alias("location_13:location", "cardiology check-in")
                .alias("location_6:location", "reception")
                .alias("building entrance:location", "building entrance")
                .alias("chair_one:location", "chair one")
                .alias("reception_location", "reception")
                .alias("reception:location", "reception")
                .alias("dock:location", "dock");
    }

    private static List<Example> examples() {
        String survey = "\"Hello, do you have time to fill out this survey?\"";
        return List.of(
            // Validation Q / Validation Translation examples
            new Example("validation-mission",
                "mean(local_user,startMission(alex,and(goToRoom(alex,main_lab_space),searchRoomForPeople(alex,main_lab_space),goToRoom(alex,experiment_room),searchRoomForPeople(alex,experiment_room),goToLocationWrapper(alex,building entrance:location))))",
                "mission", "main lab space", "experiment room", "people", "building entrance"),
            new Example("validation-task-definition",
                "mean(webapp,submitTaskSequenceDefinition(ava,patient_survey_round,and(and(\"chair 1\",presentSurvey(ava,"+survey+")),and(\"chair 2\",presentSurvey(ava,"+survey+")),and(\"chair 3\",presentSurvey(ava,"+survey+")))))",
                "patient survey round", "chair", "survey", "do you have time"),
            new Example("validation-schedule-repeating",
                "mean(webapp,scheduled(measureWifiAtLocations(ava,and(location_5:location,location_13:location,location_6:location)),start(10,MINUTES),period(30,MINUTES)))",
                "wifi", "pavilion", "cardiology check in", "reception", "10 minute", "30 minute"),
            new Example("validation-schedule-now",
                "mean(webapp,scheduled(measureWifiAtLocations(ava,and(location_5:location,location_13:location,location_6:location)),start(0,MINUTES),period(30,MINUTES)))",
                "wifi", "pavilion", "reception", "30 minute"),
            new Example("validation-schedule-end",
                "mean(webapp,scheduled(measureWifiAtLocations(ava,and(location_5:location,location_13:location,location_6:location)),start(10,MINUTES),period(30,MINUTES),end(17-00-00)))",
                "wifi", "10 minute", "30 minute", "5 pm"),
            new Example("validation-schedule-once",
                "mean(webapp,scheduled(measureWifiAtLocations(ava,and(location_5:location,location_13:location,location_6:location)),start(2,HOURS)))",
                "wifi", "2 hour"),

            // General templates / modifiers
            new Example("and", "and(learnedTourSequence(introduction_tour),canAskToPerformBySaying(pleasePerform,introduction_tour))", "introduction tour"),
            new Example("not", "not(open(door_1))", "not", "open", "door 1"),
            new Example("error", "error(spot,not(detectDoor))", "did not see", "door"),
            new Example("cannot", "cannot(goToLocation(ava,reception:location),because(not(localized)))", "cannot", "reception", "localized"),
            new Example("ack", "ack(attempting(ava,searchRoomForPeople(ava,experiment_room)))", "okay", "attempt", "experiment room"),
            new Example("greeting", "greeting(ava,eric,\"Hello ava\")", "hello", "eric"),
            new Example("mean-simple", "mean(ava,goToRoom(ava,main_lab_space))", "main lab space"),
            new Example("attempting", "attempting(ava,searchRoomForPeople(ava,experiment_room))", "attempting"),
            new Example("unknown-word", "doNotKnowWhat(ava,\"knob\",means)", "knob"),
            new Example("know-how", "know(ava,howTo(stand(ava)))", "know", "how to"),
            new Example("urgent", "urgentPriority(goToLocation(ava,reception:location))", "reception", "immediately"),

            // Facts and timing
            new Example("at", "at(ava,dock:location)", "at", "dock"),
            new Example("patrol-path", "patrolPath(\"security round\",contains(and(main_entrance:location,back_door:location,side_room:location)))", "security round"),
            new Example("battery", "batteryLevel(65)", "battery level", "65"),
            new Example("task-name", "taskName(buildingSweep)", "task name", "building sweep"),
            new Example("task-predicate", "taskPredicate(dock(ava))", "task predicate"),
            new Example("timing-of", "timingOf(execution)", "timing of", "execution"),
            new Example("abort", "abort(question)", "abort", "question"),
            new Example("kiosk-goal", "kioskGoal(security_patrol(ava),3600)", "kiosk goal", "3600"),
            new Example("start-relative", "start(60,SECONDS)", "start", "60", "seconds"),
            new Example("start-absolute", "start(10-00-00)", "start", "10 00 00"),
            new Example("end-relative", "end(60,SECONDS)", "end", "60", "seconds"),
            new Example("end-absolute", "end(10-00-00)", "end", "10 00 00"),
            new Example("period-relative", "period(30,MINUTES)", "period", "30", "minutes"),
            new Example("period-absolute", "period(18-30-00)", "period", "18 30 00"),
            new Example("scheduled-once", "scheduled(escortFromLocationToLocation(ava,patient,reception:location,chair_one:location),start(15-00-00))", "scheduled", "escort", "reception", "chair one"),
            new Example("scheduled-repeat-end", "scheduled(measureWifiAtLocations(ava,lab:location),period(1,HOURS),end(17-00-00))", "scheduled", "wifi", "lab"),
            new Example("scheduled-window", "scheduled(advertisement_round(ava),start(09-00-00),period(30,MINUTES),end(17-00-00))", "scheduled", "advertisement round"),
            new Example("location", "location(reception:location)", "location", "reception"),
            new Example("option", "option(3)", "option", "3"),
            new Example("amount", "amount(10)", "amount", "10"),
            new Example("starting-patrol", "starting(patrol(ava,officePath,measureTemperature(ava)))", "starting", "patrol"),
            new Example("wifi-quality", "wifiQualityAt(router:location,\"-30dBm\")", "wifi quality", "router", "30dbm"),
            new Example("goal-status", "goal(status(SUSPENDED),predicate(learnAction(ava)))", "goal", "suspended"),
            new Example("crate-width", "crateWidth(230)", "crate width", "230"),
            new Example("crate-length", "crateLength(160)", "crate length", "160"),
            new Example("crate-height", "crateHeight(100)", "crate height", "100"),
            new Example("packable-layers", "maxPackableLayersPerCrate(3)", "max packable layers per crate", "3"),

            // Actions
            new Example("go-location-wrapper", "goToLocationWrapper(ava,exampleLocation:location)", "example location"),
            new Example("go-room", "goToRoom(spot,main_lab_space)", "main lab space"),
            new Example("wait", "waitForDuration(ava,60)", "wait", "60 seconds"),
            new Example("search-room", "searchRoomForPeople(spot,back_room)", "back room", "people"),
            new Example("scan-people", "scanForPeople(spot)", "scan", "people"),
            new Example("clear-room-door", "clearRoomForDoor(spot,\"eric's office\")", "clear room for door", "eric s office"),
            new Example("scan-door", "scanForDoor(spot)", "scan", "door"),
            new Example("break-door", "breakOpenDoor(spot)", "open", "door"),
            new Example("escort-safety", "escortCiviliansToSafety(spot,entrance:location)", "escort civilians to safety", "entrance"),
            new Example("go-location", "goToLocation(ava,exampleLocation:location)", "example location"),
            new Example("follow", "follow(ava)", "follow", "you"),
            new Example("stop-follow", "stopFollowing(ava)", "stop following", "you"),
            new Example("escort-to", "escortToLocation(ava,owen,chair_one:location)", "escort to location", "owen", "chair one"),
            new Example("escort-from-to", "escortFromLocationToLocation(ava,guest,reception_location,chair_one:location)", "escort from location to location", "guest", "reception", "chair one"),
            new Example("save-location", "saveLocation(ava,location_one)", "save location", "location one"),
            new Example("go-home", "goHome(ava)", "home", "charge"),
            new Example("wifi-current", "measureWifiAtCurrentLocation(ava)", "measure wifi at current location"),
            new Example("wifi-locations", "measureWifiAtLocations(ava,and(office_1:location,office_2:location,office_3:location))", "wifi", "office 1", "office 3"),
            new Example("wifi-report", "measureWifiAtLocationsAndReportAt(ava,and(office_1:location,office_2:location,office_3:location),dock:location)", "measure wifi at locations and report at", "dock"),
            new Example("temperature-current", "measureTemperatureAtCurrentLocation(ava)", "measure temperature at current location"),
            new Example("temperature-locations", "measureTemperatureAtLocations(ava,and(office_1:location,office_2:location,office_3:location))", "temperature", "office 1", "office 3"),
            new Example("temperature-report", "measureTemperatureAtLocationsAndReportAt(ava,and(office_1:location,office_2:location,office_3:location),dock:location)", "measure temperature at locations and report at", "dock"),
            new Example("cancel-current", "cancelCurrentTask(ava)", "cancel", "current task"),
            new Example("cancel-queued", "cancelAllQueuedTasks(ava)", "cancel all queued tasks"),
            new Example("cancel-current-queued", "cancelAllCurrentAndQueuedTasks(ava)", "cancel all current and queued tasks"),
            new Example("cancel-queue-index", "cancelSingleTaskInQueue(ava,1)", "cancel task in queue by index", "1"),
            new Example("cancel-scheduled", "cancelAllScheduledGoals(self)", "cancel all scheduled goals"),
            new Example("suspend", "suspendCurrentTask(self)", "suspend", "current task"),
            new Example("resume", "resumeSystemGoals(self)", "resume", "paused task"),
            new Example("forget", "forgetTaughtAction(ava,greetPeopleAtEntrance(ava))", "forget taught action"),
            new Example("kiosk-timeout", "setKioskTimeout(30,MINUTES)", "set kiosk timeout", "30", "minutes"),
            new Example("kiosk-enabled", "setKioskModeEnabled(true)", "set kiosk mode enabled", "true"),
            new Example("video-volume", "setVideoVolume(100)", "set video volume", "100"),
            new Example("next-video", "playNextVideo(ava)", "play next video"),
            new Example("next-video-location", "playNextVideoAtLocation(ava,atrium:location)", "play next video at location", "atrium"),
            new Example("specific-video", "playSpecificVideo(ava,flossing.mp4)", "play specific video", "flossing mp4"),
            new Example("specific-video-location", "playSpecificVideoAtLocation(ava,flossing.mp4,chair_one:location)", "play specific video at location", "chair one"),
            new Example("display-image", "displayNamedImage(ava,infographic.png)", "display named image", "infographic png"),
            new Example("display-image-location", "displayNamedImageAtLocation(ava,infographic.png,chair_one:location)", "display named image at location", "chair one"),
            new Example("display-duration", "displayNamedImageForDuration(ava,infographic.png,180)", "display named image for duration", "180"),
            new Example("display-duration-location", "displayNamedImageForDurationAtLocation(ava,infographic.png,60,chair_one:location)", "display named image for duration at location", "60", "chair one"),
            new Example("stop-talking", "dontRespond(ava)", "stop talking"),
            new Example("schedule-task", "scheduleTask(ava)", "schedule task"),
            new Example("learn-task", "learnTaskSequence(ava)", "learn task sequence"),
            new Example("sit", "sit(ava)", "sit"),
            new Example("stand", "stand(ava)", "stand")
        );
    }
}

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import java.lang.Boolean;
import java.lang.Long;
import java.lang.Integer;
import java.lang.String;
import java.util.List;
import java.util.Map;

() = waitForMediaCompletion() {
    onInterrupt :{
        cancel: tsc:cancelVideoPlayback();
        suspend: tsc:pauseVideoPlayback();
        resume: tsc:resumeVideoPlayback();
    }
    while(tsc:isMediaPlaying()) {
        op:sleep(500);
    }
}

() = playNextVideo() {
    tsc:displayNextVideo();
    act:waitForMediaCompletion();
}

() = playSpecificVideo(String ?videoName) {
    Boolean !videoPlayed;
    !videoPlayed = tsc:displayVideoMatching(?videoName);
    if (op:equals(!videoPlayed,false)) {
        exit(FAIL,couldNotPlayVideo(?videoName));
    }
    act:waitForMediaCompletion();
}

//TODO: These variations shouldn't exist - get LLM to target then(goal1,goal2) semantics generally
() = playNextVideoAtLocation(Symbol ?location:location) {
    act:goToLocation(?location);
    tsc:displayNextVideo();
    act:waitForMediaCompletion();
}

() = playSpecificVideoAtLocation(Symbol ?videoName, Symbol ?location:location) {
    Boolean !videoPlayed;
    String !videoNameString;

    !videoNameString = op:getName(?videoName);
    act:goToLocation(?location);
    !videoPlayed = tsc:displayVideoMatching(!videoNameString);
    if (op:equals(!videoPlayed,false)) {
        exit(FAIL,couldNotPlayVideo(?videoName));
    }
    act:waitForMediaCompletion();
}

() = setVideoVolume(Symbol ?volume) {
    Boolean !success;

    !success = tsc:setVideoVolume(?volume);
    if (op:equals(!success,false)) {
        act:generateResponseFromString("Supplied volume must be between 0 and 100");
        exit(FAIL,invalid(volume));
    }
}

() = lowerVideoVolume() {
    Boolean !success = tsc:lowerVideoVolume();
    if (op:equals(!success,false)) {
        exit(FAIL,invalid(volume));
    }
}

() = raiseVideoVolume() {
    Boolean !success = tsc:raiseVideoVolume();
    if (op:equals(!success,false)) {
        exit(FAIL,invalid(volume));
    }
}

////Could just have playNextVideo kiosk action with low timeout, not sure if this should ideally be instructed directly or exist
//() = playAllVideosLoping() {
//    while (true) {
//        tsc:displayNextVideo();
//        act:waitForMediaCompletion();
//    }
//}
////TODO: Want default AvaDisplayComponent to be configurable between looping images and still image?
////TODO: difference between default looping image set and all images uploaded? I.e. kiosk images vs presentation images?
//() = displayImage() {
//
//}
//
//() = teachPresentationAction() {
//    //Duration upfront
//    //Ask for video/image collection one by one
//    //Should this be done automatically through a GUI upon submission of slides rather than language?
//}

() = displayNamedImage(String ?imageName) {
    act:displayNamedImageForDurationAtLocation(?imageName,-1,current);
}

() = displayNamedImageAtLocation(String ?imageName, Symbol ?location:location) {
    act:displayNamedImageForDurationAtLocation(?imageName,-1,?location);
}

() = displayNamedImageForDuration(String ?imageName, Symbol ?durationSymbol) {
    act:displayNamedImageForDurationAtLocation(?imageName,?durationSymbol,current);
}

() = displayNamedImageForDurationAtLocation(String ?imageName, Symbol ?durationSymbol, Symbol ?location:location) {
    Boolean !imageShown;
    String !locationName;

    !locationName = op:getName(?location);
    if (~op:equals(!locationName,"current")) {
        act:goToLocation(?location);
    }

    !imageShown = act:displayImageWithName(?imageName,?durationSymbol);

    if (op:equals(!imageShown,false)) {
        exit(FAIL,couldNotShowImage(?imageName));
    }
}
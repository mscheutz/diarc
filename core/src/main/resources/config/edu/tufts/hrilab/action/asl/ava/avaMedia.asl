import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import java.lang.Boolean;
import java.lang.Long;
import java.lang.Integer;
import java.lang.String;
import java.util.List;
import java.util.Map;

() = showVideo(Symbol ?location = none) {
    onInterrupt :{
        cancel: tsc:cancelVideoPlayback();
        suspend: tsc:pauseVideoPlayback();
        resume: tsc:resumeVideoPlayback();
    }
    if (~op:equalsValue(?location, "none")) {
        act:goToLocation(?location);
    }
    tsc:displayNextVideo();
    while(tsc:isMediaPlaying()) {
        op:sleep(500);
    }
}

() = showVideoMatching(String ?matchString) {
    onInterrupt :{
        cancel: tsc:cancelVideoPlayback();
        suspend: tsc:pauseVideoPlayback();
        resume: tsc:resumeVideoPlayback();
    }
    tsc:displayVideoMatching(?matchString);
    while(tsc:isMediaPlaying()) {
        op:sleep(500);
    }
}

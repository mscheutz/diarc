import java.util.List;
import java.lang.String;
import edu.tufts.hrilab.cognex.consultant.CognexJob;
import edu.tufts.hrilab.cognex.consultant.CognexResult;
import edu.tufts.hrilab.cognex.consultant.CognexReference;

() = perceiveEntityFromSymbol["runs a job for a given pre-existing ?refId and binds the relevant result to that reference"](edu.tufts.hrilab.fol.Symbol ?refId) {
    List !cameraResults;
    String !jobName;
    CognexJob !job;
    CognexResult !result;
    CognexReference !ref;

    op:log("debug", "[perceiveEntityFromSymbol] for ?refId");
    (!ref) = tsc:getCognexReferenceForID(?refId);

    (!job) = tsc:getCognexJobForCognexReference(!ref);
    (!jobName) = op:invokeMethod(!job, "getName");
    (!cameraResults) = tsc:getCameraData(!jobName);
    if (op:isEmpty(!cameraResults)) {
        op:log("info","[perceiveEntityFromSymbol] failed to get cognex results");
    } else {
        (!result) = tsc:getMatchingResult(!ref,!cameraResults);
        //bind reference to the result that it matches
        tsc:bindCognexResult(!ref, !result, 0);
        op:log("info","[perceiveEntityFromSymbol] Bound !result to !ref");
    }
}


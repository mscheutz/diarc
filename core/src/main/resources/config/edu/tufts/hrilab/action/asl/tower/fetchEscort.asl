import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import java.util.List;

() = blobBehindTest() {
    op:log("info", "checking for blob behind");
    java.lang.Boolean !blobBehind;
    java.lang.Boolean !true;
    !true = op:newObject("java.lang.Boolean", true);
    !blobBehind = op:newObject("java.lang.Boolean", false);
    while (op:==(!true, !true)) {
        !blobBehind = act:blobBehind();
        if (op:==(!blobBehind, !true)) {
            op:log("info", "blob!!");
        }
    }
}

() = takeSpeakerTo(Symbol ?location) {
    java.lang.Long !twoSeconds = op:newObject("java.lang.Long", 3000);
    java.lang.Long !fiveSeconds = op:newObject("java.lang.Long", 5000);
    // Symbol !front_cam = "front:agent";
    // Symbol !back_right_cam = "back_right:agent";
    Symbol !back_left_cam = "back_left";

        // pre obs: stillFollowing(follower, !front_cam, !fiveSeconds);
        // overall obs: stillFollowing(follower, !back_right_cam, !fiveSeconds);
    conditions : {
        pre obs: stillFollowing(follower, "back_left");
        // or : {
        overall obs: stillFollowing(follower, !back_left_cam, ?location);
        // }
    }

    op:log("info", "starting takeSpeakerTo");
    act:goToLocation(?location);
}

(List ?return) = observeFollower(edu.tufts.hrilab.fol.Predicate ?predicate) {
    edu.tufts.hrilab.fol.Symbol !follower;
   
    java.lang.Long !wait = op:newObject("java.lang.Long", 2500);
    observes : stillFollowing(!follower, !cam, !loc);
    observes : stillFollowing(!follower, !cam);

    java.util.Map !bindings = op:newHashMap("edu.tufts.hrilab.fol.Variable", "edu.tufts.hrilab.fol.Symbol");
    ?return = op:newArrayList("java.util.Map");

    Symbol !person = "physobj_11:physobj";
    java.util.List !tokenIds;
    java.lang.Boolean !bool = op:newObject("java.lang.Boolean", true);
    java.lang.Boolean !true = op:newObject("java.lang.Boolean", true);
    java.lang.Long !startTime;
    java.lang.Long !currTime;
    java.lang.Long !comp;


    !cam.act:startVisualSearch(!person);
    !tokenIds = !cam.act:getTokenIds(!person);

    // wait !wait seconds
    !startTime = op:invokeStaticMethod("java.lang.System", "currentTimeMillis");
    !comp = op:invokeStaticMethod("java.lang.Long", "sum", !startTime, !wait);

    while (op:isEmpty(!tokenIds) && op:==(!bool, !true)) {
      !currTime = op:invokeStaticMethod("java.lang.System", "currentTimeMillis");
      
      if (op:gt(!currTime, !comp)) {
        !bool = op:newObject("java.lang.Boolean", false);
      }
      !tokenIds = !cam.act:getTokenIds(!person);
    }

    if (op:isEmpty(!tokenIds)) {
        op:log("info", "no person found, (hopefully) executing recovery behavior");
        act:stop();
        act:findFollower(!person, !loc);
        // return nothing
    } elseif (act:checkInRange(!cam, !person)) {
        op:add(?return, !bindings);
    } else {
        op:log("info", "person found, but not in following range");
        // return nothing
    }
    
}

() = findFollower(Symbol ?person, Symbol ?location) {
    java.util.List !tokenIds;
    java.lang.Long !waitTime = op:newObject("java.lang.Long", 7500); // 7.5 seconds
    java.lang.Long !startTime;
    java.lang.Long !currTime;
    java.lang.Long !comp;
    java.lang.Boolean !bool = op:newObject("java.lang.Boolean", true);
    java.lang.Boolean !true = op:newObject("java.lang.Boolean", true);
    java.lang.Boolean !false = op:newObject("java.lang.Boolean", false);
    java.lang.Boolean !inRange = op:newObject("java.lang.Boolean", false);

    Symbol !cam = "back_left";

    !cam.act:startVisualSearch(?person);
    (!tokenIds) = !cam.act:getTokenIds(?person);


    !startTime = op:invokeStaticMethod("java.lang.System", "currentTimeMillis");
    !comp = op:invokeStaticMethod("java.lang.Long", "sum", !startTime, !waitTime);
    
    // wait 7.5 seconds (ends early if person in range)
    while (op:==(!inRange, !false) && op:==(!bool, !true)) {
        !currTime = op:invokeStaticMethod("java.lang.System", "currentTimeMillis");
        
        if (op:gt(!currTime, !comp)) {
            !bool = op:newObject("java.lang.Boolean", false);
        }
        (!tokenIds) = !cam.act:getTokenIds(?person);
        !inRange = !cam.act:checkInRange(!cam, ?person);
    }
    if (op:==(!inRange, !true)) {
        act:takeSpeakerTo(?location);
    } else {
        op:log("info", "follower couldn't keep up... giving up");
    }
}

// TODO: fix this
// () = findFollower(Predicate ?failedActionSignature, List ?failureReasons, Predicate ?failedGoal) {
//     Symbol !person = "physobj_11:physobj";
//     java.util.List !tokenIds;
//     java.lang.Long !waitTime;
//     java.lang.Long !startTime;
//     java.lang.Long !currTime;
//     java.lang.Long !comp;
//     java.lang.Boolean !bool = op:newObject("java.lang.Boolean", true);
//     java.lang.Boolean !true = op:newObject("java.lang.Boolean", true);
//     java.lang.Boolean !false = op:newObject("java.lang.Boolean", false);
//     java.lang.Boolean !inRange = op:newObject("java.lang.Boolean", false);

//     Symbol !cam = "back_left";

//     recovery: {
//         goals: {takeSpeakerTo}
//         actionStatuses: {FAIL_RETURNVALUE, FAIL_OVERALLCONDITIONS}
//     }

//     !waitTime = op:newObject("java.lang.Long", 7500); // 7.5 seconds
//     !cam.act:startVisualSearch(!person);
//     (!tokenIds) = !cam.act:getTokenIds(!person);


//     !startTime = op:invokeStaticMethod("java.lang.System", "currentTimeMillis");
//     !comp = op:invokeStaticMethod("java.lang.Long", "sum", !startTime, !waitTime);
    
//     // wait 7.5 seconds (ends early if person in range)
//     while (op:==(!inRange, !false) && op:==(!bool, !true)) {
//         !currTime = op:invokeStaticMethod("java.lang.System", "currentTimeMillis");
        
//         if (op:gt(!currTime, !comp)) {
//             !bool = op:newObject("java.lang.Boolean", false);
//         }

//         !inRange = act:checkInRange(!cam, !person);
//     }
// }


(java.lang.Boolean ?inRange) = checkInRange(Symbol ?person) {
    ?inRange = op:newObject("java.lang.Boolean", false);
    java.lang.Double !dist;
    java.lang.Double !distThresh = op:newObject("java.lang.Double", 4.0);
    java.util.List !tokenIds;
    java.lang.Long !tokenId;
    edu.tufts.hrilab.vision.stm.MemoryObject !token;

    !tokenIds = act:getTokenIds(?person);
    if (op:isEmpty(!tokenIds)) { 
        op:log("warn", "person disappeared before range could be checked");
        ?inRange = op:newObject("java.lang.Boolean", false);
    } else {
        !tokenId = op:get(!tokenIds, 0);
        !token = act:getToken(!tokenId, .1);
        !dist = op:invokeMethod(!token, "getDist");
        op:log("info", "person found. Distance: ");
        op:log("info", !dist);
        if (op:le(!dist, !distThresh)) {
            ?inRange = op:newObject("java.lang.Boolean", true);
        }
    }
}

() = testDepthGetter() {
    Symbol !person = "physobj_11:physobj";
    java.lang.Double !dist;
    java.util.List !tokenIds;
    java.lang.Long !tokenId;
    edu.tufts.hrilab.vision.stm.MemoryObject !token;

    act:startVisualSearch(!person);
    !tokenIds = act:getTokenIds(!person);

    while (op:isEmpty(!tokenIds)) {
        !tokenIds = act:getTokenIds(!person);
    }

    op:log("info", "person found! Getting depth...");

    !tokenId = op:get(!tokenIds, 0);
    !token = act:getToken(!tokenId, .1);
    !dist = op:invokeMethod(!token, "getDist");
    op:log("info", "person found: ");
    op:log("info", !dist);
}
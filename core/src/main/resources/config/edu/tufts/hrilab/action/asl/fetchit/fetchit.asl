import edu.tufts.hrilab.fol.Symbol;

// IMPORTANT -- these actions should only be used for the fetch performance assessment test
// and should only be modified for reasons related to updating/fixing that test

() = assemble(Symbol ?object) {
  //act:fetch(object_1, location_1);
  act:fetch(physobj_1:physobj, location_1:location);
  act:fetch(physobj_2:physobj, location_2:location);
  act:fetch(physobj_3:physobj, location_3:location);
  act:fetch(physobj_4:physobj, location_4:location);
  act:fetch(physobj_5:physobj, location_5:location);
  act:deliver(?object, location_6:location);
}

() = fetch(Symbol ?objectRef, Symbol ?location) {
  edu.tufts.hrilab.fol.Symbol !arm ="arm";
  edu.tufts.hrilab.fol.Symbol !prepare ="prepare";
  edu.tufts.hrilab.fol.Symbol !location_0 ="location_0:location";
  edu.tufts.hrilab.fol.Symbol !physobj_0 ="physobj_0:physobj";

  act:approach(?location, !location_0);
  act:pickup(?objectRef, !arm);
  act:goToPose(!prepare);
  act:carry(?objectRef,!location_0,?location);
  act:placeIn(?objectRef,!physobj_0);
}

() = deliver(Symbol ?object, Symbol ?location) {
  edu.tufts.hrilab.fol.Symbol !arm ="arm";
  edu.tufts.hrilab.fol.Symbol !carry ="carry";
  edu.tufts.hrilab.fol.Symbol !dropOff ="dropOff";
  act:pickup(?object,!arm);
  act:goToPose(!carry);
  act:carry(?object, ?location);
  act:goToPose(!dropOff);
  act:placeOn(?object, ?location);
}

() = graspObject(Symbol ?objectRef, Symbol ?arm = "arm") {
    float !closePosition = 0.0f;

    effects : {
      success : grasping(?actor,?objectRef,?arm);
      success infer : touching(?actor,?objectRef);
    }

    act:openGripper(?arm);
    act:findObject(?objectRef);
    act:moveTo(?arm, ?objectRef);
    act:graspObject(?arm, ?objectRef, !closePosition);
    //act:stopAllSearches();
    op:log(debug, "[graspObject] successfully grasped ?objectRef");
}

() = releaseObject(Symbol ?objectRef, Symbol ?arm = "arm") {
    conditions : {
      pre : grasping(?actor,?objectRef,?arm);
    }
    effects : {
      success : not(grasping(?actor,?objectRef,?arm));
      success infer : not(touching(?actor,?objectRef));
      success infer : not(holding(?actor,?objectRef,?arm));
    }
    act:releaseObject(?arm, ?objectRef, 1.0);
    act:goToPose(release);
    act:goToPose(carry);
}

() = pickup(Symbol ?objectRef, Symbol ?arm="arm") {
    conditions : {
      pre : not(grasping(?actor,?objectRef,?arm));
    }
    effects : {
      success : holding(?actor,?objectRef,?arm);
    }
    act:graspObject(?objectRef, ?arm);
    act:moveObjectFetchItPrimitive(?objectRef, ?arm, up);
    op:log(debug, "[pickup] successfully picked up ?objectRef");
}

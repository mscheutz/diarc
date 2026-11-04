import edu.tufts.hrilab.fol.Symbol;

() = carry[""](Symbol ?objectRef, Symbol ?desiredLocation, Symbol ?initialLocation) {
  conditions : {
    pre : holding(?actor,?objectRef,arm);
  }
  effects : {
    success : holding(?actor,?objectRef,arm);
  }
  act:approach(?desiredLocation, ?initialLocation);
}

() = carry[""](Symbol ?objectRef, Symbol ?desiredLocation) {
  conditions : {
    pre : holding(?actor,?objectRef,arm);
  }
  effects : {
    success : holding(?actor,?objectRef,arm);
  }
  act:approach(?desiredLocation);
}

() = approach(Symbol ?desiredLocation) {
    act:stopAllSearches();
    act:approachLocation(?desiredLocation);
    act:look("down");
    op:sleep(2000);
}

() = approach(Symbol ?desiredLocation, Symbol ?initialLocation) {
    act:stopAllSearches();
    act:approachLocation(?desiredLocation, ?initialLocation);
    act:look("down");
    op:sleep(2000);
}

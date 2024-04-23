%location of stationary robots
agentAt(gofa, location_1).
agentAt(yumi, location_3).

%init location of mobile robot
agentAt(mobileyumi, location_2).
free(mobileyumi).
free(gofa).
free(yumi).

predicate(propertyof,var,property).

%todo:brad: should this be done dynamically ssomehwere?
constant(frenchfries, property).
constant(coke, property).
constant(hamburger, property).
constant(taco, property).

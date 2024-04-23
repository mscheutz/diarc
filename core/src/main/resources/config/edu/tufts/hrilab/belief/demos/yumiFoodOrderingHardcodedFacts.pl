%location of stationary robots
agentAt(left, location_1).
agentAt(right, location_1).

%init location of mobile robot
agentAt(human, location_0).
free(human).
free(left).
free(right).

predicate(propertyof,var,property).

%%todo:brad: should this be done dynamically ssomehwere?
%constant(frenchfries, property).
%constant(coke, property).
%constant(hamburger, property).
%constant(taco, property).

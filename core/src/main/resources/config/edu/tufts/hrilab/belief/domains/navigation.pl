%super: core

%%TYPES
subtype(base, agent).
subtype(waypoint, location).
subtype(physobj, physical).

%% PREDICATES
predicate(holding, agent, physobj).
predicate(at, physical, location).

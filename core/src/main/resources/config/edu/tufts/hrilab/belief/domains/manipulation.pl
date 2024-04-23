%%TYPES
subtype(actor, agent).
subtype(physobj, physical).
subtype(direction, concept).
subtype(distance, concept).

%% PREDICATES
predicate(holding, agent, physobj).
predicate(free, actor).
predicate(unknownlocation, actor).
predicate(near, actor, physobj).
predicate(above, physobj).
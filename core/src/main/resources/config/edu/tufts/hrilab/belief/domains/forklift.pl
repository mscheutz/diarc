%% Types
subtype(var, object).
subtype(concept, var).
subtype(physical, var).

subtype(location, concept).
subtype(physobj, physical).
subtype(agent, physical).

%% Functions
function(current_weight, location).
function(capacity, location).
function(slots, location).
function(distance, location, location).
function(weight, physobj).
function(cost).

%% Objects
predicate(property_of,object,property).

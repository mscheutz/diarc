subtype(concept, object).
subtype(physical, object).
subtype(location, concept).
subtype(physobj, physical).
subtype(agent, physical).
subtype(property, concept).
subtype(area, concept).
subtype(room, concept).

%agent semantic types
subtype(movebase, agent).
subtype(temi, agent).
subtype(fetch, movebase).
subtype(spot, agent).
subtype(human, agent).

subtype(movebaselocation,location).
subtype(temilocation,location).
subtype(spotlocation,location).

%TODO: for the purposes of the automate demo were not ging to make the arm/agent distinction.
%Its soemthing we will need to solve in geenral though.. I twillreuireq getting type checking working in other places
%subtype(place, location).
%subtype(pose, location).
%If an object is countable/there are more than one, needs to be declared as its own type

%TODO based on agents in the GM?
%object(fetch, fetch).
%object(temi, temi).

function(amount, area, property).
predicate(property_of,object,property).

%function([function_name], [type_1], [type_2])
%function(amount, place, property).

%derived(test(agent), free(agent)).





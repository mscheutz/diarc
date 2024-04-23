subtype(var, object).
subtype(concept, var).
subtype(physical, var).
subtype(location, concept).
subtype(method, concept).
subtype(physobj, physical).
subtype(agent, physical).
subtype(container, physobj).
subtype(container, place).
%TODO: for the purposes of the automate demo were not ging to make the arm/agent distinction.
%Its soemthing we will need to solve in geenral though.. I twillreuireq getting type checking working in other places
subtype(place, location).
subtype(pose, location).
%If an object is countable/there are more than one, needs to be declared as its own type
subtype(syringe,physobj).
subtype(bandage,physobj).
subtype(painkillers,physobj).
subtype(antiseptic,physobj).
subtype(pillBottle, physobj).
subtype(screwBox, physobj).
subtype(ioCard, physobj).
subtype(counter, concept).

%TODO based on agents in the GM?
object(robottwo, agent).
object(robotone, agent).
object(placeholder, physobj). %This is required due to annoying pddl stuff

%function([function_name], [type_1], [type_2])
function(amount, place, counter).

%From poc4
cameraHeight(230).

%derived(test(agent), free(agent)).

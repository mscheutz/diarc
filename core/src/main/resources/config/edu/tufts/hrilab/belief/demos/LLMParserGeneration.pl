subtype(var, object).
subtype(concept, var).
subtype(physical, var).
subtype(location, concept).
subtype(physobj, physical).
subtype(agent, physical).
subtype(property, concept).
subtype(room, concept).
subtype(container,physobj).
subtype(container, location).
%agent semantic types
subtype(movebase, agent).
subtype(fetch, movebase).
subtype(human, agent).
subtype(movebaselocation,location).

subtype(medicalcaddy, container).
subtype(bowl, container).
subtype(box, container).

subtype(painkiller, physobj).
subtype(bandagebox, physobj).
subtype(antiseptic, physobj).
subtype(apple, physobj).
subtype(baseball, physobj).
subtype(glassbottle, physobj).
subtype(carrot, physobj).
subtype(donut, physobj).
subtype(flowerpot, physobj).
subtype(computermouse, physobj).
subtype(car, physobj).
subtype(sportsbottle, physobj).
subtype(teddybear, physobj).
subtype(tennisball, physobj).
subtype(waterbottle, physobj).

constant(medicalcaddy, property).
constant(bowl, property).
constant(box, property).

constant(painkiller, property).
constant(bandagebox, property).
constant(antiseptic, property).
constant(apple, property).
constant(baseball, property).
constant(glassbottle, property).
constant(carrot, property).
constant(donut, property).
constant(flowerpot, property).
constant(computermouse, property).
constant(car, property).
constant(sportsbottle, property).
constant(teddybear, property).
constant(tennisball, property).
constant(waterbottle, property).

%TODO based on agents in the GM?
object(self, fetch).

%derived(test(agent), free(agent)).

predicate(violation).
type(X,agent).
type(Y,physobj).
type(Z,agent).

%ownershipViolation(X,Y,Z):-owned(Y,Z),carrying(X,Y),not(equals(X,Z)),type(X,agent),type(Y,physobj),type(Z,agent).
%colorViolation(X,Y):-blue(Y),carrying(X,Y),type(X,agent),type(Y,physobj).
%propertyofrecipe(X,Y) :- carrying(X,Y), type(X,agent), carrying(Y,physobj).

object(medkit, recipe).
subtype(recipe,concept).
constant(medkit, property).

%derived(medkitproperty(container),
% propertyof(container, 'medicalcaddy'),
% fluent_equals(amount(container, 'painkiller'), 1),
% fluent_equals(amount(container, 'bandagebox'), 1),
% fluent_equals(amount(container, 'antiseptic'), 1)
%).


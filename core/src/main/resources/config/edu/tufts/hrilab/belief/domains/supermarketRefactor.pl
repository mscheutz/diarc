%% TYPES and SUBTYPES -- OBJECT ONTOLOGY

subtype(agent, object).
subtype(location, object).
subtype(physobj, object).

subtype(food, physobj).
subtype(cart, physobj).
subtype(basket, physobj).
subtype(list,object).
subtype(cart, location).
subtype(basket, location).
subtype(shelf, location).
subtype(register, location).
subtype(counter, location).
subtype(cartReturn, location).
subtype(basketReturn, location).

%% FLUENT FUNCTION DEFINITIONS
function(amount, object, food).

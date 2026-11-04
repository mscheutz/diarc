%% TYPES and SUBTYPES -- OBJECT ONTOLOGY
subtype(physical, object).
subtype(agent, physical).
subtype(location, object).
subtype(direction, object).

subtype(cart, physical).
subtype(food, object).
subtype(shelf, physical).
subtype(aisle, object).
subtype(register, physical).
subtype(counter, physical).
subtype(cartReturn, physical).


%% FLUENT FUNCTION DEFINITIONS
function(specificFoodQuantity, cart, food).
function(generalFoodQuantity, cart).
function(capacity, cart).

empty(Cart):- fluent_equals(generalFoodQuantity, Cart, 0).
full(Cart):- fluent_same(generalFoodQuantity, Cart, capacity, Cart).

function(inventoryQuantity, agent, food).
function(shoppingListQuantity, agent, food).

predicate(purchasedInventory, agent).
predicate(violation).

%% Particular shelves, foods, etc.

%% Store aisles.
object(aisle1, aisle).
object(aisle2, aisle).
object(aisle3, aisle).
object(aisle4, aisle).
object(aisle5, aisle).
object(aisle6, aisle).

function(proximity, agent, agent).

object(cart0, cart).
%object(cart1, cart).
%object(cart2, cart).
%object(cart3, cart).

object(evan, agent).


violated_proximitynorm(X, Y) :- fluent_leq(proximity, X, Y, 1).
violated_norms(X) :- violated_proximitynorm(X, Y).

ownershipViolation(agent, physical) :- not(owned(physical,agent)), holding(agent, physical).
colorViolation(agent, physical) :- blue(physical), holding(agent, physical).

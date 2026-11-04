%% TYPES ONTOLOGY

% IMPORTANT: only assert subtype(x,y) facts, but only query for type(x,y)

% rules to make type hierarchy implicit
type(X,Y) :- constant(X,Y),X\=Y.
type(X,Y) :- object(X,Y),X\=Y.
type(X,Y) :- subtype(X,Y).
type(A,C) :- subtype(B,C),type(A,B).

% get "concrete" objects X of type Z
typeobject(X,Z) :- constant(X,Y), type(X,Z), not(type(W,X)).
typeobject(X,Z) :- object(X,Y), type(X,Z), not(type(W,X)).

subtype(concept, object).
subtype(physical, object).

subtype(physobj, physical).
subtype(agent, physical).
subtype(location, physical).
subtype(room, physical).
subtype(floor, physical).

subtype(door, physobj).

subtype(self, agent).

%% PREDICATE DEFINITIONS: predicate(predName, args...)
predicate(on_elevator, agent).
predicate(at, agent, location).
predicate(property_of, object, property).
predicate(goal, agent, location).

%% NUMERIC FLUENT FUNCTION DEFINITIONS: function(funcName, args...)
function(on_floor, physical). %% for locations and agents
function(target_floor, physobj). %% for elevator buttons

%% OBJECTS: object(obj, type)
object(door, physobj).

%% TODO: populate floors dynamically from map component?
object(loc_0,location).
object(loc_1,location).
object(loc_2,location).
object(loc_3,location).
object(loc_4,location).

%% TODO: populate references from vision component
object(obj_0,physobj).
object(obj_1,physobj).
object(obj_2,physobj).
object(obj_3,physobj).
object(obj_4,physobj).
object(obj_5,physobj).
object(obj_5,physobj).
object(obj_6,physobj).
object(obj_7,physobj).
object(obj_8,physobj).

%% CONSTANTS: constant(const, type)
constant(self, agent).

%% TODO: populate properties from vision component
constant(button_0, property).
constant(button_1, property).
constant(button_2, property).
constant(button_3, property).
constant(button_4, property).
constant(button_5, property).
constant(button_6, property).
constant(button_up, property).
constant(button_down, property).

%% location property
constant(elevator, property).

%% door property
constant(open, property).

%% ASSUMED INIT WORLD STATE: init(pred)

%% world state -- TODO: populate this dynamically
at(self,loc_4).
fluent_equals(on_floor(self),4).
goal(self, loc_1).

fluent_equals(on_floor(loc_0),3).
fluent_equals(on_floor(loc_1),3).
fluent_equals(on_floor(loc_2),3).
fluent_equals(on_floor(loc_3),4).
fluent_equals(on_floor(loc_4),4).

property_of(loc_0, elevator).
property_of(loc_3, elevator).

fluent_equals(target_floor(obj_0),0).
fluent_equals(target_floor(obj_1),1).
fluent_equals(target_floor(obj_2),2).
fluent_equals(target_floor(obj_3),3).
fluent_equals(target_floor(obj_4),4).
fluent_equals(target_floor(obj_5),5).
fluent_equals(target_floor(obj_6),6).

property_of(obj_0, button_0).
property_of(obj_1, button_1).
property_of(obj_2, button_2).
property_of(obj_3, button_3).
property_of(obj_4, button_4).
property_of(obj_5, button_5).
property_of(obj_6, button_6).
property_of(obj_7, button_up).
property_of(obj_8, button_down).
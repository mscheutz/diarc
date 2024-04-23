%% TYPES and SUBTYPES -- OBJECT ONTOLOGY
%%TYPES

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

subtype(coord, concept).
subtype(distance, concept).
subtype(property, concept).
subtype(action, concept).

% TODO: remove thses in favor of e.g., constant(unlocked,property)
subtype(unlocked, property).
subtype(interactable, property).

subtype(physobj, physical).
subtype(agent, physical).

subtype(self, agent).

subtype(placeable, physobj).
subtype(breakable, physobj).
subtype(rubber, physobj).
subtype(wall, physobj).
subtype(air, physobj).
subtype(plank, physobj).
subtype(stick, physobj).
subtype(pogo_stick, physobj).
subtype(tree_tap, physobj).
subtype(crafting_table, physobj).
subtype(log, physobj).

subtype(log, placeable).
subtype(plank, placeable).
subtype(crafting_table, placeable).
subtype(tree_tap, placeable).

subtype(log, breakable).
subtype(plank, breakable).
subtype(crafting_table, breakable).
subtype(tree_tap, breakable).

% TODO: figure out the subtypes for the phase 2 items (e.g., breakable, placeable, etc.)
subtype(safe, physobj).
subtype(key, physobj).
subtype(chest, physobj).
subtype(door, physobj).
subtype(open_door, physobj).
subtype(open_door, door).
subtype(iron_pickaxe, physobj).
subtype(block_of_diamond, physobj).
subtype(block_of_platinum, physobj).
subtype(block_of_titanium, physobj).
subtype(diamond_ore, physobj).
subtype(diamond, physobj).
subtype(sapling, physobj).

subtype(block_of_platinum, breakable).
subtype(diamond_ore, breakable).

subtype(sapling, placeable).

subtype(trader, agent).
subtype(pogoist, agent).
subtype(thief, agent).

subtype(oak_plank, plank).
subtype(oak_log, log).

%%PREDICATES

%%High level planning
%%Agent facing cardinal direction
%predicate(facing, agent, dir).
%%Agent is facing object at specified distance
predicate(facing_obj, agent, physical, distance).
predicate(next_to, physical, physical).
%%Agent is holding an object
predicate(holding, agent, physobj).
%% An object is floating and can be picked up, i.e. an object is an entity
predicate(floating, physobj).
predicate(propertyOf,physical,property).
%%predicate(explored,physical).

%%defining types of functions for exploration
changeable(fluent_increase(inventory(self,sapling),1.0)).
changeable(fluent_increase(inventory(self,sapling),2.0)).

%fluents that can be observed at all times
visiblefluent(inventory, self, object).
visiblefluent(world, none, object).

%identification of navigation actions for use in exploration policies
navigation(move).
navigation(teleport).

%%in other words, we should not try to update this action if something unexpected happens
unfixable(get_sapling).

%% NUMERIC FLUENT FUNCTION DEFINITIONS
function(inventory, agent, object).
function(container, physical, object).
function(world, object).
 %% add these back in for action cost planning
%function(cost_1, action).
%function(cost_2, action, physical).
%function(cost_3, action, physical, physical).
%function(totalcost). %% add this back in for action cost planning

constant(stick, stick).
constant(tree_tap, tree_tap).
constant(pogo_stick, pogo_stick).
constant(crafting_table, crafting_table).
constant(wall, wall).
constant(air, air).
constant(rubber, rubber).
constant(oak_plank, oak_plank).
constant(oak_log, oak_log).
constant(safe, safe).
constant(key, key).
constant(iron_pickaxe, iron_pickaxe).
constant(block_of_diamond, block_of_diamond).
constant(chest, chest).
constant(diamond_ore, diamond_ore).
constant(diamond, diamond).
constant(door, door).
constant(open_door,open_door).
constant(block_of_platinum, block_of_platinum).
constant(block_of_titanium, block_of_titanium).
constant(sapling, sapling).
constant(rival, agent).
constant(rival2, agent).

% TODO: figure out how to remove these without causing a novelty -- related to recipes
constant(plank, plank).
constant(log, log).

constant(self, agent).

% TODO: replace these with e.g., constant(unlocked,property)
constant(unlocked, unlocked).
constant(interactable, interactable).

% for facing_obj(agent, object, distance) distance values, so they aren't fluents
constant(one, distance).
constant(two, distance).

% define actions for use in cost() functions
constant(use, action).
constant(break_block, action).
constant(collect, action).
constant(select_item, action).
constant(place, action).
constant(craft, action).
constant(smooth_move, action).
constant(smooth_turn, action).
constant(sense_all, action).
constant(sense_recipes, action).
constant(sense_actor_actions, action).

% define default action costs
fluent_equals(cost_1(smooth_move),27.906975).
fluent_equals(cost_1(smooth_turn),24.0).

fluent_equals(cost_1(collect),1200.0).
fluent_equals(cost_2(collect,tree_tap),50000.0).
fluent_equals(cost_2(collect,safe),1200.0).
fluent_equals(cost_2(collect,chest),1200.0).

fluent_equals(cost_1(select_item),120.0).
fluent_equals(cost_2(select_item,tree_tap),120.0).
fluent_equals(cost_2(select_item,iron_pickaxe),120.0).
fluent_equals(cost_2(select_item,key),120.0).

fluent_equals(cost_2(use,safe),300.0).
fluent_equals(cost_2(use,door),300.0).

fluent_equals(cost_1(place),300.0).
fluent_equals(cost_2(place,oak_log),300.0).
fluent_equals(cost_2(place,tree_tap),300.0).
fluent_equals(cost_2(place,crafting_table),300.0).
fluent_equals(cost_2(place,sapling),300.0).

% TODO: add other break_block action costs -- break action cost might also depend on object being held
fluent_equals(cost_1(break_block),3600.0).
fluent_equals(cost_2(break_block,oak_log),3600.0).
fluent_equals(cost_2(break_block,tree_tap),3600.0).
fluent_equals(cost_2(break_block,crafting_table),3600.0).
fluent_equals(cost_2(break_block,diamond_ore),600.0).
fluent_equals(cost_2(break_block,block_of_platinum),600.0).

fluent_equals(cost_2(craft,pogo_stick),8400.0).
fluent_equals(cost_2(craft,stick),2400.0).
fluent_equals(cost_2(craft,oak_plank),1200.0).
fluent_equals(cost_2(craft,tree_tap),7200.0).
fluent_equals(cost_2(craft,block_of_diamond),10800.0).

fluent_equals(cost_1(sense_all),114.0).
fluent_equals(cost_1(sense_recipes),1200.0).
fluent_equals(cost_1(sense_actor_actions),6.0).

% recipe format: recipe(input_ingredient_slots[0-8],output_item,num_output_items)
recipe(log,0,0,0,0,0,0,0,0,plank,4).
recipe(plank,stick,plank,plank,0,plank,0,plank,0,tree_tap,1).
recipe(plank,0,0,plank,0,0,0,0,0,stick,4).
recipe(plank,plank,0,plank,plank,0,0,0,0,crafting_table,1).
recipe(diamond,diamond,diamond,diamond,diamond,diamond,diamond,diamond,diamond,block_of_diamond,1).
recipe(stick,block_of_titanium,stick,block_of_diamond,block_of_titanium,block_of_diamond,0,rubber,0,pogo_stick,1).

% trade recipes
trade(input(log,10),output(block_of_titanium,1)).
trade(input(diamond,18),output(block_of_platinum,1)).
trade(input(block_of_platinum,2),output(diamond,9)).
trade(input(block_of_platinum,1),output(block_of_titanium,1)).

%always true in the world at the beginning of each episode
init(fluent_equals(world(chest), 1)).
init(fluent_equals(world(oak_log), 5)).
init(fluent_equals(world(crafting_table), 1)).
init(fluent_equals(inventory(self, iron_pickaxe), 1)).
init(fluent_equals(container(safe, diamond), 18)).
init(fluent_equals(container(chest, key), 1)).

% numberOfType is not included in ppdl -- only for checking for novelties
init(fluent_equals(numberOfType(trader), 2)).
init(fluent_equals(numberOfType(pogoist), 1)).

% not used to check for novelties -- (to be) used for planning with action cost
init(fluent_equals(totalcost, 0.0)).

% for keeping track of which rooms have been explored
uncharted(X,Y) :- entryway(X,Y), not(accessed(X,Y)).

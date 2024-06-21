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

subtype(property, concept).

subtype(physobj, physical).
subtype(agent, physical).

object(self, agent).

%% init world state -- TODO: observe this
clear(physobj_0).
clear(physobj_1).
clear(physobj_2).
clear(physobj_3).
ontable(physobj_0).
ontable(physobj_1).
ontable(physobj_2).
ontable(physobj_3).
%on(physobj_2,physobj_1).
handempty(self).
%%TYPES
subtype(var, object).
subtype(concept, var).
subtype(physical, var).
subtype(physobj, physical).
subtype(location, concept).
%subtype(location, concept).
subtype(agent, physical).
subtype(property, concept).

object(person, location).
object(table, location).
object(physobj_0, physobj).
object(physobj_1, physobj).
object(physobj_2, physobj).
object(physobj_3, physobj).
object(bottle, property).
object(apple, property).
object(free, property).
property_of(physobj_0, bottle).
property_of(physobj_1, bottle).
property_of(physobj_2, apple).
property_of(physobj_3, apple).
observableat(bottle,table).
observableat(apple,table).
available(physobj_0).
available(physobj_1).
available(physobj_2).
available(physobj_3).
at(self,person).
free(self).

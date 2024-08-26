%%TYPES
subtype(var, object).
subtype(concept, var).
subtype(physical, var).
subtype(physobj, physical).
subtype(location, concept).
%subtype(location, concept).
subtype(agent, physical).
subtype(property, concept).

object(location_0, location).
object(location_1, location).

object(physobj_0, physobj).
object(physobj_1, physobj).
object(physobj_2, physobj).
object(physobj_3, physobj).
object(bottle, property).
object(apple, property).
property_of(physobj_0, bottle).
property_of(physobj_1, bottle).
property_of(physobj_2, apple).
property_of(physobj_3, apple).

observableat(bottle,location_1).
observableat(apple,location_1).
at(self,location_0).
free(self).

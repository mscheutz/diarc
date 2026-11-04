

%% agents the system should know about
name(self,self).
diarcAgent(self).
memberOf(X,X).
memberOf(self, self).
team(self).
name(self, spot).
name(spot, spot).

actor(amitis).
diarcAgent(spot).
object(self, agent).
object(spot, agent).
subtype(cup, physobj).

memberOf(X,X).
memberOf(self, self).
team(self).
cup(cup1).


subtype(concept, object).
subtype(physical, object).
subtype(location, concept).
subtype(physobj, physical).
subtype(agent, physical).
subtype(property, concept).
subtype(area, concept).
subtype(room, concept).
at(cup(cup1),spotlocation_1).




subtype(human, agent).
subtype(spotlocation,location).
subtype(place, location).
subtype(pose, location).
ree(spot).

actorAt(spot,spotlocation_0).
actorAt(self,spotlocation_0).
at(spot,spotlocation_0).
at(self,spotlocation_0).

actorAt(human,spotlocation_2).
not(perceptionpose(self)).

accessibleBy(spot,spotlocation_0).
accessibleBy(spot,spotlocation_1).
accessibleBy(spot,spotlocation_2).
accessibleBy(spot,spotlocation_3).
accessibleBy(spot,spotlocation_4).
accessibleBy(spot,spotlocation_5).
accessibleBy(spot,spotlocation_6).
accessibleBy(spot,spotlocation_7).

actorInRoom(spot,room1).

locationInRoom(spotlocation_0, room1).
locationInRoom(spotlocation_1, room1).
locationInRoom(spotlocation_2, room1).
locationInRoom(spotlocation_3, room2).
locationInRoom(spotlocation_4, room1).
locationInRoom(spotlocation_5, room3).
locationInRoom(spotlocation_6, room2).
locationInRoom(spotlocation_7, room1).
doorBetweenRooms(spotlocation_2, room1, room2).
doorBetweenRooms(spotlocation_4, room1, room3).


admin(admin).
supervisor(amitis).
admin(amitis).


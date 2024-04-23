free(spot).

actorAt(spot,spotlocation_0).

areaBoundToLocation(tableG, spotlocation_1).
areaBoundToLocation(pharmacy, spotlocation_5).

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

object(pharmacy, area). %only accessible by the spot

%todo: (pete) remove this. hadcoded addl information to test replanning without recovery or extended dialogue with multiple places for stuff.
observableAt(bandagebox,pharmacy).
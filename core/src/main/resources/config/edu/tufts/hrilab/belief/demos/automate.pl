subtype(var, object).
subtype(concept, var).
subtype(physical, var).
subtype(location, concept).
subtype(method, concept).
subtype(physobj, physical).
subtype(agent, physical).
subtype(container, physobj).
subtype(arm, agent).
subtype(place, location).
subtype(pose, location).
subtype(smallobj, physobj).
subtype(syringe,smallobj).
subtype(bandage,smallobj).
subtype(painkillers,smallobj).
subtype(antiseptic,smallobj).
subtype(counter, concept).

object(armtwo, arm).
object(armone, arm).

object(sharedloc, place).
object(armoneplace, place).
object(armtwoplace, place).

object(pose_2, pose).
object(pose_3, pose).
object(pose_0, pose).
object(pose_1, pose).

object(medkit1, container).
object(syringe1, syringe).
object(syringe2, syringe).
object(syringe3, syringe).
object(bandage1, bandage).
object(bandage2, bandage).
object(antiseptic1, antiseptic).
object(painkillers1, antiseptic).

oftype(syringe1, syringecounter).
oftype(syringe2,syringecounter).
oftype(syringe3,syringecounter).
oftype(bandage1,bandagecounter).
oftype(bandage2,bandagecounter).
oftype(antiseptic1,antisepticcounter).
oftype(painkillers1,painkillerscounter).

free(armtwo).
free(armone).

at(armone, pose_0).
at(armtwo, pose_1).

at(medkit1, sharedloc).
at(syringe1, armoneplace).
at(syringe2, armoneplace).
at(syringe3, armoneplace).
at(bandage1, armoneplace).
at(bandage2, armoneplace).
at(painkillers1, armtwoplace).
at(antiseptic1, armtwoplace).

%available
available(pose_2).
available(pose_3).
available(sharedloc).

%accessible
accessible(armone, pose_0).
accessible(armone, pose_2).
accessible(armtwo, pose_1).
accessible(armtwo, pose_3).
accessible(armone, sharedloc).
accessible(armtwo, sharedloc).

%above
above(pose_2, sharedloc).
above(pose_3, sharedloc).
above(pose_0, armoneplace).
above(pose_1, armtwoplace).

function(amount, container, counter).

%These can be auto generated
object(syringecounter, counter).
object(antisepticcounter, counter).
object(bandagecounter, counter).
object(painkillerscounter, counter).

%Init counters
fluent_equals(amount, medkit1, syringecounter, 0).
fluent_equals(amount, medkit1, antisepticcounter, 0).
fluent_equals(amount, medkit1, bandagecounter, 0).
fluent_equals(amount, medkit1, painkillerscounter, 0).
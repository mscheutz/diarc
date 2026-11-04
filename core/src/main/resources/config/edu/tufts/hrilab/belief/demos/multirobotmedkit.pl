subtype(var, object).
subtype(concept, var).
subtype(physical, var).
subtype(location, concept).
subtype(method, concept).
subtype(physobj, physical).
subtype(actor, physical).
subtype(container, physobj).
subtype(base, actor).
subtype(arm, actor).
subtype(conveyor, actor).
subtype(place, location).
subtype(pose, location).


object(robotone, base).
object(robottwo, base).
object(armtwo, arm).
object(armone, arm).
object(conveyor, conveyor).
object(conveyorstart, place).
object(armoneconveyorpose, pose).
object(conveyorend, place).
object(conveyorpickup, pose).
object(armoneplatform, place).
object(armoneloading_areapose, pose).
object(loading_area, place).
object(dropoff, pose).
object(t1, place).
object(lab, place).
object(alpha, place).
object(l2, place).
object(medkit1, container).
%object(medkit2, container).
object(syringe, physobj).
object(bandage, physobj).
object(antiseptic, physobj).
object(painkillers, physobj).

above(armoneconveyorpose, conveyorstart).
above(conveyorpickup, conveyorend).
above(dropoff, loading_area).
above(armoneloading_areapose, armoneplatform).

free(armtwo).
free(armone).
free(robotone).
free(robottwo).

at(robotone, t1).
at(robottwo, lab).
at(armtwo, conveyorpickup).
at(armone, armoneconveyorpose).

at(medkit1, conveyorstart).
%at(medkit2, conveyorstart).
at(syringe, armoneplatform).
at(bandage, armoneplatform).
at(painkillers, armoneplatform).
at(antiseptic, armoneplatform).

available(alpha).
available(l2).
available(dropoff).
available(armoneloading_areapose).
available(loading_area).

accessible(conveyor, conveyorstart).
accessible(conveyor, conveyorend).

accessible(armtwo, conveyorend).
accessible(armtwo, dropoff).
accessible(armtwo, conveyorpickup).

accessible(armone, conveyorstart).
accessible(armone, armoneloading_areapose).
accessible(armone, armoneconveyorpose).

accessible(robotone, t1).
accessible(robottwo, t1).
accessible(robotone, lab).
accessible(robottwo, lab).
accessible(robotone, alpha).
accessible(robottwo, alpha).
accessible(robotone, l2).
accessible(robottwo, l2).
accessible(robotone, loading_area).
accessible(robottwo, loading_area).
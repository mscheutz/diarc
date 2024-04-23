subtype(var, object).
subtype(concept, var).
subtype(physical, var).
subtype(location, concept).
subtype(area, concept).
subtype(agent, physical).
subtype(id, concept).
subtype(locname, concept).
subtype(direction, concept).
subtype(base, agent).
subtype(station, agent).

object(right,direction).
object(left,direction).
object(one,id).
object(two,id).
object(three,id).
object(four,id).
object(five,id).
object(six,id).
object(seven,id).
object(eight,id).
object(nine,id).
object(ten,id).
object(eleven,id).
object(twelve,id).
object(robotone, agent).
object(robottwo, agent).
object(station, station).

%%move to init

object(alpha, area).
object(beta, area).
object(gamma, area).
object(start, area).

adjacent(start, alpha).
adjacent(start, beta).
adjacent(start, gamma).
adjacent(alpha, start).
adjacent(beta, start).
adjacent(gamma, start).

sealed(start).
sealed(alpha).
sealed(beta).
sealed(gamma).
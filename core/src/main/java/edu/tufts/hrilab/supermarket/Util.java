package edu.tufts.hrilab.supermarket;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;

import static edu.tufts.hrilab.supermarket.SupermarketObservation.Direction.*;

public class Util {

  private static final Logger log = LoggerFactory.getLogger(Util.class);

  static boolean projectCollision(SupermarketObservation.Direction dir, SupermarketObservation obs, SupermarketObservation.InteractiveObject obj) {
    double STEP = 0.4;
    double x = obj.position[0];
    double y = obj.position[1];
    switch (dir) {
      case NORTH:
        y -= STEP;
        break;
      case SOUTH:
        y += STEP;
        break;
      case EAST:
        x += STEP;
        break;
      case WEST:
        x -= STEP;
        break;
      default:
        log.error("Unknown direction: " + dir);
        return true;
    }
    return projectCollision(obs, obj, x, y);
  }

  private static boolean projectCollision(SupermarketObservation obs, SupermarketObservation.InteractiveObject obj, double x, double y) {
    return projectCollision(obs.basketReturns, obj, x, y) ||
//            projectCollision(obs.baskets, obj, x, y) ||
            projectCollision(obs.cartReturns, obj, x, y) ||
//            projectCollision(obs.carts, obj, x, y) ||
            projectCollision(obs.counters, obj, x, y) ||
            projectCollision(obs.registers, obj, x, y) ||
            projectCollision(obs.shelves, obj, x, y) ||
            checkBounds(obj, x, y);
//            project_collision(obs.players, obj) && //todo: remove self
    //todo: Add boundaries
  }

  private static boolean projectCollision(SupermarketObservation.InteractiveObject[] objects, SupermarketObservation.InteractiveObject test, double x, double y) {
    return Arrays.stream(objects).anyMatch(o -> o.collision(test, x, y));
  }

  private static boolean checkBounds(SupermarketObservation.InteractiveObject test, double x, double y) {
    return (x < 0.5 || y < 2.5 || x + test.width > 24 || y + test.height > 19);
  }

  public boolean aligned(double x1, double span1, double x2, double span2) {
    return x1 < x2 + span2 && x2 < x1 + span1;
  }

}



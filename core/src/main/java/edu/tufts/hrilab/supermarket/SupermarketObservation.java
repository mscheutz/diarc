/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.supermarket;

import edu.tufts.hrilab.diarc.DiarcComponent;
import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SupermarketObservation {

    public Player[] players;
    public Cart[] carts;
    public Basket[] baskets;
    public Shelf[] shelves;
    public Counter[] counters;
    public Register[] registers;
    public CartReturn[] cartReturns;
    public BasketReturn[] basketReturns;
    public int interactive_stage;
    public int total_stages;
    private static Logger log = LoggerFactory.getLogger(DiarcComponent.class);

    private static double aisle_offset = 2.56;
    private static double aisle_height = 2.4;

    public static boolean overlap(double x1, double y1, double width1, double height1,
                                  double x2, double y2, double width2, double height2) {
        return !(x1 > x2 + width2 || x2 > x1 + width1 || y1 > y2 + height2 || y2 > y1 + height1);
    }

    @Override
    public boolean equals(Object other) {
        if (other == null) { return false; }
        if (other == this) { return true; }
        if (other.getClass() != getClass()) {
            return false;
        }
        SupermarketObservation obs = (SupermarketObservation)other;
        return new EqualsBuilder().append(this.players, obs.players)
                .append(this.carts, obs.carts)
                .append(this.baskets, obs.baskets)
                .append(this.shelves, obs.shelves)
                .append(this.counters, obs.counters)
                .append(this.registers, obs.registers)
                .append(this.cartReturns, obs.cartReturns)
                .append(this.basketReturns, obs.basketReturns)
                .append(this.interactive_stage, obs.interactive_stage)
                .append(this.total_stages, obs.total_stages)
                .isEquals();
    }

    @Override
    public int hashCode() {
        return new HashCodeBuilder().append(this.players)
                .append(this.carts)
                .append(this.baskets)
                .append(this.shelves)
                .append(this.counters)
                .append(this.registers)
                .append(this.cartReturns)
                .append(this.basketReturns)
                .append(this.interactive_stage)
                .append(this.total_stages).hashCode();
    }

    public static boolean defaultCanInteract(InteractiveObject obj, Player player, double range) {
        double x = player.position[0];
        double y = player.position[1];
        switch(player.direction) {
            case 0: // North
//                Player is facing north, and a spot just above the player collides with the object. etc.
                return obj.collision(player, x, y-range);
            case 1: // South
                return obj.collision(player, x, y+range);
            case 2: // East
                return obj.collision(player, x + range, y);
            case 3: // West
                return obj.collision(player, x - range, y);
        }
        return false;
    }

    public static boolean defaultCanInteract(InteractiveObject obj, Player player) {
        return defaultCanInteract(obj, player, 0.5);
    }

    public abstract class InteractiveObject {
        public double width;
        public double height;

        public double[] position;

        public boolean collision(InteractiveObject other, double x, double y) {
            return overlap(this.position[0], this.position[1], this.width, this.height,
                    x, y, other.width, other.height);
        }

        public boolean collision(InteractiveObject other) {
            return collision(other, other.position[0], other.position[1]);
        }

        public abstract boolean canInteract(Player player);

        @Override
        public boolean equals(Object obj) {
            return EqualsBuilder.reflectionEquals(this, obj);
        }

        @Override
        public int hashCode() {
            return HashCodeBuilder.reflectionHashCode(this);
        }
    }

    public class Shelf extends InteractiveObject {
        public String food_name;
        public String food_image;
        public String shelf_image;
        public int price;
        public int quantity;
        public int capacity;

        @Override
        public boolean canInteract(Player player) {
            return player.direction <= 1 && defaultCanInteract(this, player);
        }
    }

    public class Register extends InteractiveObject {
        public int capacity;
        public String image;
        public String[] food_images;
        public String[] foods;
        public int[] food_quantities;
        public int num_items;
        public int curr_player;

        @Override
        public boolean canInteract(Player player) {
            return defaultCanInteract(this, player);
        }
    }

    public class Counter extends InteractiveObject {
        public String food;

        @Override
        public boolean canInteract(Player player) {
            return defaultCanInteract(this, player);
        }
    }

    public class CartReturn extends InteractiveObject {
        public int quantity;

        @Override
        public boolean canInteract(Player player) {
            double range = player.curr_cart >= 0 ? 1.5 : 0.5;
            return player.direction == 1 && defaultCanInteract(this, player, range);
        }
    }

    public class Player extends InteractiveObject {
        public int index;
        public int direction; // NORTH is 0, SOUTH is 1, EAST is 2, WEST is 3

        // This is the *index* of the current cart/basket in the carts/basket array.
        public int curr_cart;
        public int curr_basket;

        public String sprite_path;

        public String[] shopping_list;
        public int[] list_quant;

        public String[] bagged_items;
        public int[] bagged_quant;

        public boolean left_store;

        public String holding_food;
        public boolean bought_holding_food;

        public double budget;

        @Override
        public boolean canInteract(Player player) {
            return false;
        }
    }

    public class Cart extends InteractiveObject {
        public int direction;

        public int owner;
        public int last_held;


        public int capacity;

        public String[] contents;
        public int[] contents_quant;
        public String[] purchased_contents;
        public int[] purchased_quant;


//        NOTE that the cart's height and width change depending on its direction.
//        when direction is NORTH or SOUTH, (width, height) = (0.5, 0.75).
//        when direction is EAST or WEST, (width, height) = (0.75, 0.4).
//        This is hacky, but makes the collisions work better than the alternative.

//        Its position also shifts when the player turns so that the player's always holding it.

        @Override
        public boolean canInteract(Player player) {
            return defaultCanInteract(this, player);
        }
    }

    public class Basket extends Cart {
        public boolean being_held;
    }

    public class BasketReturn extends CartReturn {
    }

/* CART RETURN HELPERS */
//    The functions northOfCartReturn, southOfCartReturn, and atCartReturn are technically referring not to the cart
//    return itself, but to a small square directly to the north of the cart return.  The idea is that if you're in
//    the aisle hub at a point where neither northOfCartReturn nor southOfCartReturn is true, then you can go due west
//    until you're atCartReturn, and then turn south and pick up a cart.

    public boolean northOfCartReturn(int playerIndex) {
        return this.players[playerIndex].position[1] < 17.75;
    }

    public boolean southOfCartReturn(int playerIndex) {
        return this.players[playerIndex].position[1] > 18.05;
    }

    public boolean atCartReturn(int playerIndex) {
        double x = this.players[playerIndex].position[0];
        double y = this.players[playerIndex].position[1];
        return y >= 17.75 && y <= 18.05 && x >= 0.5 && x <= 2.5;
    }


//    Whether you're in patch of floor directly south of a particular shelf.
    public boolean atShelf(Player player, InteractiveObject shelf) {
        return player.position[0] > shelf.position[0] && player.position[0] < shelf.position[0] + shelf.width
                && player.position[1] > shelf.position[1] && player.position[1] < shelf.position[1] + 4;
    }

    public boolean inEntrance(int playerindex) {
        return this.players[playerindex].position[0] < 3.75;
    }

/* AISLE HUB HELPERS */
//    The "aisle hub" (for lack of a better term) is just a patch of floor between the aisles and the cart return/
//    checkout. While you're in this patch, you won't run into any aisle, the checkout, or the cart return. It's thus
//    a convenient hub for a player to move while navigating from one aisle to another or to the cart return/checkout.
    public boolean inAisleHub(int playerIndex) {
        return this.players[playerIndex].position[0] >= 3.75
                && this.players[playerIndex].position[0] <= 4.75;
    }

//    The "rear aisle hub" is another patch of floor, but at the back of the store (so it can be used to travel between
//    aisles and the food counters
    public boolean inRearAisleHub(int playerIndex) {
        return this.players[playerIndex].position[0] >= 15.5
                && this.players[playerIndex].position[0] <= 16.25;
    }


//    This just determines whether the player's in the aisleIndex-th aisle (aisles here are one-indexed; sorry).
    public boolean inAisle(int playerIndex, int aisleIndex) {
        double x = this.players[playerIndex].position[0];
        double y = this.players[playerIndex].position[1];
        double aisleLeft = 5.5;
        double aisleRight = 14.5;
        double aisleTop = aisle_offset + 4.*(aisleIndex-1);
        double aisleBottom = aisleTop + aisle_height;
        return y > aisleTop && y < aisleBottom
                && x >= aisleLeft && x <= aisleRight;
    }

/* EXIT ROW HELPERS */
//    I've arbitrarily picked one of the exit doors here, and northOfExitRow/southOfExitRow determine where you stand
//    with respect to that door. If you get to a position where neither northOfExitRow nor southOfExitRow is true and
//    then go straight west, you should exit the store.
    public boolean northOfExitRow(Player player) {
        return player.position[1] <= 7;
    }

    public boolean southOfExitRow(Player player) {
        return player.position[1] >= 7.8;
    }

//    This one speaks for itself.
    public boolean inStore(Player player) {
        return player.position[0] >= 0;
    }


//    The goal of these is to direct the player into a particular strip of floor which (1) is south of
//    the aisle in question, and (2) from which you can safely turn your cart south without bashing into a shelf.
//    Thus, it's possible that you'll be technically within a particular aisle, and one of northOfAisle and southOfAisle
//    will still be true.
    public boolean southOfAisle(int playerIndex, int aisleIndex) {
        double y = this.players[playerIndex].position[1];
        double aisleBottom = 0.5 + 4.*aisleIndex - 1.5;
        return y >= aisleBottom;
    }

    public boolean northOfAisle(int playerIndex, int aisleIndex) {
        double y = this.players[playerIndex].position[1];
        double aisleTop = 0.5 + 4.*aisleIndex - 2;
        return y <= aisleTop;
    }

    public boolean westOf(Player player, InteractiveObject obj) {
        return player.position[0] <= obj.position[0];
    }

    public boolean eastOf(Player player, InteractiveObject obj) {
        return player.position[0] + player.width >= obj.position[0] + obj.width;
    }

    public boolean northOf(Player player, InteractiveObject obj) {
        return player.position[1] <= obj.position[1];
    }

    public boolean southOf(Player player, InteractiveObject obj) {
        return player.position[1] + player.height >= obj.position[1] + obj.height;
    }

    public double proximity(int index1, int index2) {
        Player player1 = this.players[index1];
        Player player2 = this.players[index2];
        return Math.hypot(Math.abs(player1.position[0]-player2.position[0]),
                Math.abs(player1.position[1]-player2.position[1]));
    }
}

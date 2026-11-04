// Action scripts for the supermarket environment

() = initializeagent[""]() {
    edu.tufts.hrilab.fol.Variable !x;
    edu.tufts.hrilab.fol.Predicate !pred;

    obs:inAisleHub(?actor);
    obs:inRearAisleHub(?actor);
    obs:inEntrance(?actor);
    obs:northOfCartReturn(?actor);
    obs:southOfCartReturn(?actor);
    obs:atCartReturn(?actor);
    obs:northOfExitRow(?actor);
    obs:southOfExitRow(?actor);
    obs:inStore(?actor);

    obs:inAisle(?actor,aisle1:aisle);
    obs:inAisle(?actor,aisle2:aisle);
    obs:inAisle(?actor,aisle3:aisle);
    obs:inAisle(?actor,aisle4:aisle);
    obs:inAisle(?actor,aisle5:aisle);
    obs:inAisle(?actor,aisle6:aisle);


    for (!i=1; !i lt 30; !i++) {
      !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "atShelf(?actor,shelf!i:shelf)");
      obs:!pred;
    }

    op:log(info, "Finished observing for ?actor");

}


// NAVIGATION
() = leaveaisle["?actor leaves ?aisle"](edu.tufts.hrilab.fol.Symbol ?aisle:aisle, edu.tufts.hrilab.fol.Symbol ?shelf:shelf){
    java.lang.Boolean !val;
    edu.tufts.hrilab.fol.Predicate !query;

    conditions : {
        pre obs : inAisle(?actor, ?aisle);
        pre obs : atShelf(?actor, ?shelf);
        pre infer : shelfInAisle(?shelf, ?aisle);
    }
    effects : {
        success obs : not(inAisle(?actor, ?aisle));
        success obs: inAisleHub(?actor);
        success obs : not(atShelf(?actor, ?shelf));
    }
    op: log(info, ">> leaving aisle");
    // TODO: actually implement this
    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "inAisleHub(?actor)");

    while(~obs:!query) {
        act:goWest();
    }
}

() = leaveaisletorear["?actor leaves ?aisle to the rear"](edu.tufts.hrilab.fol.Symbol ?aisle:aisle, edu.tufts.hrilab.fol.Symbol ?shelf:shelf) {
    java.lang.Boolean !val;
    edu.tufts.hrilab.fol.Predicate !query;

    conditions : {
        pre obs : inAisle(?actor, ?aisle);
        pre obs : atShelf(?actor, ?shelf);
    }
    effects : {
        success obs : not(inAisle(?actor, ?aisle));
        success obs : inRearAisleHub(?actor);
        success obs : not(atShelf(?actor, ?shelf));
    }
    op: log(info, ">> leaving aisle to rear");
    // TODO: actually implement this

    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "inRearAisleHub(?actor)");

    while(~obs:!query) {
        act:goEast();
    }
}

() = enteraisle["?actor enters ?aisle"](edu.tufts.hrilab.fol.Symbol ?aisle:aisle, edu.tufts.hrilab.fol.Symbol ?shelf:shelf){

    java.lang.Boolean !val;
    edu.tufts.hrilab.fol.Predicate !query;
    edu.tufts.hrilab.fol.Predicate !northQuery;
    edu.tufts.hrilab.fol.Predicate !southQuery;

    conditions : {
        pre obs : inAisleHub(?actor);
        pre infer : firstShelf(?aisle, ?shelf);
    }
    effects : {
        success obs : not(inAisleHub(?actor));
        success obs : inAisle(?actor, ?aisle);
        success obs : atShelf(?actor, ?shelf);
    }
    op: log(info, ">> going to aisle ?aisle");

    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "inAisle(?actor, ?aisle)");
    !northQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "northOfAisle(?actor, ?aisle)");
    !southQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "southOfAisle(?actor, ?aisle)");

    while(~obs:!query) {
        if(obs:!northQuery) {
            act:goSouth();
        } elseif(obs:!southQuery) {
            act:goNorth();
        } else {
            act:goEast();
        }
    }
}

// Dummy action: actually, going to the counter doesn't cause the agent to leave the rear aisle hub, but we need
//  this for planning.
() = countertorearaislehub["?actor returns to rear aisle hub"](edu.tufts.hrilab.fol.Symbol ?counter:counter) {
    conditions : {
        pre infer : atCounter(?actor, ?counter);
    }
    effects : {
        success infer : not(atCounter(?actor, ?counter));
        success infer : inRearAisleHub(?actor);
    }
    op:log(info,"countertorearaislehub");
}

() = gotocounter["?actor goes to counter ?counter"](edu.tufts.hrilab.fol.Symbol ?counter:counter) {
    java.lang.Boolean !val;
    edu.tufts.hrilab.fol.Predicate !query;
    edu.tufts.hrilab.fol.Predicate !northQuery;
    edu.tufts.hrilab.fol.Predicate !southQuery;
    conditions : {
        pre obs : inRearAisleHub(?actor);
    }
    effects : {
        success infer : not(inRearAisleHub(?actor));
        success infer : atCounter(?actor, ?counter);
    }
    op: log(info, ">> going to counter ?counter");
    // Go south if you're too far north
    !northQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "northOf(?actor, ?counter)");
    !southQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "southOf(?actor, ?counter)");

    while(obs:!northQuery) {
        act:goSouth();
    }

    // Go north if you're too far south
    while(obs:!southQuery) {
        act:goNorth();
    }
}

() = getfoodfromcounter["?actor gets food from ?counter"](edu.tufts.hrilab.fol.Symbol ?food:food, edu.tufts.hrilab.fol.Symbol ?counter:counter, edu.tufts.hrilab.fol.Symbol ?cart:cart) {

    java.lang.Boolean !val;
    edu.tufts.hrilab.fol.Predicate !query;

    conditions : {
        pre infer : atCounter(?actor, ?counter);
        pre infer : holdingCart(?actor, ?cart);
        pre infer : stockedAt(?food, ?counter);
    }

    effects : {
        success infer : fluent_increase(specificFoodQuantity(?cart, ?food), 1);
        success infer : fluent_increase(generalFoodQuantity(?cart), 1);
        success infer : fluent_increase(inventoryQuantity(?actor, ?food), 1);
        success infer : not(purchasedInventory(?actor));
    }

    op:log(info, "getting food from counter");

    act:goEast();
    act:goEast();
    act:goEast();
    act:goEast();
    act:goEast();
    act:goEast();
    act:goWest();

    act:toggleShoppingCart();
    while(~obs:canInteractWith(?actor, ?counter)) {
        act:goEast();
    }

    act:interactWithObject();
    act:interactWithObject();
    act:interactWithObject();

    while(~obs:canInteractWith(?actor, ?cart)) {
        act:goWest();
    }

    act:interactWithObject();
    act:interactWithObject();
    act:goWest();
    act:goWest();
    act:goWest();
    act:goWest();
    act:toggleShoppingCart();
}

() = enteraislefromrear["?actor enters ?aisle from rear"](edu.tufts.hrilab.fol.Symbol ?aisle:aisle, edu.tufts.hrilab.fol.Symbol ?shelf:shelf) {
    java.lang.Boolean !val;
    edu.tufts.hrilab.fol.Predicate !query;
    edu.tufts.hrilab.fol.Predicate !northQuery;
    edu.tufts.hrilab.fol.Predicate !southQuery;

    conditions : {
        pre obs : inRearAisleHub(?actor);
        pre infer : lastShelf(?aisle, ?shelf);
    }
    effects : {
        success obs : not(inRearAisleHub(?actor));
        success obs : inAisle(?actor, ?aisle);
        success obs : atShelf(?actor, ?shelf);
    }

    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "inAisle(?actor, ?aisle)");
    !northQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "northOfAisle(?actor, ?aisle)");
    !southQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "southOfAisle(?actor, ?aisle)");

    op: log(info, ">> going to aisle ?aisle");
    while(~obs:!query) {
        if(obs:!northQuery) {
            act:goSouth();
        } elseif(obs:!southQuery) {
            act:goNorth();
        } else {
            act:goWest();
        }
    }
}

() = gotoshelf["?actor goes to ?shelf"](edu.tufts.hrilab.fol.Symbol ?shelf:shelf, edu.tufts.hrilab.fol.Symbol ?aisle:aisle, edu.tufts.hrilab.fol.Symbol ?shelf2:shelf){
    java.lang.Boolean !val;
    edu.tufts.hrilab.fol.Predicate !query;
    edu.tufts.hrilab.fol.Predicate !westQuery;
    edu.tufts.hrilab.fol.Predicate !eastQuery;
    conditions : {
        pre infer : shelfInAisle(?shelf, ?aisle);
        pre obs : inAisle(?actor, ?aisle);
        pre obs : atShelf(?actor, ?shelf2);
    }
    effects : {
        success obs : not(atShelf(?actor, ?shelf2));
        success obs : atShelf(?actor, ?shelf);
    }
    op: log(info, ">> going to shelf ?shelf");

    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "atShelf(?actor, ?shelf)");
    !westQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "westOf(?actor, ?shelf)");
    !eastQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "eastOf(?actor, ?shelf)");

    while(~obs:!query) {
        if (obs:!westQuery) {
            act:goEast();
        }

        // Go south if you're too far east
        elseif (obs:!eastQuery) {
            act:goWest();
        }
    }

}

() = enterstore["?actor enters store"]() {
    java.lang.Boolean !val;
    edu.tufts.hrilab.fol.Predicate !query;

    conditions : {
        pre infer : inEntrance(?actor);
    }
    effects : {
        success infer: not(inEntrance(?actor));
        success obs: inAisleHub(?actor);
        success obs: not(atCartReturn(?actor));
    }
    op: log(info, ">> entering store");
    while(~obs:inAisleHub(?actor)) {
        act:goEast();
    }
}


() = gotocartreturn["?actor goes to cart return"]() {
    java.lang.Boolean !val;
    edu.tufts.hrilab.fol.Predicate !query;
    edu.tufts.hrilab.fol.Predicate !northQuery;
    edu.tufts.hrilab.fol.Predicate !southQuery;

    conditions : {
        pre obs : inAisleHub(?actor);
    }
    effects : {
        success obs : not(inAisleHub(?actor));
        success obs : atCartReturn(?actor);
        success obs : inEntrance(?actor);

    }
    op: log(info, ">> going to cart return");

    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "atCartReturn(?actor)");
    !northQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "northOfCartReturn(?actor)");
    !southQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "southOfCartReturn(?actor)");

    while(~obs:!query) {
        if(obs:!northQuery) {
            act:goSouth();
        } elseif(obs:!southQuery) {
            act:goNorth();
        } else {
            act:goWest();
        }
    }
    act:goSouth();
}

(edu.tufts.hrilab.fol.Symbol ?retcart) = acquirecart["?actor gets a cart from the cart return"](edu.tufts.hrilab.fol.Symbol ?cart:cart) {
    java.util.List !bindings;
    edu.tufts.hrilab.fol.Predicate !query;
    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !x;

    conditions : {
        pre obs : atCartReturn(?actor);
        pre infer : handsFree(?actor);
        pre infer : inStore(?actor);
    }
    effects : {
       success infer : holdingCart(?actor, ?cart);
       success infer : holding(?actor,?cart);
       success infer : not(handsFree(?actor));
       success infer : fluent_assign(generalFoodQuantity(?cart), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, fresh_fish:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, ham:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, prepared_foods:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, garlic:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, apples:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, avocado:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, milk:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, leek:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, oranges:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, onion:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, cucumber:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, sausage:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, lettuce:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, broccoli:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, cheese_wheel:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, strawberry_milk:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, carrot:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, yellow_bell_pepper:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, red_bell_pepper:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, steak:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, swiss_cheese:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, raspberry:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, strawberry:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, banana:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, brie_cheese:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, chicken:food), 0);
       success infer : fluent_assign(specificFoodQuantity(?cart, chocolate_milk:food), 0);

    }

    op:log(info, ">> getting a cart from the cart return");
    act:interactWithObject();
    (?retcart) = act:lastCart();
}

() = putfoodincart["?actor puts ?food in cart"](edu.tufts.hrilab.fol.Symbol ?food:food, edu.tufts.hrilab.fol.Symbol ?shelf:shelf, edu.tufts.hrilab.fol.Symbol ?cart:cart) {

    java.lang.Boolean !val;
    edu.tufts.hrilab.fol.Predicate !query;
    edu.tufts.hrilab.fol.Predicate !query2;

    conditions : {
        pre infer : stockedAt(?food, ?shelf);
        pre obs : atShelf(?actor, ?shelf);
        pre infer : holdingCart(?actor, ?cart);
        pre infer : not(full(?cart));
        pre infer : inStore(?actor);
    }
    effects : {
        success infer : fluent_increase(specificFoodQuantity(?cart, ?food), 1);
        success infer : fluent_increase(generalFoodQuantity(?cart), 1);
        success infer : fluent_increase(inventoryQuantity(?actor, ?food), 1);
        success infer : not(purchasedInventory(?actor));
    }

    //op:log(debug, ">> putting ?food from ?shelf in ?cart");
    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "canInteractWith(?actor, ?shelf)");
    !query2 = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "canInteractWith(?actor, ?cart)");

    act:goSouth();
    act:toggleShoppingCart();

    while(~obs:!query) {
        act:goNorth();
    }

    act:interactWithObject();
    act:interactWithObject();
    while(~obs:!query2) {
        act:goSouth();
    }
    act:interactWithObject();
    act:interactWithObject();
    act:toggleShoppingCart();
}

() = registertoaislehub["?actor goes back to the aisle hub"](edu.tufts.hrilab.fol.Symbol ?register:register) {
    edu.tufts.hrilab.fol.Predicate !query;

    conditions : {
        pre infer : atRegister(?actor, ?register);
    }
    effects : {
        success infer : not(atRegister(?actor, ?register));
        success obs : inAisleHub(?actor);
    }

    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "inAisleHub(?actor)");

    while(~obs:!query) {
        act:goEast();
    }
}

() = gotoregister["?actor goes to checkout ?register"](edu.tufts.hrilab.fol.Symbol ?register:register) {
    java.lang.Boolean !val;
    edu.tufts.hrilab.fol.Predicate !query;
    edu.tufts.hrilab.fol.Predicate !northQuery;
    edu.tufts.hrilab.fol.Predicate !southQuery;

    conditions : {
        pre obs : inAisleHub(?actor);
    }
    effects : {
        success infer : atRegister(?actor, ?register);
        success infer : not(inAisleHub(?actor));
    }
    op: log(info, ">> going to register ?register");
    // Go south if you're too far north

    !northQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "northOf(?actor, ?register)");
    !southQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "southOf(?actor, ?register)");

    // Go north if you're too far south
    while(obs:!southQuery) {
        act:goNorth();
    }
    while(obs:!northQuery) {
        act:goSouth();
    }
}

() = checkoutcart["?actor checks out cart at register ?checkout"](edu.tufts.hrilab.fol.Symbol ?register:register, edu.tufts.hrilab.fol.Symbol ?cart:cart) {
    java.lang.Boolean !val;
    edu.tufts.hrilab.fol.Predicate !query;
    edu.tufts.hrilab.fol.Predicate !southQuery;
    edu.tufts.hrilab.fol.Predicate !eastQuery;
    conditions : {
        pre infer : atRegister(?actor, ?register);
        pre infer : holdingCart(?actor, ?cart);
        pre infer : inStore(?actor);
    }
    effects : {
        success infer : purchasedInventory(?actor); // This isn't true if the agent is using multiple carts, but whatevs.
        success obs : not(inStore(?actor));
        success obs : not(inAisleHub(?actor));
    }
    op: log(info, ">> checking out ?cart at ?register");
    // We're going to the south side of the checkout

    !southQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "southOf(?actor, ?register)");
    !eastQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "eastOf(?actor, ?register)");

    while(~obs:!southQuery) {
        act:goSouth();
    }
    act:goSouth();
    act:goSouth();
    act:goSouth();
    act:goSouth();
    act:goSouth();

    while(obs:!eastQuery) {
        act:goWest();
    }
    act:toggleShoppingCart();
    act:goNorth();
    act:interactWithObject();
    act:interactWithObject();
    act:interactWithObject();
    act:goWest();
    act:toggleShoppingCart();

    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "inAisleHub(?actor)");
    while(~obs:!query) {
       act:goEast();
    }

    act:leavestore();
}

() = leavestore["?actor leaves the store"]() {
    java.lang.Boolean !val;
    edu.tufts.hrilab.fol.Predicate !query;
    edu.tufts.hrilab.fol.Predicate !southQuery;
    edu.tufts.hrilab.fol.Predicate !northQuery;
    op: log(info, ">> leaving the store");

    !southQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "southOfExitRow(?actor)");
    !northQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "northOfExitRow(?actor)");
    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "inStore(?actor)");

    // Go south if you're too far north
    while(obs:!northQuery) {
        act:goSouth();
    }

    // Go north if you're too far south
    while(obs:!southQuery) {
        act:goNorth();
    }

    while(obs:!query) {
        act:goWest();
    }
}
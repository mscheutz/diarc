() = pickUp["?actor picks up ?count ?object"](edu.tufts.hrilab.fol.Symbol ?object, java.lang.Integer ?count) {
    java.lang.Integer !units = 0;

    conditions : {
      pre : on(?object,table);
      pre : count(?object,?count);
    }
    effects : {
      success : pickedUp(?actor,?object,?count);
      failure : pickedUp(?actor,?object,!units);
    }

    while (op:lt(!units, ?count)) {
      act:pickUp(?object);
      !units = op:++(!units);
    }
    op:log("info", "Picked up !units ?object .");
}


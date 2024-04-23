import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;

() = setupdemo[""](Symbol ?actor) {
    effects : {
        success infer : fluent_equals(cost, 0);
        success infer : at(self, location_2:location);
        success infer : free(self);
        success infer : fluent_equals(weight(physobj_1:physobj), 10);
        success infer : fluent_equals(weight(physobj_2:physobj), 20);
        success infer : fluent_equals(weight(physobj_3:physobj), 30);
        success infer : fluent_equals(weight(physobj_4:physobj), 40);
        success infer : fluent_equals(weight(physobj_5:physobj), 50);
        success infer : fluent_equals(weight(physobj_0:physobj), 100);
    }

    act:initAreas();
    op:log(info,"Demo initialized");
}

() = moveto["?actor moves from ?from to ?to"](Symbol ?from:location, Symbol ?to:location) {
   conditions : {
     pre infer : at(?actor,?from);
   }
   effects : {
     success infer : at(?actor,?to);
     success infer : not(at(?actor,?from));
     success infer : fluent_increase(cost(), distance(?from,?to));
   }

   op:log("info", "Moving from ?from to ?to");
}

() = load["?actor loads ?object at ?zone"](Symbol ?object:physobj, Symbol ?zone:location) {
   conditions : {
     pre : holding(?actor, ?object);
     //pre : property_of(?zone, truck:property);
     pre infer : at(?actor,?zone);
     pre : fluent_leq(add(weight(?object), current_weight(?zone)), capacity(?zone));
     pre : fluent_geq(slots(?zone), 1);
   }
   effects : {
     success : not(holding(?actor,?object));
     success : free(?actor);
     success infer : at(?object,?zone);
     success infer : fluent_increase(current_weight(?zone), weight(?object));
     success infer : fluent_decrease(slots(?zone), 1);
     success infer : fluent_increase(cost(), 10);
   }

    op:log("info","Loading ?object at ?zone");
}

() = unload["?actor unloads ?object from ?zone"](Symbol ?object:physobj, Symbol ?zone:location) {
      conditions : {
        pre : free(?actor);
        pre infer : at(?actor,?zone);
        pre infer : at(?object,?zone);
//        pre : property_of(?zone, truck:property);
      }
      effects : {
        success : holding(?actor,?object);
        success : not(free(?actor));
        success infer : not(at(?object,?zone));
        success infer : fluent_decrease(current_weight(?zone), weight(?object));
        success infer : fluent_increase(slots(?zone), 1);
        success infer : fluent_increase(cost(), 10);
      }
       op:log("info","Unloading ?object at ?zone");
}

() = loadLanguage[""](Symbol ?zone) {
    Variable !key = X;

    // Get truck capacity, if not known
    Predicate !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "fluent_equals(capacity(?zone), 0)");
    Symbol !capacity;

    java.util.List !bindings = act:queryBelief(!queryPred);
    java.util.Map !binding = op:get(!bindings, 0);

    if(op:invokeMethod(!binding, "isEmpty")) {
        !capacity = act:askweight(?zone);
    } else {
        !capacity = op:get(!binding,!key);
    }


    !binding = act:askQuestionFromString(?actor,"Do you want to optimize for cargo, weight, or time?", val(X));
    Symbol !metric = op:get(!binding, !key);

    Predicate !goal;
    Predicate !metricGoal;
    Predicate !intermediateGoal;
    if(op:equalsValue(!metric, "weight")) {
        !intermediateGoal = act:createWeightGoal(?zone);
        !goal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goal", ?actor, !intermediateGoal);
        !metricGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "minimize(cost())");
    } elseif(op:equalsValue(!metric, "cargo")) {
        !goal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goal(?actor, fluent_equals(slots(?zone), 0))");
        !metricGOal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "minimize(cost())");
    } elseif(op:equalsValue(!metric, "time")) {
        !goal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goal(?actor, fluent_equals(slots(?zone), 0))");
        !metricGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "minimize(cost())");
    }
    act:submitGoalWithMetric(!goal, !metricGoal);
}

(Symbol ?weight) = askweight[""](Symbol ?zone) {

    effects : {
      success infer : fluent_equals(capacity(?zone), ?weight);
    }

    java.util.Map !bindings;
    Predicate !tmp;
    Variable !x = "X";
    !bindings = act:askQuestionFromString(?actor,"What is the weight capacity of ?zone?", val(X));
    ?weight = op:get(!bindings, !x);

    act:generateResponseFromString("Okay.");
}

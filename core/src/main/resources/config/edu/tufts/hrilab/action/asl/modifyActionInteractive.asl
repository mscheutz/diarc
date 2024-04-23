() = modifyInteractive[""](edu.tufts.hrilab.fol.Predicate ?actionName) {

    edu.tufts.hrilab.fol.Predicate !likeGoal;
    java.util.Map !bindings;
    java.util.List !bindingArgs;
    edu.tufts.hrilab.fol.Variable !x = "X";
    edu.tufts.hrilab.fol.Predicate !modification;
    java.lang.String !modName;
    edu.tufts.hrilab.fol.Predicate !location = "none()";

    !bindings = act:askQuestionFromString(?actor,"what will change", mod(X,Y));
    !modification = op:get(!bindings, !x);
    op:log(debug, "modification !modification");
    act:modifyAction(?actionName,!modification,!location);

    !bindings = act:askQuestionFromString(?actor,"okay are there any other changes", mod(X,Y));
    !modification = op:get(!bindings, !x);
    !modName = op:getName(!modification);
    while(op:!=(!modName,"none")){
        act:modifyAction(?actionName,!modification,!location);
        !bindings = act:askQuestionFromString(?actor,"okay are there any other changes", mod(X,Y));
        !modification = op:get(!bindings, !x);
        !modName = op:getName(!modification);
    }

    act:sayText("okay");
}

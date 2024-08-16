() = modifyInteractiveGerman[""](edu.tufts.hrilab.fol.Predicate ?actionName) {

    edu.tufts.hrilab.fol.Predicate !likeGoal;
    java.util.Map !bindings;
    java.util.List !bindingArgs;
    edu.tufts.hrilab.fol.Variable !x = "X";
    edu.tufts.hrilab.fol.Predicate !modification;
    java.lang.String !modName;
    edu.tufts.hrilab.fol.Predicate !location = "none()";

    !bindings = act:askQuestionFromString(?actor,"was wird geaendert", mod(X,Y));
    !modification = op:get(!bindings, !x);
    op:log(debug, "modification !modification");
    act:modifyAction(?actionName,!modification,!location);

    !bindings = act:askQuestionFromString(?actor,"okay gibt es noch andere Aenderungen", mod(X,Y));
//    !bindings = act:askQuestionFromString(?actor,"okay sind da mehr Aenderungen", mod(X));

    !modification = op:get(!bindings, !x);
    !modName = op:getName(!modification);
    while(op:!=(!modName,"none")){
        act:modifyAction(?actionName,!modification,!location);
        !bindings = act:askQuestionFromString(?actor,"okay gibt es noch andere Aenderungen", mod(X,Y));
//        !bindings = act:askQuestionFromString(?actor,"okay sind da mehr Aenderungen", mod(X));
        !modification = op:get(!bindings, !x);
        !modName = op:getName(!modification);
    }

    act:sayText("okay");
}

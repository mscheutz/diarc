import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;

() = stiTests() {
    Symbol !y = mug;
    Predicate !pred = above(cup,table);
    java.lang.String !predString;
    Term !term = above(plate,table);
    Symbol !symbol = above(bowl,table);

    // test variable binding
    goal:above(!y, table);

    // intentionally left spaces before, in-between, and after predicate
    goal: above(cup, table) ;

    goal:!pred;
    goal:!term;

    // expected to throw exception. actspec can't handle variable form
    // act:!pred;

    // expected to fail. must be term or predicate class
    // goal:!symbol;

    // should fail. can't pass in string like this
    // !predString = op:invokeMethod(!pred, "toString");
    // goal:!predString;

}

() = liftCup(Symbol ?x) {
    effects : {
      success : above(?x,table);
    }
    op:log("info", "in liftCup ?x table script");
}


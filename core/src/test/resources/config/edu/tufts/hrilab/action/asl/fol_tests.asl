import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

() = createPredicateTest() {
  // string based construction
  Predicate !pred1 = op:createPredicate("and(this,that)");
  Predicate !pred2 = op:createPredicate("and(the,other)");

  // fol based, vararg construction
  Predicate !pred3 = op:createPredicate("and", !pred1, !pred2);
}
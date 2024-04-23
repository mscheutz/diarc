import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Term;

() = init["simulates environmental setup"]() {
   //robot identifiers
   Symbol !mobileYumi = "mobileyumi:mobileyumi";
   Symbol !yumi = "yumi:yumi";
   Symbol !gofa = "gofa:gofa";

   //location reference local vars
   Symbol !pantryRef;
   Symbol !tableOneRef;
   Symbol !tableTwoRef;
   Symbol !tableThreeRef;
   Symbol !deliveryRef;

   //pose reference local vars
   Symbol !cookTopRef;
   Symbol !fryerRef;
   Symbol !ingredientPoseRef;
   Symbol !prepAreaOneRef;
   Symbol !prepAreaTwoRef;
   Symbol !prepAreaOneMobileRef;
   Symbol !prepAreaTwoMobileRef;
   Symbol !prepAreaThreeRef;
   Symbol !deliveryTableRef;

   act:setAllSleepDuration(0);

   //Save locations
   !pantryRef= !mobileYumi.act:saveLocationHelper("pantry location");
   !tableOneRef= !mobileYumi.act:saveLocationHelper("table one");
   !tableTwoRef= !mobileYumi.act:saveLocationHelper("table two");
   !tableThreeRef= !mobileYumi.act:saveLocationHelper("table three");
//   !deliveryRef= !mobileYumi.act:saveLocationHelper("delivery");

   //Save poses
   //TODO:brad:better name
   !cookTopRef= !gofa.act:savePoseHelper("cook top",!tableOneRef);
   !fryerRef= !gofa.act:savePoseHelper("fryer",!tableOneRef);
   !prepAreaOneRef= !gofa.act:savePoseHelper("prep area one",!tableOneRef);

   !prepAreaTwoRef= !yumi.act:savePoseHelper("prep area two",!tableTwoRef);

   !ingredientPoseRef= !mobileYumi.act:savePoseHelper("pantry",!pantryRef);
   !prepAreaThreeRef= !mobileYumi.act:savePoseHelper("prep area three",!tableThreeRef);
   !prepAreaOneMobileRef= !mobileYumi.act:savePoseHelper("prep area one mobile",!tableThreeRef, !prepAreaOneRef);
   !prepAreaTwoMobileRef= !mobileYumi.act:savePoseHelper("prep area two mobile",!tableThreeRef, !prepAreaTwoRef);
//   !deliveryTableRef= !mobileYumi.act:savePoseHelper("delivery table",!deliveryRef);


   //Define Ingredients
   act:defineIngredientHelper("bun",!ingredientPoseRef,"none","bun");
   act:defineIngredientHelper("bottom bun",!ingredientPoseRef,"none","\"bottom bun\"");
   act:defineIngredientHelper("top bun",!ingredientPoseRef,"none","\"top bun\"");
   act:defineIngredientHelper("beef patty",!ingredientPoseRef,"none","\"beef patty\"");
   act:defineIngredientHelper("veggie patty",!ingredientPoseRef,"none","\"veggie patty\"");
   act:defineIngredientHelper("chicken patty",!ingredientPoseRef,"none","\"chicken patty\"");
   act:defineIngredientHelper("lettuce",!ingredientPoseRef,"none","lettuce");
   act:defineIngredientHelper("tomato",!ingredientPoseRef,"none","tomato");
   act:defineIngredientHelper("ketchup",!ingredientPoseRef,"none","ketchup");
   act:defineIngredientHelper("mustard",!ingredientPoseRef,"none","mustard");
   act:defineIngredientHelper("mayonnaise",!ingredientPoseRef,"none","mayonnaise");
   act:defineIngredientHelper("fries box",!ingredientPoseRef,"none","\"fries box\"");
   act:defineIngredientHelper("potatoes",!ingredientPoseRef,"none","potatoes");
     act:defineIngredientHelper("coke",!ingredientPoseRef,"drink","coke");
   act:defineIngredientHelper("sprite",!ingredientPoseRef,"drink","sprite");
   act:defineIngredientHelper("guiness",!ingredientPoseRef,"drink","guiness");
   act:defineIngredientHelper("crust",!ingredientPoseRef,"none","crust");
   act:defineIngredientHelper("pepperoni",!ingredientPoseRef,"topping","pepperoni");
   act:defineIngredientHelper("mushrooms",!ingredientPoseRef,"topping","mushrooms");
   act:defineIngredientHelper("onions",!ingredientPoseRef,"topping","onions");
   act:defineIngredientHelper("tomato sauce",!ingredientPoseRef,"none","\"tomato sauce\"");
   act:defineIngredientHelper("cheese",!ingredientPoseRef,"none","cheese");
//   act:defineIngredientHelper("salsa verde",!ingredientPoseRef,"salsa","salsaVerde");
   act:defineIngredientHelper("mango salsa",!ingredientPoseRef,"salsa","\"mango salsa\"");
   act:defineIngredientHelper("pico de gallo",!ingredientPoseRef,"salsa","\"pico de gallo\"");
   act:defineIngredientHelper("tortilla",!ingredientPoseRef,"none","tortilla");
   act:defineIngredientHelper("pork",!ingredientPoseRef,"taco meat","pork");
   act:defineIngredientHelper("chicken",!ingredientPoseRef,"taco meat","chicken");


   //TODO:brad: is tray special?
   act:defineIngredientHelper("tray",gofapose_2:gofapose,"none","tray");
   act:setAllSleepDuration(1000);

}



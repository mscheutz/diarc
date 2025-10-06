//TODO: make methods safe/fail gracefully if packing solver not ran yet?
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.lang.String;
import java.util.Map;
import java.lang.Integer;

() = packingConfigurationStub() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    Integer !palletL;
    Integer !palletW;
    Integer !palletH;
    Integer !itemL;
    Integer !itemW;
    Integer !itemH;
    Integer !numItemsToPack;

    !bindings = act:askQuestionFromString(?actor,"Before commencing, there are outstanding packing parameters which must be configured. May I ask for them now?", val(X));
    if (~op:invokeMethod(!bindings,"isEmpty")) {
        !response = op:get(!bindings, !x);
        if (op:equalsValue(!response,yes)) {
            //TODO: could check to see which of these are already configured and ask conditionally
            !palletL = act:askForPalletLength();
            !palletW = act:askForPalletWidth();
            !palletH = act:askForPalletHeight();
            tsc:setPalletDimensions(!palletL,!palletW,!palletH);
            !itemL = act:askForItemLength();
            !itemW = act:askForItemWidth();
            !itemH = act:askForItemHeight();
            tsc:setPackingItemDimensions(!itemL,!itemW,!itemH);
            act:solvePalletPacking3D();
            act:reportMaximumPackedItems();

            !numItemsToPack = act:askForNumberOfItemsToPack();

            //loop/separate task
            act:reportStartingPacking();
            //TODO: pack !numItemsToPack items
            act:reportPackingComplete();
            return;
        }
    }
    exit(FAIL,missing(configurationParameters));
}

(Integer ?length) = askForPalletLength() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    String !sizeString;

    (!bindings) = act:askQuestionFromString(?actor,"How long is the pallet?", sizeInMM(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        //TODO EW: note to self - think about refactoring to have askQuestionFromString throw this error and then force
                // scripts to handle differently through recovery policy if desired.
        exit(FAIL,not(wasAnswered(myQuestion)));
    } else {
        !response = op:get(!bindings,!x);
        !sizeString = op:getName(!response);
        ?length = op:newObject("java.lang.Integer", !sizeString);
    }
}

(Integer ?width) = askForPalletWidth() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    String !sizeString;

    (!bindings) = act:askQuestionFromString(?actor,"How wide is the pallet?", sizeInMM(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    } else {
        !response = op:get(!bindings,!x);
        !sizeString = op:getName(!response);
        ?width = op:newObject("java.lang.Integer", !sizeString);
    }
}

(Integer ?height) = askForPalletHeight() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    String !sizeString;

    (!bindings) = act:askQuestionFromString(?actor,"How tall is the pallet?", sizeInMM(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    } else {
        !response = op:get(!bindings,!x);
        !sizeString = op:getName(!response);
        ?height = op:newObject("java.lang.Integer", !sizeString);
    }
}

(Integer ?length) = askForItemLength() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    String !sizeString;

    (!bindings) = act:askQuestionFromString(?actor,"How long is the item being packed?", sizeInMM(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    } else {
        !response = op:get(!bindings,!x);
        !sizeString = op:getName(!response);
        ?length = op:newObject("java.lang.Integer", !sizeString);
    }
}

(Integer ?width) = askForItemWidth() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    String !sizeString;

    (!bindings) = act:askQuestionFromString(?actor,"How wide is the item being packed?", sizeInMM(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    } else {
        !response = op:get(!bindings,!x);
        !sizeString = op:getName(!response);
        ?width = op:newObject("java.lang.Integer", !sizeString);
    }
}

(Integer ?height) = askForItemHeight() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    String !sizeString;

    (!bindings) = act:askQuestionFromString(?actor,"How tall is the item being packed?", sizeInMM(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    } else {
        !response = op:get(!bindings,!x);
        !sizeString = op:getName(!response);
        ?height = op:newObject("java.lang.Integer", !sizeString);
    }
}

() = reportMaximumPackedItems() {
    Integer !numLayers;
    Integer !numItemsPerLayer;
    Integer !totalNumItems;

    !numLayers = tsc:getNumLayers();
    !numItemsPerLayer = tsc:getNumberOfItemsPerLayer();
    !totalNumItems = tsc:getTotalNumberOfItems();

    act:generateResponseFromString("I can fit a maximum of !totalNumItems items into the pallet. This will consist of !numItemsPerLayer items per layer across !numLayers layers.");
}

(Integer ?numItems) = askForNumberOfItemsToPack() {
    Map !bindings;
    Variable !x = "X";
    Integer !totalNumItems = 0;
    String !numItemsString;
    String !response;

    !totalNumItems = tsc:getTotalNumberOfItems();

    !bindings = act:askQuestionFromString(?actor, "do you want me to fill the pallet completely?", val(X));
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !response = op:get(!bindings, !x);
    if (op:equalsValue(!response,yes)) {
        ?numItems = op:newObject("java.lang.Integer", !totalNumItems);
        return;
    }

    !bindings = act:askQuestionFromString(?actor, "How many items would you like me to pack?", amount(X));
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !numItemsString = op:get(!bindings, !x);
    if (op:equalsValue(!numItemsString,"max")) {
        ?numItems = op:newObject("java.lang.Integer", !totalNumItems);
    } else {
        ?numItems = op:newObject("java.lang.Integer", !numItemsString);
    }

    if (op:gt(?numItems,!totalNumItems)) {
        act:generateResponseFromString("I cannot fit ?numItems into a single pallet, the most I can do is !totalNumItems. I will proceed with that maximum amount.");
    }
    if (op:lt(?numItems,0)) {
        exit(FAIL,cannot(packNegativeAmountOfObjects));
    }
}

() = reportStartingPacking() {
    //ask, do you want me to start/let me know when you want me to start?
    act:generateResponseFromString("Starting packing procedure now");
}

() = reportPackingComplete() {
    act:generateResponseFromString("I have finished packing this pallet, please replace it with an empty one");
}
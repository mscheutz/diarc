// IMPORTS //
import React, { useState } from "react";

// NPM packages
import { SingleValue } from "react-select";
import { SendMessage } from "react-use-websocket";

// Internal imports
import { Button } from "../../Button";
import ActionChain, { ActionChainLink, linkToPredicate } from "./ActionChain";
import { Option } from "../util/constants";

// UTILITY METHODS //
// Given an Option containing an action signature, find how many arguments it
// takes
const getArgumentCount = (option: Option) => {
    return Array.from(option.value).filter((str) => str === "?").length;
};

// Given an action signature, find its argument names.
const getArgumentNames = (signature: string) => {
    return signature
        .slice(signature.indexOf("(") + 1, signature.indexOf(")")) // get arg list
        .split(",")
        .map((arg) => {
            return arg.trim()
                .slice(1) // remove "?" at start
        });
};

type Props = {
    actionsAvailable: string[],
    sendMessage: SendMessage,
    path: string
};

/**
 * ActionProgrammer. Subcomponent that provides a GUI to define a new action.
 * The top has a text box to define the action's name. This is followed by
 * the Action Chain, and finally by buttons to Add or Clear the new action.
 * @author Lucien Bao
 * @version 1.0
 */
const ActionBuilder: React.FC<Props> = ({
    actionsAvailable, sendMessage, path
}) => {
    // STATE & CALLBACKS //
    const actionOptions: Option[] = actionsAvailable.map((str) => {
        return { value: str, label: str }
    });

    // Head of the chain
    const [actionName, setActionName] = useState<string>("");
    const handleActionNameChange = (e: React.ChangeEvent<HTMLInputElement>) =>
        setActionName(e.target.value);

    const [actionChain, setActionChain] = useState<ActionChainLink[]>([]);

    // Handle changing an action chain link's selected action.
    const handleChainSelect = (index: number, option: SingleValue<Option>) => {
        if (option === null) return;
        setActionChain(actionChain.slice(0, index)
            .concat({
                action: option,
                args: new Array(getArgumentCount(option)).fill("")
            })
            .concat(actionChain.slice(index + 1)));
    };

    // Handle changing one of an action chain link's text inputs (arguments).
    const handleArgInputChange = (
        e: React.ChangeEvent<HTMLInputElement>,
        actionLinkIndex: number,
        argumentIndex: number
    ) => {
        setActionChain(actionChain.slice(0, actionLinkIndex)
            .concat({
                action: actionChain[actionLinkIndex].action,
                args: actionChain[actionLinkIndex].args.slice(0, argumentIndex)
                    .concat(e.target.value)
                    .concat(actionChain[actionLinkIndex].args.slice(argumentIndex + 1))
            })
            .concat(actionChain.slice(actionLinkIndex + 1))
        );
    };

    // Append a new action to the chain.
    const addToChain = () => {
        setActionChain(actionChain.concat({
            action: actionOptions[0],
            args: new Array(getArgumentCount(actionOptions[0])).fill("")
        }));
    };

    // Handle deleting a selected action from the chain.
    const deleteFromChain = (index: number) => {
        setActionChain(actionChain.slice(0, index)
            .concat(actionChain.slice(index + 1)));
    };

    // Handle submitting the newly defined action.
    const handleSubmit = (e: React.FormEvent) => {
        sendMessage(JSON.stringify({
            path: path,
            method: "builder",
            name: actionName,
            chain: actionChain.map(link => linkToPredicate(link))
        }));

        // ...then reset the form
        handleReset(e);
    };

    // Resets the entire form.
    const handleReset = (e: React.FormEvent) => {
        setActionName("");
        setActionChain([]);
        e.preventDefault();
    };

    // DOM //
    return (
        <form
            className="flex flex-col rounded-md shadow-md border
                       border-[#d1dbe3] grow p-3 gap-2"
            onSubmit={handleSubmit}
            onReset={handleReset}>

            {/* Head */}
            <label className="rounded-md shadow-md border border-[#d1dbe3]
                              px-2 pt-2">
                Action name
                <span className="text-red-500">*</span>
                <div className="flex flex-row items-center gap-1">
                    <input
                        className="block box-border rounded mt-1 mb-2 text-sm
                           border border-slate-500 p-2 font-mono grow"
                        type="text"
                        value={actionName}
                        onChange={handleActionNameChange}
                        required
                    />
                    <label>(?actor)</label>
                </div>
            </label>

            <ActionChain
                actionChain={actionChain}
                actionOptions={actionOptions}
                handleChainSelect={handleChainSelect}
                deleteFromChain={deleteFromChain}
                handleArgInputChange={handleArgInputChange}
                getArgumentNames={getArgumentNames}
                addToChain={addToChain}
            />

            {/* Menu */}
            <div className="grid grid-cols-2 gap-2">
                <Button
                    type="submit"
                    title="Add action">
                    Add
                </Button>
                <Button
                    type="reset"
                    title="Clear action">
                    Clear
                </Button>
            </div>
        </form>
    );
};

export default ActionBuilder;
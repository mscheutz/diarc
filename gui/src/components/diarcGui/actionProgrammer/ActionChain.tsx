// IMPORTS //
import React from "react";

// NPM packages
import Select, { SingleValue } from "react-select";

import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faPlusCircle, faTrashCan } from "@fortawesome/free-solid-svg-icons";

// Internal imports
import { Option } from "../util/constants";

type ActionChainLink = {
    action: Option,
    args: string[]
}

const linkToPredicate = (link: ActionChainLink) => {
    return link.action.value.slice(0, link.action.value.indexOf("?"))
        .concat(link.args[0] === "" ? "?actor" : "") // actor arg is optional
        .concat(link.args.join(","))
        .concat(")");
};

export { ActionChainLink, linkToPredicate };

type Props = {
    actionChain: ActionChainLink[],
    actionOptions: Option[],
    handleChainSelect: (index: number, option: SingleValue<Option>) => void,
    deleteFromChain: (index: number) => void,
    handleArgInputChange: (e: React.ChangeEvent<HTMLInputElement>,
        actionLinkIndex: number, argumentIndex: number) => void,
    getArgumentNames: (signature: string) => string[],
    addToChain: () => void
};

/**
 * ActionChain. Subcomponent that renders the series of actions that make up
 * the body of the action to be learned.
 * 
 * The action chain is made up of "links"; each represents an ASL action call.
 * Each link contains a `select` input to choose its action and a series of
 * text boxes to determine the arguments. Finally, each link has a delete
 * button to remove it from the chain. The end of the action chain has a button
 * to add new links.
 * 
 * @author Lucien Bao
 * @version 1.0
 */
const ActionChain: React.FC<Props> = ({
    actionChain, actionOptions, handleChainSelect, deleteFromChain,
    handleArgInputChange, getArgumentNames, addToChain
}) => {
    // DOM //
    return (
        <div className="grow flex flex-col gap-2 overflow-auto min-h-0
                        basis-0">
            {actionChain.map((chainLink, actionLinkIndex) =>
                // Chain link
                <div
                    key={actionLinkIndex}
                    className="flex flex-row items-center gap-2">

                    {/* Line number */}
                    <div>
                        {actionLinkIndex + 1 + ":"}
                    </div>

                    {/* Box */}
                    <div className="rounded-md shadow-md border
                                    border-[#d1dbe3] p-2 grow flex flex-col
                                    items-left gap-2 min-w-0 basis-0">

                        {/* Content */}
                        <div className="grow flex flex-row items-center
                                        w-full gap-2 min-w-0 basis-0">
                            <Select
                                className="grow text-sm min-w-0 basis-0"
                                options={actionOptions}
                                value={chainLink.action}
                                onChange={(e) => handleChainSelect(actionLinkIndex, e)}
                                autoFocus
                            />

                            {/* Delete button */}
                            <button>
                                <FontAwesomeIcon
                                    className="text-red-500 cursor-pointer
                                               focus:ring-2 ring-offset-5"
                                    icon={faTrashCan}
                                    size="lg"
                                    onClick={() => deleteFromChain(actionLinkIndex)}
                                />
                            </button>
                        </div>

                        {/* Argument inputs */}
                        {getArgumentNames(chainLink.action.value)
                            .map((arg, argumentIndex) => {
                                return (
                                    <div
                                        key={argumentIndex}
                                        className="flex flex-row gap-2
                                                   items-center">

                                        {/* "Required" dot */}
                                        <label className="font-mono text-sm">
                                            {arg}
                                            <span className="text-red-500">
                                                {argumentIndex !== 0 && "*"}
                                            </span>
                                        </label>

                                        {/* Text box */}
                                        <input
                                            className="block box-border rounded text-sm
                                                       border border-slate-500 p-1 font-mono grow"
                                            type="text"
                                            value={chainLink.args[argumentIndex]}
                                            onChange={(e) => handleArgInputChange(e, actionLinkIndex, argumentIndex)}
                                            required={argumentIndex !== 0}
                                        />
                                    </div>
                                );
                            })}
                    </div>
                </div>
            )}

            {/* "Add" button */}
            {
                <button
                    className="self-center rounded-full hover:shadow-lg
                               cursor-pointer mb-96 focus:ring-2"
                    onClick={addToChain}>
                    <FontAwesomeIcon
                        className="text-slate-900 hover:text-slate-900/90"
                        icon={faPlusCircle}
                        size="2xl"
                    />
                </button>
            }
        </div>
    );
};

export default ActionChain;
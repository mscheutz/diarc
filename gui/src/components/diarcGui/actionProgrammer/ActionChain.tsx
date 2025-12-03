import React from "react";

import Select, { SingleValue } from "react-select";

import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faPlusCircle, faTrashCan } from "@fortawesome/free-solid-svg-icons";

import { Option } from "../util/constants";

type ActionChainLink = {
    action: Option,
    args: string[]
}

const linkToPredicate = (link: ActionChainLink) => {
    return link.action.value.slice(0, link.action.value.indexOf("?"))
        .concat(link.args[0] === "" ? "?actor" : "")
        .concat(link.args.join(","))
        .concat(")");
};

export type { ActionChainLink };
export { linkToPredicate };

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

const ActionChain: React.FC<Props> = ({
    actionChain, actionOptions, handleChainSelect, deleteFromChain,
    handleArgInputChange, getArgumentNames, addToChain
}) => {
    const selectStyles = {
        control: (provided: any) => ({
            ...provided,
            backgroundColor: 'hsl(var(--background))',
            borderColor: 'hsl(var(--border))',
        }),
        menu: (provided: any) => ({
            ...provided,
            backgroundColor: 'hsl(var(--popover))',
            borderColor: 'hsl(var(--border))',
        }),
        option: (provided: any, state: any) => ({
            ...provided,
            backgroundColor: state.isFocused ? 'hsl(var(--accent))' : 'hsl(var(--popover))',
            color: 'hsl(var(--foreground))',
        }),
        singleValue: (provided: any) => ({
            ...provided,
            color: 'hsl(var(--foreground))',
        }),
        input: (provided: any) => ({
            ...provided,
            color: 'hsl(var(--foreground))',
        }),
    };

    return (
        <div className="grow flex flex-col gap-2 overflow-auto min-h-0
                        basis-0">
            {actionChain.map((chainLink, actionLinkIndex) =>
                <div
                    key={actionLinkIndex}
                    className="flex flex-row items-center gap-2">

                    <div>
                        {actionLinkIndex + 1 + ":"}
                    </div>

                    <div className="rounded-md shadow-md border
                                    border-border p-2 grow flex flex-col
                                    items-left gap-2 min-w-0 basis-0 bg-card">

                        <div className="grow flex flex-row items-center
                                        w-full gap-2 min-w-0 basis-0">
                            <Select
                                className="grow text-sm min-w-0 basis-0"
                                options={actionOptions}
                                value={chainLink.action}
                                onChange={(e) => handleChainSelect(actionLinkIndex, e)}
                                autoFocus
                                styles={selectStyles}
                            />

                            <button>
                                <FontAwesomeIcon
                                    className="text-red-500 dark:text-red-400 cursor-pointer
                                               focus:ring-2 ring-offset-5"
                                    icon={faTrashCan}
                                    size="lg"
                                    onClick={() => deleteFromChain(actionLinkIndex)}
                                />
                            </button>
                        </div>

                        {getArgumentNames(chainLink.action.value)
                            .map((arg, argumentIndex) => {
                                return (
                                    <div
                                        key={argumentIndex}
                                        className="flex flex-row gap-2
                                                   items-center">

                                        <label className="font-mono text-sm">
                                            {arg}
                                            <span className="text-red-500 dark:text-red-400">
                                                {argumentIndex !== 0 && "*"}
                                            </span>
                                        </label>

                                        <input
                                            className="block box-border rounded text-sm
                                                       border border-input bg-background text-foreground p-1 font-mono grow
                                                       focus:outline-none focus:ring-2 focus:ring-ring"
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

            {
                <button
                    className="self-center rounded-full hover:shadow-lg
                               cursor-pointer mb-96 focus:ring-2"
                    onClick={addToChain}>
                    <FontAwesomeIcon
                        className="text-primary hover:text-primary/90"
                        icon={faPlusCircle}
                        size="2xl"
                    />
                </button>
            }
        </div>
    );
};

export default ActionChain;

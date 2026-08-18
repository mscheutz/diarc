// IMPORTS //
import React from "react";

// NPM packages
import TreeView from "react-accessible-treeview";

// Internal imports
import "../util/TreeStyle.css";

type Props = {
    actionList: any[],
    setActionFormContext: React.Dispatch<React.SetStateAction<string[]>>,
    setSelectedIds: React.Dispatch<React.SetStateAction<Set<number>>>
}

/**
 * ActionBrowser. Subcomponent which renders a list view of available actions.
 * This is a flattened `TreeView` which allows for multiple selections;
 * when an action is selected, it will update the Action Form context so that
 * the Action Form knows how to populate its fields.
 * 
 * @author Lucien Bao
 * @version 1.0
 */
const ActionBrowser: React.FC<Props> = ({
    actionList, setActionFormContext, setSelectedIds
}) => {
    // ActionFormContext is a string[] whose first el't is the name of the
    // action and any further el'ts are its parameters

    // Callback when actions are selected
    const handleSelect = (e: any) => {
        setSelectedIds(e.treeState.selectedIds);

        const actionSignature: string = e.element.name;
        const openParenthesis = actionSignature.indexOf("(");
        const name = actionSignature.slice(0, openParenthesis);
        const parameters =
            actionSignature
                .slice(openParenthesis + 1, -1)
                .split(", ");
        setActionFormContext(
            [name].concat(
                // take off the "?" at the start of each parameter
                parameters.map((str) => (str.slice(1)))
            )
        );
    };

    // DOM //
    return (
        <div className="flex flex-col w-full min-h-0 max-h-full
                        shrink overflow-x-scroll overflow-y-scroll grow-0
                        border border-[#d1dbe3] rounded-md shadow-md
                        md:border-0">
            <TreeView
                data={actionList}
                className="basic"
                onNodeSelect={handleSelect}
                multiSelect
                clickAction="EXCLUSIVE_SELECT"
                nodeRenderer={
                    ({ element, getNodeProps, level, isSelected }) => {
                        return (
                            <div
                                {...getNodeProps()}
                                style={{ paddingLeft: 20 * level - 15 }}
                                title={element.name}
                                className={isSelected ? "selected" : "tree-node"}>
                                <p className="text-nowrap text-sm font-mono
                                              select-none">
                                    {element.name}
                                </p>
                            </div>
                        )
                    }
                }
            />
        </div>
    );
};

export default ActionBrowser;
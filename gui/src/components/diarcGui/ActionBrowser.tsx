/**
 * @author Lucien Bao
 * @version 0.9
 * @date 3 June 2024
 * ActionBrowser. Subcomponent which renders a list view of available actions.
 */

import React from "react";

import TreeView, { flattenTree } from "react-accessible-treeview";

import "./TreeStyle.css";

const ActionBrowser = ({ actionList, setActionFormContext }) => {
    // ActionFormContext is a string[] whose first el't is the name of the
    // action and any further el'ts are its parameters
    const handleSelect = (e) => {
        console.log(e.treeState.selectedIds);

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

    return (
        <div className="w-full h-full overflow-x-scroll overflow-y-scroll">
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
                                className={isSelected ? "selected" : "tree-node"}
                            >
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
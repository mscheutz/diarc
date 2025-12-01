import React from "react";

import TreeView from "react-accessible-treeview";

import "../util/TreeStyle.css";

type Props = {
    actionList: any[],
    setActionFormContext: React.Dispatch<React.SetStateAction<string[]>>,
    setSelectedIds: React.Dispatch<React.SetStateAction<Set<number>>>
}

const ActionBrowser: React.FC<Props> = ({
    actionList, setActionFormContext, setSelectedIds
}) => {
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
                parameters.map((str) => (str.slice(1)))
            )
        );
    };

    return (
        <div className="flex flex-col w-full min-h-0 max-h-full
                        shrink overflow-x-scroll overflow-y-scroll grow-0
                        border border-border rounded-md shadow-md
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

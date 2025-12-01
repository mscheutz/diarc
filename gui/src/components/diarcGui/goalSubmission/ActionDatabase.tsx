import React from "react";

import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faCheck, faSync } from "@fortawesome/free-solid-svg-icons";

import ActionBrowser from "./ActionBrowser";
import { Button } from "../../Button";

type Props = {
    actionList: any[],
    setActionFormContext: React.Dispatch<React.SetStateAction<string[]>>,
    setSelectedIds: React.Dispatch<React.SetStateAction<Set<number>>>,
    filterNodes: (value: string) => void,
    handleExport: () => void,
    exportStatus: string
};

const ActionDatabase: React.FC<Props> = ({
    actionList, setActionFormContext, setSelectedIds, filterNodes,
    handleExport, exportStatus
}) => {
    return (
        <div className="flex flex-col h-full min-h-0">
            <div className="flex items-center gap-2 p-3 border-b border-border bg-muted/30">
                <input
                    type="text"
                    className="flex-1 h-8 rounded-md text-sm border border-input bg-background 
                               text-foreground px-3 placeholder:text-muted-foreground
                               focus:outline-none focus:ring-1 focus:ring-primary focus:border-primary transition-colors"
                    placeholder="Search actions..."
                    onChange={(e) => filterNodes(e.target.value)}
                />
                <Button size="sm" variant="outline" onClick={handleExport}>
                    Export
                    {exportStatus === "wait" && (
                        <FontAwesomeIcon icon={faSync} className="ml-1.5 h-3 w-3 text-amber-500" spin />
                    )}
                    {exportStatus === "successful" && (
                        <FontAwesomeIcon icon={faCheck} className="ml-1.5 h-3 w-3 text-green-500" />
                    )}
                </Button>
            </div>
            <div className="flex-1 min-h-0 overflow-auto">
                {actionList.length === 1 ? (
                    <div className="p-4 text-center text-sm text-muted-foreground">
                        No actions match filter
                    </div>
                ) : (
                    <ActionBrowser
                        actionList={actionList}
                        setActionFormContext={setActionFormContext}
                        setSelectedIds={setSelectedIds}
                    />
                )}
            </div>
        </div>
    );
};

export default ActionDatabase;

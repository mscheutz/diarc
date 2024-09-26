// IMPORTS //
import React from "react";

// NPM packages
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faCheck, faSync } from "@fortawesome/free-solid-svg-icons";

// Internal imports
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

/**
 * ActionDatabase. Subcomponent which combines the action browser with a menu
 * bar for filtering and exporting actions.
 * 
 * @author Lucien Bao
 * @version 1.0
 */
const ActionDatabase: React.FC<Props> = ({
    actionList, setActionFormContext, setSelectedIds, filterNodes,
    handleExport, exportStatus
}) => {
    // DOM //
    return (
        <div className="flex flex-col shrink min-h-0 max-h-full gap-5 md:gap-1">
            {/* Menu bar */}
            <div className="p-3 rounded-tl-md outline outline-1 outline-[#d1dbe3]
                            flex flex-row gap-3 overflow-auto items-stretch
                            shrink-0">
                <input
                    type="text"
                    className="block box-border rounded 
                               text-sm border border-slate-500
                               font-mono grow pl-2"
                    placeholder="Filter actions..."
                    onChange={(e) => {
                        filterNodes(e.target.value);
                    }}>
                </input>
                <Button onClick={handleExport}>
                    Export selected
                </Button>
                {exportStatus ? (
                    <div className="flex flex-row items-center">
                        {exportStatus === "wait" ? (
                            <FontAwesomeIcon icon={faSync} color="#efd402" spin />
                        ) : (
                            <FontAwesomeIcon icon={faCheck} color="#00a505" />
                        )}
                    </div>) : null}
            </div>

            {/* Results */}
            <div className="flex flex-col w-full min-h-0 max-h-full shrink">
                {actionList.length === 1 ? (
                    <div className="pl-1">No actions match filter.</div>
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
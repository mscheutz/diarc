/**
 * @author Lucien Bao
 * @version 0.9
 * @date 13 June 2024
 * ActionDatabase. Subcomponent which combines the action browser with a menu
 * bar for filtering and exporting actions.
 */

import React from "react";
import { Button } from "../../Button";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faCheck, faSync } from "@fortawesome/free-solid-svg-icons";
import ActionBrowser from "./ActionBrowser";

const ActionDatabase = (
    { actionList, setActionFormContext, setSelectedIds, filterNodes,
        handleExport, exportStatus }
) => {
    return (
        <div className="flex flex-col shrink min-h-0 max-h-full">
            {/* Menu bar */}
            <div className="p-3 rounded-md outline outline-1 outline-[#d1dbe3]
                            flex flex-row gap-3 overflow-auto items-stretch
                            shrink-0"
            >
                <input
                    type="text"
                    className="block box-border rounded 
                               text-sm border border-slate-500
                               font-mono grow pl-2"
                    placeholder="Filter actions..."
                    onChange={(e) => {
                        filterNodes(e.target.value);
                    }}
                >
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
            <div className="w-full min-h-0 max-h-full shrink">
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
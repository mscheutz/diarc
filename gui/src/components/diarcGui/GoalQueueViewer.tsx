import React, { useState, useEffect } from "react";

import { SendMessage, ReadyState } from "react-use-websocket";

import { Button } from "../Button";
import { statuses } from "./util/constants";
import {FontAwesomeIcon} from "@fortawesome/react-fontawesome";
import {faPause, faPlay, faXmark} from "@fortawesome/free-solid-svg-icons";

type ChangeStatusProps = {
    handleSuspend: () => void,
    handleResume: () => void,
    handleCancel: () => void,
    hasSelection: boolean
}

const ChangeStatus: React.FC<ChangeStatusProps> = ({
    handleSuspend, handleResume, handleCancel, hasSelection
}) => {
    return (
        <div className="flex gap-1.5 shrink-0">
            <Button
                size="sm"
                variant="outline"
                title="Suspend"
                onClick={handleSuspend}
                disabled={!hasSelection}
                className="h-7 px-2 text-xs">
                <FontAwesomeIcon icon={faPause} />
            </Button>
            <Button
                size="sm"
                variant="outline"
                title="Resume"
                onClick={handleResume}
                disabled={!hasSelection}
                className="h-7 px-2 text-xs">
                <FontAwesomeIcon icon={faPlay} />
            </Button>
            <Button
                size="sm"
                variant="outline"
                title="Cancel"
                onClick={handleCancel}
                disabled={!hasSelection}
                className="h-7 px-2 text-xs">
                <FontAwesomeIcon icon={faXmark} />
            </Button>
        </div>
    );
};

type Entry = {
    name: string,
    actor: string,
    status: string,
    start: string,
    end: string,
    id: number,
    failureReason?: string,
    scheduled: boolean,
    interval: string
};

const StatusBadge: React.FC<{ status: string; failureReason?: string }> = ({ status, failureReason }) => {
    const isError = status === "failed";
    const isActive = status === "active";
    
    return (
        <span 
            className={`text-xs w-16 text-right shrink-0 ${isError ? "text-destructive" : isActive ? "text-primary font-medium" : "text-muted-foreground"}`}
            title={isError && failureReason ? `Reason: ${failureReason}` : undefined}
        >
            {status}{isError && failureReason && " ⓘ"}
        </span>
    );
};

type GoalTableProps = {
    data: Entry[],
    handleSelect: any,
    handleSelectAll: (ids: number[], selected: boolean) => void,
    selectedIds: number[],
    filteredActor: string,
    selectedStatuses: string[],
    actions: React.ReactNode
}

const formatTime = (datetime: string | undefined) => {
    if (!datetime) return "";
    const timePart = datetime.split(' ')[1];
    return timePart || datetime;
};

const GoalRow: React.FC<{
    goal: Entry;
    isSelected: boolean;
    onSelect: (e: React.ChangeEvent<HTMLInputElement>) => void;
    showCheckbox: boolean;
    isPast?: boolean;
}> = ({ goal, isSelected, onSelect, showCheckbox, isPast }) => {
    // For past goals, show end time; for active, show start time
    const timeToShow = isPast ? (goal.end || goal.start) : goal.start;
    
    return (
        <label className={`flex items-center gap-2 py-0.5 text-xs cursor-pointer hover:bg-muted/50 px-1 -mx-1 rounded-sm ${isPast ? "opacity-60" : ""}`}>
            {showCheckbox && (
                <input
                    type="checkbox"
                    value={goal.id}
                    checked={isSelected}
                    onChange={onSelect}
                    className="shrink-0"
                />
            )}
            <span className="truncate flex-1">{goal.name}</span>
            <StatusBadge status={goal.status} failureReason={goal.failureReason} />
            <span className="text-muted-foreground shrink-0 tabular-nums w-14 text-right">{formatTime(timeToShow)}</span>
        </label>
    );
};

const GoalTable: React.FC<GoalTableProps> = (
    { data, handleSelect, handleSelectAll, selectedIds, filteredActor, selectedStatuses, actions }
) => {
    let activeGoals: Entry[] = [], pastGoals: Entry[] = []
    data.forEach((g) => (g.status === "active" || g.status === "suspended" || g.status === "pending" ? activeGoals : pastGoals).push(g));

    const visibleActiveGoals = activeGoals.filter(
        (row) => selectedStatuses.includes(row.status) && row.actor.includes(filteredActor)
    );
    const visiblePastGoals = pastGoals.filter(
        (row) => selectedStatuses.includes(row.status) && row.actor.includes(filteredActor)
    );
    const visibleActiveIds = visibleActiveGoals.map((row) => row.id);
    const allSelected = visibleActiveIds.length > 0 && visibleActiveIds.every((id) => selectedIds.includes(id));
    const someSelected = visibleActiveIds.some((id) => selectedIds.includes(id));

    const onSelectAllChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        handleSelectAll(visibleActiveIds, e.target.checked);
    };

    return (
        <div className="space-y-1">
            <div className="flex items-center justify-between border-b border-border pb-1.5 mb-1">
                <div className="flex items-center gap-2">
                    <span className="text-xs text-muted-foreground">ACTIVE</span>
                    {visibleActiveGoals.length > 0 && (
                        <label className="flex items-center gap-1 text-xs text-muted-foreground cursor-pointer">
                            <input
                                type="checkbox"
                                checked={allSelected}
                                ref={(el) => { if (el) el.indeterminate = someSelected && !allSelected; }}
                                onChange={onSelectAllChange}
                            />
                            all
                        </label>
                    )}
                </div>
                {actions}
            </div>
            {visibleActiveGoals.length === 0 
                ? <p className="text-xs text-muted-foreground py-1">—</p>
                : visibleActiveGoals.map((goal) => (
                    <GoalRow
                        key={goal.id}
                        goal={goal}
                        isSelected={selectedIds.includes(goal.id)}
                        onSelect={handleSelect}
                        showCheckbox={true}
                    />
                ))
            }

            {visiblePastGoals.length > 0 && (
                <>
                    <div className="text-xs text-muted-foreground border-b border-border pb-1 mb-1 mt-3">HISTORY</div>
                    {visiblePastGoals.map((goal) => (
                        <GoalRow
                            key={goal.id}
                            goal={goal}
                            isSelected={false}
                            onSelect={() => {}}
                            showCheckbox={false}
                            isPast={true}
                        />
                    ))}
                </>
            )}
        </div>
    );
};

type ScheduledGoalTableProps = {
    scheduledData: Entry[],
    handleSelect: (e: React.ChangeEvent<HTMLInputElement>) => void,
    filteredActor: string,
    selectedStatuses: string[]
};

const ScheduledRow: React.FC<{ goal: Entry }> = ({ goal }) => (
    <div className="flex items-center gap-2 py-0.5 text-xs px-1 -mx-1">
        <span className="truncate flex-1">{goal.name}</span>
        <span className="text-muted-foreground shrink-0">{goal.interval && goal.interval !== "once" ? goal.interval : "once"}</span>
        <span className="text-muted-foreground shrink-0 tabular-nums">{goal.start?.split(' ')[1] || goal.start}</span>
    </div>
);

const ScheduledGoalTable: React.FC<ScheduledGoalTableProps> = (
    { scheduledData, filteredActor }
) => {
    const visibleScheduled = scheduledData.filter((row) => row.actor.includes(filteredActor));
    
    return (
        <div className="space-y-1">
            {visibleScheduled.length === 0 
                ? <p className="text-xs text-muted-foreground py-1">—</p>
                : visibleScheduled.map((goal) => (
                    <ScheduledRow key={goal.id} goal={goal} />
                ))
            }
        </div>
    );
};

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState
};

const GoalViewer: React.FC<Props> = ({
    path, lastMessage, sendMessage
}) => {
    const [sortAscending] = useState<boolean>(true);
    const [filteredActor] = useState<string>("");
    const [selectedStatuses] = useState<string[]>(statuses);
    const [data, setData] = useState<Entry[]>([]);
    const [scheduledData, setScheduledData] = useState<Entry[]>([]);
    const [selectedIds, setSelectedIds] = useState<number[]>([]);

    const handleGoalSelect = (e: React.ChangeEvent<HTMLInputElement>) => {
        if (e.target.checked)
            setSelectedIds(selectedIds.concat(+e.target.value))
        else
            setSelectedIds(selectedIds.filter((element) =>
                element !== +e.target.value))
    };

    const handleSelectAll = (ids: number[], selected: boolean) => {
        if (selected) {
            const newIds = Array.from(new Set([...selectedIds, ...ids]));
            setSelectedIds(newIds);
        } else {
            setSelectedIds(selectedIds.filter((id) => !ids.includes(id)));
        }
    };

    const handleSuspend = () => {
        sendMessage(JSON.stringify({
            "path": path,
            "method": "suspend",
            "ids": selectedIds
        }));
    };

    const handleResume = () => {
        sendMessage(JSON.stringify({
            "path": path,
            "method": "resume",
            "ids": selectedIds
        }));
    };

    const handleCancel = () => {
        sendMessage(JSON.stringify({
            "path": path,
            "method": "cancel",
            "ids": selectedIds
        }))
        setSelectedIds([])
    };

    useEffect(() => {
        const sorted = (array: Entry[]) => {
            const compareRaw = (entry0: Entry, entry1: Entry) => {
                if (entry0["status"] === "pending") {
                    if (entry1["status"] === "pending") {
                        if (entry0["id"] < entry1["id"]) {
                            return 1;
                        }
                    }
                    return -1;
                }

                if (entry0["status"] === "succeeded" || entry0["status"] === "failed" || entry0["status"] === "canceled") {
                    if (entry1["status"] === "succeeded" || entry1["status"] === "failed" || entry1["status"] === "canceled") {
                        if (entry0["end"] > entry1["end"]) {
                            return -1;
                        }
                    }
                    return 1;
                }

                if (entry1["status"] === "pending") {
                    return 1;
                }

                if (entry1["status"] === "succeeded" || entry1["status"] === "failed" || entry1["status"] === "canceled") {
                    return -1;
                }

                if (entry0["start"] < entry1["start"]) {
                    return -1;
                }
                if (entry0["start"] > entry1["start"]) {
                    return 1;
                }
                return 0;
            };

            const compare = (entry0: Entry, entry1: Entry) => {
                return sortAscending ? compareRaw(entry0, entry1)
                    : -compareRaw(entry0, entry1);
            };

            const copy = [...array];
            return copy.sort(compare);
        };

        setData(d => sorted(d));
        setScheduledData(d => sorted(d));

        if (lastMessage === null) return;
        const newData = JSON.parse(lastMessage.data);
        if (!newData.path || newData.path !== path) return;

        const newScheduledGoals = newData.goals.filter((e: Entry) => e.scheduled)
        const newGoals = newData.goals.filter((e: Entry) => !e.scheduled)

        setData(prevData => {
            const mergedMap = new Map(
                [...prevData, ...newGoals].map(entry => [entry["id"], entry])
            );
            const mergedArray = Array.from(mergedMap.values());
            return sorted(mergedArray);
        });

        setScheduledData(prevData => {
            const mergedMap = new Map(
                [...prevData, ...newScheduledGoals].map(entry => [entry["id"], entry])
            );
            const mergedArray = Array.from(mergedMap.values());
            return sorted(mergedArray);
        });
    }, [lastMessage, sortAscending, path]);

    useEffect(() => {
        sendMessage(JSON.stringify({
            path: path,
            method: "startup"
        }));
    }, [path, sendMessage]);

    const actionButtons = (
        <ChangeStatus 
            handleSuspend={handleSuspend}
            handleCancel={handleCancel} 
            handleResume={handleResume}
            hasSelection={selectedIds.length > 0}
        />
    );

    return (
        <div className="flex flex-col w-full min-h-0 flex-1 gap-2 p-2 sm:p-3 overflow-hidden">
            <div className="flex-1 min-h-0 overflow-y-auto space-y-3">
                {/* Scheduled - visually distinct */}
                <div className="rounded border border-dashed border-border bg-muted/20 p-2 sm:p-3">
                    <div className="text-xs text-muted-foreground pb-1 mb-1">SCHEDULED</div>
                    <ScheduledGoalTable
                        scheduledData={scheduledData}
                        handleSelect={handleGoalSelect}
                        filteredActor={filteredActor}
                        selectedStatuses={selectedStatuses}
                    />
                </div>

                {/* Active/History - actions are here */}
                <div className="rounded border border-border bg-card p-2 sm:p-3">
                    <GoalTable
                        data={data}
                        handleSelect={handleGoalSelect}
                        handleSelectAll={handleSelectAll}
                        selectedIds={selectedIds}
                        filteredActor={filteredActor}
                        selectedStatuses={selectedStatuses}
                        actions={actionButtons}
                    />
                </div>
            </div>
        </div>
    )
}

export default GoalViewer;

// IMPORTS //
import React, { useState, useEffect, ChangeEvent, ChangeEventHandler } from "react";

// NPM packages
import { SendMessage, ReadyState } from "react-use-websocket";
import Select from "react-select"

// Internal imports
import ConnectionIndicator from "./util/ConnectionIndicator";
import { Button } from "../Button";
import { columns, Option, sortCriteria, statuses } from "./util/constants";

// SUBCOMPONENTS //
type ChangeStatusProps = {
    handleSuspend: () => void,
    handleResume: () => void,
    handleCancel: () => void
}

const ChangeStatus: React.FC<ChangeStatusProps> = ({
    handleSuspend, handleResume, handleCancel
}) => {
    return (
        <div className="flex flex-col gap-2 shadow-md border border-1
                        border-[#d1dbe3] rounded-md p-3">
            <label className="text-lg">Change status</label>
            <div className="flex flex-col justify-center gap-2">
                <Button
                    title="Suspend the selected goals"
                    onClick={handleSuspend}>
                    Suspend
                </Button>
                <Button
                    title="Resume the selected goals"
                    onClick={handleResume}>
                    Resume
                </Button>
                <Button
                    title="Cancel the selected goals"
                    onClick={handleCancel}>
                    Cancel
                </Button>
            </div>
        </div>
    );
};

type SortGoalsProps = {
    sortCriterion: Option,
    setSortCriterion: any,
    sortAscending: boolean,
    handleRadioSelect: any
};

const SortGoals: React.FC<SortGoalsProps> = ({
    sortCriterion, setSortCriterion, sortAscending, handleRadioSelect
}) => {
    return (
        <div className="flex flex-col gap-2 shadow-md border border-1
                        border-[#d1dbe3] rounded-md p-3">
            <label className="text-lg">Sort goals</label>
            <Select
                options={sortCriteria}
                value={sortCriterion}
                //@ts-ignore
                onChange={setSortCriterion}
            />

            <div className="flex flex-col">
                <div className="flex flex-row gap-1">
                    <input type="radio" name="sortOrder" id="ascending"
                        value="ascending" checked={sortAscending}
                        onChange={handleRadioSelect} />
                    <label htmlFor="ascending">Ascending</label>
                </div>

                <div className="flex flex-row gap-1">
                    <input type="radio" name="sortOrder" id="descending"
                        value="descending" checked={!sortAscending}
                        onChange={handleRadioSelect} />
                    <label htmlFor="descending">Descending</label>
                </div>
            </div>
        </div>
    );
};

type FilterGoalsProps = {
    filteredActor: string,
    selectedStatuses: string[],
    onChangeFilteredActor: ChangeEventHandler<HTMLInputElement>,
    onSelectStatus: ChangeEventHandler<HTMLInputElement>
};

const FilterGoals: React.FC<FilterGoalsProps> = (
    { filteredActor, selectedStatuses, onChangeFilteredActor, onSelectStatus }
) => {
    return (
        <div className="flex flex-col gap-2
                        shadow-md border border-1
                        border-[#d1dbe3] rounded-md p-3">
            <label className="text-lg">Filter goals</label>
            <label>Actor</label>
            <input
                type="text"
                value={filteredActor}
                onChange={onChangeFilteredActor}
                className="block box-border w-full rounded mt-1 mb-2 text-sm
                           border border-slate-500 p-2 font-mono"
            />

            <label>Statuses</label>
            {statuses.map((status, index) =>
                <div key={index} className="flex flex-row gap-1">
                    <input type="checkbox" id={status} onChange={onSelectStatus}
                        checked={selectedStatuses.find(element =>
                            element === status) !== undefined} />
                    <label htmlFor={status} >
                        {status.slice(0, 1).toUpperCase() + status.slice(1)}
                    </label>
                </div>
            )}
        </div>
    );
};

type Entry = {
    name: string,
    actor: string,
    currentAction: string,
    status: string,
    start: string,
    end: string,
    priority: number,
    id: number
};

const toArray = (e: Entry) => {
    return [
        e.name,
        e.actor,
        e.currentAction,
        e.status,
        e.start,
        e.end,
        e.priority,
        e.id
    ];
};

type GoalTableProps = {
    data: Entry[],
    handleSelect: any,
    filteredActor: string,
    selectedStatuses: string[]
}

const GoalTable: React.FC<GoalTableProps> = (
    { data, handleSelect, filteredActor, selectedStatuses }
) => {
    return (
        <table className="border border-[#d1dbe3]">
            <caption className="hidden">Goal table</caption>
            <thead>
                <tr className="bg-sky-100">
                    <th className="border border-[#d1dbe3] p-1">
                        Select
                    </th>
                    {columns.map((col, index) =>
                        <th className="border border-[#d1dbe3] p-1"
                            key={index}>
                            {col}
                        </th>
                    )}
                </tr>
            </thead>

            <tbody>
                {data.map((row, index) =>
                (selectedStatuses.find((element) =>
                    element === row.status) !== undefined
                    && row.actor.includes(filteredActor)
                    && <tr
                        className="even:bg-slate-100"
                        key={index}>
                        <td className="border border-[#d1dbe3] px-1 text-center">
                            <input
                                type="checkbox"
                                value={row.id}
                                id={row.id + ""}
                                onChange={handleSelect}
                                className="cursor-pointer"
                            />
                        </td>
                        {toArray(row).map((prop, index) =>
                            <td className="border border-[#d1dbe3] px-1"
                                key={index}>
                                <label
                                    htmlFor={row.id + ""}
                                    className="cursor-pointer">
                                    {prop + ""}
                                </label>
                            </td>
                        )}
                    </tr>)
                )}
            </tbody>
        </table>
    );
};

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState
};

/**
 * GoalViewer. Defines a component which shows a dynamic table of robot goals
 * and has options for filtering and performing operations on them.
 * @author Lucien Bao
 * @version 1.0
 */
const GoalViewer: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState
}) => {
    // STATE & CALLBACKS //
    // SortGoals
    const [sortCriterion, setSortCriterion] = useState<Option>(sortCriteria[0]);
    const [sortAscending, setSortAscending] = useState<boolean>(true);
    // FilterGoals
    const [filteredActor, setFilteredActor] = useState<string>("");
    const [selectedStatuses, setSelectedStatuses] = useState<string[]>(statuses);
    // GoalTable
    const [data, setData] = useState<Entry[]>([]);
    const [selectedIds, setSelectedIds] = useState<number[]>([]);

    // CALLBACKS //
    const handleGoalSelect = (e: React.ChangeEvent<HTMLInputElement>) => {
        if (e.target.checked)
            // `+` will convert string to number
            setSelectedIds(selectedIds.concat(+e.target.value))
        else
            setSelectedIds(selectedIds.filter((element) =>
                element !== +e.target.value))
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
    };

    const handleRadioSelect = (e) => {
        setSortAscending(e.target.value === "ascending");
    };

    const handleActorSelect = (e: ChangeEvent<HTMLInputElement>) => {
        setFilteredActor(e.target.value);
    };

    const handleStatusSelect = (e: ChangeEvent<HTMLInputElement>) => {
        if (e.target.checked)
            setSelectedStatuses(selectedStatuses.concat(e.target.id))
        else
            setSelectedStatuses(selectedStatuses.filter((element) =>
                element !== e.target.id))
    };

    // INPUT //
    useEffect(() => {
        const sorted = (array: Entry[]) => {
            const compareRaw = (entry0: Entry, entry1: Entry) => {
                const propName = sortCriterion.value;
                if (propName === "priority" || propName === "id")
                    return entry0[propName] - entry1[propName];
                if (entry0[propName] < entry1[propName])
                    return -1;
                if (entry0[propName] > entry1[propName])
                    return 1;
                return 0;
            };

            const compare = (entry0: Entry, entry1: Entry) => {
                return sortAscending ? compareRaw(entry0, entry1)
                    : -compareRaw(entry0, entry1);
            };

            //@ts-ignore
            const copy = array.slice();
            return copy.sort(compare);
        };

        setData(sorted(data));

        if (lastMessage === null) return;
        const newData = JSON.parse(lastMessage.data);
        if (!newData.path || newData.path !== path) return;

        setData(prevData => {
            const mergedMap = new Map(
                [...prevData, ...newData.goals].map(entry => [entry["id"], entry])
            );
            const mergedArray = Array.from(mergedMap.values());
            return sorted(mergedArray);
        });
    }, [lastMessage, sortAscending, sortCriterion, path]);

    useEffect(() => {
        sendMessage(JSON.stringify({
            path: path,
            method: "startup"
        }));
    }, [path, sendMessage]);

    // DOM //
    return (
        <div className="flex flex-col w-full min-h-0 basis-0 grow border
                        border-1 p-5 border-[#d1dbe3] justify-stretch
                        shadow-md rounded-md gap-5">
            <div className="shadow-md border border-1 border-[#d1dbe3]
                            p-5 flex flex-col justify-start gap-5
                            rounded-md grow">

                <div className="flex flex-row gap-5 grow min-h-0 basis-0">
                    {/* Filter menu */}
                    <div className="flex flex-col gap-5 overflow-y-auto grow
                                    w-1/4">
                        <ChangeStatus handleSuspend={handleSuspend}
                            handleCancel={handleCancel} handleResume={handleResume} />

                        <SortGoals
                            sortCriterion={sortCriterion}
                            setSortCriterion={setSortCriterion}
                            sortAscending={sortAscending}
                            handleRadioSelect={handleRadioSelect}
                        />

                        <FilterGoals
                            filteredActor={filteredActor}
                            selectedStatuses={selectedStatuses}
                            onChangeFilteredActor={handleActorSelect}
                            onSelectStatus={handleStatusSelect}
                        />
                    </div>

                    {/* Table */}
                    <div className="shadow-md border border-1 border-[#d1dbe3]
                                    p-3 flex flex-col justify-start gap-3
                                    rounded-md grow overflow-auto w-3/4">
                        <p className="text-lg">Goals</p>
                        <div className="grow flex flex-col min-h-0
                                        basis-0">
                            <GoalTable
                                data={data}
                                handleSelect={handleGoalSelect}
                                filteredActor={filteredActor}
                                selectedStatuses={selectedStatuses}
                            />
                        </div>
                    </div>
                </div>
            </div>

            <ConnectionIndicator readyState={readyState} />
        </div>
    )
}

export default GoalViewer;

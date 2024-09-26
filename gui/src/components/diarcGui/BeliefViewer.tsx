// IMPORTS //
import React, { useState, useEffect, useCallback, FormEvent } from "react";

// NPM packages
import { ReadyState, SendMessage } from "react-use-websocket";
import { Tab, TabList, TabPanel, Tabs } from "react-tabs";
import Select from "react-select"

// Internal imports
import ConnectionIndicator from "./util/ConnectionIndicator";
import { Button } from "../Button";
import { Option } from "./util/constants";
import "./util/StyleOverrides.css";

// CONSTANTS //
const MEMORY_LEVEL_OPTIONS: Option[] = [
    { value: "universal", label: "Universal" },
    { value: "episodic", label: "Episodic" },
    { value: "working", label: "Working" }
]

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState
};

/**
 * BeliefViewer. Defines a component which displays a dynamic list of beliefs
 * and a belief timeline. It also includes an input to query, assert, and
 * retract beliefs.
 * @author Lucien Bao
 * @version 1.0
 */
const BeliefViewer: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState
}) => {
    // SET UP STATE //
    const [desiredMemoryLevel, setDesiredMemoryLevel] =
        useState<Option>(MEMORY_LEVEL_OPTIONS[2]);
    const [currentMemoryLevel, setCurrentMemoryLevel] =
        useState<Option>(MEMORY_LEVEL_OPTIONS[2]);

    const [timelineCleared, setTimelineCleared] = useState<boolean>(false);

    const [timelineBeliefs, setTimelineBeliefs] = useState<string[]>([]);
    const [currentBeliefs, setCurrentBeliefs] = useState<string[]>([]);

    useEffect(() => {
        if (lastMessage === null) return;
        const data = JSON.parse(lastMessage.data);
        if (!data.path || data.path !== path) return;

        if (data.current)
            setCurrentBeliefs(data.current);
        if (data.timeline)
            setTimelineBeliefs(data.timeline);
        if (data.timelineCleared)
            setTimelineCleared(data.timelineCleared);
        if (data.result)
            setResultValue(data.result);

        if (data.timeline && data.timeline.length > 0)
            setTimelineCleared(false);
    }, [lastMessage, path]);

    useEffect(() => {
        sendMessage(JSON.stringify({
            path: path,
            method: "startup"
        }));
    }, [sendMessage, path]);

    // CALLBACKS //
    const generateBeliefTimeline = useCallback(() => {
        return (
            timelineBeliefs.length > 0 ?
                timelineBeliefs
                    .map((value) =>
                        [value.startsWith("assert:"), value.slice()])
                    .map((value, index) =>
                        <p
                            key={index}
                            title={value[1] as string}
                            className={value[0] ? "text-green-600" : "text-red-600"}>
                            {value[1]}
                        </p>
                    )
                : <p>{timelineCleared
                    ? "Belief timeline cleared on memory level change"
                    : "Belief timeline empty"}</p>
        );
    }, [timelineBeliefs, timelineCleared]);

    const generateCurrentBeliefs = useCallback(() => {
        return (
            currentBeliefs.length > 0 ?
                currentBeliefs.map((value, index) =>
                    <p
                        key={index}
                        title={value}>
                        {value}
                    </p>
                )
                : <p>No current beliefs</p>
        );
    }, [currentBeliefs]);

    const [inputValue, setInputValue] = useState<string>("");

    const [resultValue, setResultValue] =
        useState<string>("Query result will appear here.");

    const handleSubmit = (e: FormEvent) => {
        e.preventDefault();
        sendMessage(JSON.stringify({
            "path": path,
            "method": "query",
            "input": inputValue
        }));
        setInputValue("");
    };

    const handleAssert = () => {
        sendMessage(JSON.stringify({
            "path": path,
            "method": "assert",
            "input": inputValue
        }));
        setInputValue("");
    };

    const handleRetract = () => {
        sendMessage(JSON.stringify({
            "path": path,
            "method": "retract",
            "input": inputValue
        }));
        setInputValue("");
    };

    const handleChangeMemoryLevel = () => {
        setCurrentMemoryLevel(desiredMemoryLevel);
        sendMessage(JSON.stringify({
            "path": path,
            "method": "changeMemoryLevel",
            "input": desiredMemoryLevel.value
        }));
    };

    // DOM //
    return (
        <div className="flex flex-col w-full min-h-0 basis-0 grow border
                        border-1 p-5 border-[#d1dbe3] justify-stretch
                        shadow-md rounded-md gap-5">
            <div className="flex flex-row rounded-md shadow-md border border-1
                            p-2 border-[#d1dbe3] gap-5 items-center
                            justify-evenly">
                <label className="grow text-center">
                    Memory level
                </label>
                <Select
                    className="grow"
                    options={MEMORY_LEVEL_OPTIONS}
                    value={desiredMemoryLevel}
                    // @ts-ignore
                    onChange={setDesiredMemoryLevel}
                />
                <Button
                    type="button"
                    className="grow"
                    title="Confirm change of memory level"
                    onClick={handleChangeMemoryLevel}
                    disabled={JSON.stringify(desiredMemoryLevel)
                        === JSON.stringify(currentMemoryLevel)}
                >
                    Change
                </Button>
            </div>

            {/* Main display (mobile) */}
            <Tabs className="flex flex-col min-h-0 grow md:hidden">
                <TabList>
                    <Tab>Belief Timeline</Tab>
                    <Tab>Current Beliefs</Tab>
                </TabList>

                <TabPanel className="flex flex-col min-h-0 grow shrink hidden">
                    <div className="rounded-md shadow-md border border-1 p-2
                                    border-[#d1dbe3] overflow-scroll text-sm
                                    grow">
                        {generateBeliefTimeline()}
                    </div>
                </TabPanel>
                <TabPanel className="flex flex-col min-h-0 grow shrink hidden">
                    <div className="rounded-md shadow-md border border-1 p-2
                                    border-[#d1dbe3] overflow-scroll text-sm
                                    grow">
                        {generateCurrentBeliefs()}
                    </div>
                </TabPanel>
            </Tabs>

            {/* Main display (large screen) */}
            <div className="grow grid-cols-2 gap-5 hidden md:grid min-h-0">
                <div className="grow flex flex-col min-h-0 rounded-md shadow-md
                                border border-1 p-2 border-[#d1dbe3]">
                    <div className="mb-1 text-lg">Belief Timeline</div>
                    <div className="overflow-scroll text-sm min-h-0 grow text-nowrap">
                        {generateBeliefTimeline()}
                    </div>
                </div>
                <div className="grow flex flex-col min-h-0 rounded-md shadow-md
                                border border-1 p-2 border-[#d1dbe3]">
                    <div className="mb-1 text-lg">Current Beliefs</div>
                    <div className="overflow-scroll text-sm min-h-0 grow text-nowrap">
                        {generateCurrentBeliefs()}
                    </div>
                </div>
            </div>

            {/* Menu */}
            <div className="shadow-md border border-1 border-[#d1dbe3]
                            p-3 rounded-md">
                <form onSubmit={(e) => handleSubmit(e)}>
                    <div className="justify-center gap-3 grid grid-cols-1
                                    md:flex md:flex-row md:flex-wrap">
                        <input
                            type="text"
                            className="block box-border rounded text-sm border
                               border-slate-500 font-mono grow p-2
                               overflow-x-auto self-stretch"
                            placeholder="Query/assert/retract..."
                            value={inputValue}
                            onChange={(e) => {
                                setInputValue(e.target.value);
                            }}
                            autoFocus
                            required
                        >
                        </input>
                        <div className="flex flex-row gap-3 flex-wrap justify-center">
                            <Button
                                type="submit"
                                title="Submit query"
                                disabled={inputValue === ""}>
                                Query
                            </Button>
                            <Button
                                type="button"
                                title="Assert belief"
                                onClick={handleAssert}
                                disabled={inputValue === ""}>
                                Assert
                            </Button>
                            <Button
                                type="button"
                                title="Retract belief"
                                onClick={handleRetract}
                                disabled={inputValue === ""}>
                                Retract
                            </Button>
                        </div>
                    </div>
                </form>

                <div
                    title="Query result"
                    className="mt-2 md:mt-1"
                >
                    {resultValue}
                </div>
            </div>

            <ConnectionIndicator readyState={readyState} />
        </div >
    )
}

export default BeliefViewer;

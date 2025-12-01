import React, { useState, useEffect, useCallback, FormEvent } from "react";

import { ReadyState, SendMessage } from "react-use-websocket";
import { Tab, TabList, TabPanel, Tabs } from "react-tabs";
import Select from "react-select"

import ConnectionIndicator from "./util/ConnectionIndicator";
import { Button } from "../Button";
import { Option } from "./util/constants";
import "./util/StyleOverrides.css";

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

const BeliefViewer: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState
}) => {
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
                            className={value[0] ? "text-green-600 dark:text-green-400" : "text-red-600 dark:text-red-400"}>
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

    return (
        <div className="flex flex-col w-full min-h-0 basis-0 grow border
                        border-border p-5 justify-stretch
                        shadow-md rounded-md gap-5 bg-card">
            <div className="flex flex-row rounded-md shadow-md border
                            p-2 border-border gap-5 items-center
                            justify-evenly bg-card">
                <label className="grow text-center">
                    Memory level
                </label>
                <Select
                    className="grow"
                    options={MEMORY_LEVEL_OPTIONS}
                    value={desiredMemoryLevel}
                    // @ts-ignore
                    onChange={setDesiredMemoryLevel}
                    classNames={{
                        control: () => '!bg-background !border-border',
                        menu: () => '!bg-popover !border-border',
                        option: (state) => state.isFocused ? '!bg-accent' : '!bg-popover',
                        singleValue: () => '!text-foreground',
                        input: () => '!text-foreground',
                    }}
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

            <Tabs className="flex flex-col min-h-0 grow md:hidden">
                <TabList>
                    <Tab>Belief Timeline</Tab>
                    <Tab>Current Beliefs</Tab>
                </TabList>

                <TabPanel className="flex flex-col min-h-0 grow shrink hidden">
                    <div className="rounded-md shadow-md border p-2
                                    border-border overflow-scroll text-sm
                                    grow bg-card">
                        {generateBeliefTimeline()}
                    </div>
                </TabPanel>
                <TabPanel className="flex flex-col min-h-0 grow shrink hidden">
                    <div className="rounded-md shadow-md border p-2
                                    border-border overflow-scroll text-sm
                                    grow bg-card">
                        {generateCurrentBeliefs()}
                    </div>
                </TabPanel>
            </Tabs>

            <div className="grow grid-cols-2 gap-5 hidden md:grid min-h-0">
                <div className="grow flex flex-col min-h-0 rounded-md shadow-md
                                border p-2 border-border bg-card">
                    <div className="mb-1 text-lg">Belief Timeline</div>
                    <div className="overflow-scroll text-sm min-h-0 grow text-nowrap">
                        {generateBeliefTimeline()}
                    </div>
                </div>
                <div className="grow flex flex-col min-h-0 rounded-md shadow-md
                                border p-2 border-border bg-card">
                    <div className="mb-1 text-lg">Current Beliefs</div>
                    <div className="overflow-scroll text-sm min-h-0 grow text-nowrap">
                        {generateCurrentBeliefs()}
                    </div>
                </div>
            </div>

            <div className="shadow-md border border-border
                            p-3 rounded-md bg-card">
                <form onSubmit={(e) => handleSubmit(e)}>
                    <div className="justify-center gap-3 grid grid-cols-1
                                    md:flex md:flex-row md:flex-wrap">
                        <input
                            type="text"
                            className="block box-border rounded text-sm border
                               border-input bg-background text-foreground font-mono grow p-2
                               overflow-x-auto self-stretch focus:outline-none focus:ring-2 focus:ring-ring"
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

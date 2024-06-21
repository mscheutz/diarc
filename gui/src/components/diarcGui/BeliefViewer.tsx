/**
 * @author Lucien Bao
 * @version 0.9
 * @date 17 June 2024
 * BeliefViewer. Defines a component which displays a dynamic list of beliefs
 * and a belief timeline. It also includes an input to query, assert, and
 * retract beliefs.
 */

import React, { useState, useEffect, useCallback, FormEvent } from "react";

import useWebSocket from "react-use-websocket";

import { faCirclePlus, faCircleMinus } from "@fortawesome/free-solid-svg-icons"
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";

import ConnectionIndicator from "./ConnectionIndicator";
import { Button } from "../Button";
import { Tab, TabList, TabPanel, Tabs } from "react-tabs";

const GoalViewer: React.FunctionComponent<{}> = () => {
    // SET UP STATE //
    const [timelineBeliefs, setTimelineBeliefs] = useState<string[]>([]);
    const [currentBeliefs, setCurrentBeliefs] = useState<string[]>([]);

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
                : <p>Belief timeline empty</p>
        );
    }, [timelineBeliefs]);

    const generateCurrentBeliefs = useCallback(() => {
        return (
            currentBeliefs.length > 0 ?
                currentBeliefs.map((value, index) => <p key={index}>{value}</p>)
                : <p>No current beliefs</p>
        );
    }, [currentBeliefs]);

    const [inputValue, setInputValue] = useState<string>("");

    const [resultValue, setResultValue] =
        useState<string>("Query result will appear here.");

    // SET UP WEBSOCKET //
    const url: URL = new URL(document.location.toString());
    url.port = "8080";
    url.protocol = "ws";

    const wsBaseUrl = url.toString();
    const { sendMessage, lastMessage, readyState } =
        useWebSocket(`${wsBaseUrl}belief`);

    useEffect(() => {
        if (lastMessage !== null) {
            const data = JSON.parse(lastMessage.data);

            if (data.current)
                setCurrentBeliefs(data.current);
            if (data.timeline)
                setTimelineBeliefs(data.timeline);
            if (data.result)
                setResultValue(data.result);
        }
    }, [lastMessage]);

    const handleSubmit = (e: FormEvent) => {
        e.preventDefault();
        sendMessage(JSON.stringify({
            "method": "query",
            "input": inputValue
        }));
        setInputValue("");
    };

    const handleAssert = () => {
        sendMessage(JSON.stringify({
            "method": "assert",
            "input": inputValue
        }));
        setInputValue("");
    };

    const handleRetract = () => {
        sendMessage(JSON.stringify({
            "method": "retract",
            "input": inputValue
        }));
        setInputValue("");
    };

    return (
        <div className="flex flex-col w-full min-h-0 basis-0 grow outline
                        outline-1 p-5 outline-[#d1dbe3] justify-stretch
                        shadow-md rounded-md gap-5">
            {/* Main display (mobile) */}
            <Tabs className="flex flex-col min-h-0 grow md:hidden">
                <TabList>
                    <Tab>Belief Timeline</Tab>
                    <Tab>Current Beliefs</Tab>
                </TabList>

                <TabPanel className="flex flex-col min-h-0 grow">
                    <div className="rounded-md shadow-md outline outline-1 p-2
                                    outline-[#d1dbe3] overflow-scroll text-sm">
                        {generateBeliefTimeline()}
                    </div>
                </TabPanel>
                <TabPanel className="flex flex-col min-h-0 grow">
                    <div className="rounded-md shadow-md outline outline-1 p-2
                                    outline-[#d1dbe3] overflow-scroll text-sm">
                        {generateCurrentBeliefs()}
                    </div>
                </TabPanel>
            </Tabs>
            {/* Main display (large screen) */}
            <div className="grow grid-cols-2 gap-5 hidden md:grid min-h-0">
                <div className="grow flex flex-col min-h-0 rounded-md shadow-md
                                outline outline-1 p-2 outline-[#d1dbe3]">
                    <div className="mb-1 text-lg">Belief Timeline</div>
                    <div className="overflow-scroll text-sm min-h-0 grow text-nowrap">
                        {generateBeliefTimeline()}
                    </div>
                </div>
                <div className="grow flex flex-col min-h-0 rounded-md shadow-md
                                outline outline-1 p-2 outline-[#d1dbe3]">
                    <div className="mb-1 text-lg">Current Beliefs</div>
                    <div className="overflow-scroll text-sm min-h-0 grow text-nowrap">
                        {generateCurrentBeliefs()}
                    </div>
                </div>
            </div>

            {/* Menu */}
            <div className="shadow-md outline outline-1 outline-[#d1dbe3]
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
                                disabled={inputValue === ""}
                            >
                                Submit
                            </Button>
                            <Button
                                type="button"
                                title="Assert belief (not supported yet)"
                                onClick={handleAssert}
                                disabled={inputValue === ""}
                            >
                                Assert
                            </Button>
                            <Button
                                type="button"
                                title="Retract belief (not supported yet)"
                                onClick={handleRetract}
                                disabled={inputValue === ""}
                            >
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

export default GoalViewer;

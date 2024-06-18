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
    const [beliefTimeline, setBeliefTimeline] = useState<string[]>(["fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs",]);
    const [currentBeliefs, setCurrentBeliefs] = useState<string[]>(["fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs", "fdjsklkfjdsljkfs",]);

    const generateBeliefTimeline = useCallback(() => {
        return (
            beliefTimeline.length > 0 ?
                beliefTimeline.map((value, index) => <p key={index}>{value}</p>)
                : <p>Belief timeline empty</p>
        );
    }, [beliefTimeline]);

    const generateCurrentBeliefs = useCallback(() => {
        return (
            currentBeliefs.length > 0 ?
                currentBeliefs.map((value, index) => <p key={index}>{value}</p>)
                : <p>No current beliefs</p>
        );
    }, [currentBeliefs]);

    const [inputValue, setInputValue] = useState<string>("");

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
            console.debug(data);
        }
    }, [lastMessage]);

    const handleSubmit = (e: FormEvent) => {
        e.preventDefault();
        setInputValue("");
        console.debug("submit");
        // sendMessage(JSON.stringify({
        //     "method": "cancel",
        //     "goalId": selected.id.slice(4) // remove "goal" prefix
        // }))
        // setSelected({ name: "", id: "", type: "" });
    };

    const handleAssert = () => {
        setInputValue("");
        console.debug("assert");
    };

    const handleRetract = () => {
        setInputValue("");
        console.debug("retract")
    };

    return (
        <div className="flex flex-col w-full min-h-0 basis-0 grow outline outline-1 p-5
                        outline-[#d1dbe3] justify-stretch shadow-md rounded-md
                        gap-5">
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
                <div className="rounded-md shadow-md outline outline-1 p-2
                                outline-[#d1dbe3] overflow-scroll text-sm
                                min-h-0 grow">
                    {generateBeliefTimeline()}
                </div>
                <div className="rounded-md shadow-md outline outline-1 p-2
                                outline-[#d1dbe3] overflow-scroll text-sm
                                min-h-0 grow">
                    {generateCurrentBeliefs()}
                </div>
            </div>

            {/* Menu */}
            <form onSubmit={(e) => handleSubmit(e)}>
                <div className="shadow-md outline outline-1 outline-[#d1dbe3]
                            p-3 justify-center gap-3 rounded-md
                            grid grid-cols-1
                            md:flex md:flex-row md:flex-wrap">
                    <input
                        type="text"
                        className="block box-border rounded text-sm border
                               border-slate-500 font-mono grow p-2
                               overflow-x-auto self-stretch"
                        placeholder="Query..."
                        value={inputValue}
                        onChange={(e) => {
                            setInputValue(e.target.value);
                        }}
                    >
                    </input>
                    <div className="flex flex-row gap-3 flex-wrap justify-center">
                        <Button
                            type="submit"
                            title="Submit query"
                        // disabled={!(hasSelected() && selected.type === "active")}
                        >
                            Submit
                        </Button>
                        <Button
                            type="button"
                            title="Assert belief"
                            onClick={handleAssert}
                        // disabled={!(hasSelected() && selected.type === "suspended")}
                        >
                            Assert
                        </Button>
                        <Button
                            type="button"
                            title="Retract belief"
                            onClick={handleRetract}
                        // disabled={!(hasSelected() && selected.type === "suspended")}
                        >
                            Retract
                        </Button>
                    </div>
                </div>
            </form>

            <ConnectionIndicator readyState={readyState} />
        </div >
    )
}

export default GoalViewer;

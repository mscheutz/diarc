/**
 * @author Lucien Bao
 * @version 0.9
 * @date 24 May 2024
 * GoalViewer. Defines a component which shows a dynamic list of robot goals.
 */

import React, { useState, useEffect } from "react";

import useWebSocket from "react-use-websocket";

import {
    faCaretDown,
    faCaretRight,
    faStopwatch,
    faRobot,
    faFlag
} from "@fortawesome/free-solid-svg-icons"
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";

import TreeView, { flattenTree } from "react-accessible-treeview"

import "./util/TreeStyle.css";
import ConnectionIndicator from "./util/ConnectionIndicator";
import { Button } from "../Button";

type Node = {
    name: string,
    id?: number | string,
    children: Node[]
};

type Selection = {
    name: string,
    id: string,
    type: string
};

const GoalViewer: React.FunctionComponent<{}> = () => {
    const [goals, setGoals] = useState<Node>({
        name: "root", // root is not displayed
        id: "root",
        children: [
            {
                id: "active", name: "active", children: []
            },
            {
                id: "suspended", name: "suspended", children: []
            },
            {
                id: "past", name: "past", children: []
            }
        ]
    });

    const url: URL = new URL(document.location.toString());
    url.port = "8080";
    url.protocol = "ws";

    const wsBaseUrl = url.toString();
    const { sendMessage, lastMessage, readyState } =
        useWebSocket(`${wsBaseUrl}goalViewer`);

    useEffect(() => {
        if (lastMessage !== null) {
            const data = JSON.parse(lastMessage.data);
            setGoals(data);
        }
    }, [lastMessage]);

    // Determines if the node with the given id is in the "active" or
    // "suspended" groups.
    const isSelectable = (id: number | string) => {
        const active = goals.children[0];
        for (const agentIndex in active.children) {
            const agent = active.children[agentIndex];
            for (const taskIndex in agent.children) {
                const task = agent.children[taskIndex]
                if (task.id! === id) {
                    return "active";
                }
            }
        }
        const suspended = goals.children[1];
        for (const agentIndex in suspended.children) {
            const agent = suspended.children[agentIndex];
            for (const taskIndex in agent.children) {
                const task = agent.children[taskIndex]
                if (task.id! === id) {
                    return "suspended";
                }
            }
        }
        return "";
    }

    const getAgent = (id: number | string) => {
        const active = goals.children[0];
        for (const agentIndex in active.children) {
            const agent = active.children[agentIndex];
            for (const taskIndex in agent.children) {
                const task = agent.children[taskIndex]
                if (task.id! === id) {
                    return agent.name;
                }
            }
        }
        const suspended = goals.children[1];
        for (const agentIndex in suspended.children) {
            const agent = suspended.children[agentIndex];
            for (const taskIndex in agent.children) {
                const task = agent.children[taskIndex]
                if (task.id! === id) {
                    return agent.name;
                }
            }
        }
        return "";
    };

    const [selected, setSelected] = useState<Selection>({ name: "", id: "", type: "" });
    const handleSelect = (e) => {
        const { element } = e;
        // A goal and not a group or agent
        if ((element.name as string).includes("(")) {
            const selectable = isSelectable(element.id);
            if (selectable !== "") {
                setSelected({ name: element.name, id: element.id, type: selectable });
                return;
            }
        }
        setSelected({ name: "", id: "", type: "" });
    };

    const hasSelected = () => {
        return selected && selected.name && selected.id;
    }

    const handleSuspend = () => {
        const agent = getAgent(selected.id);
        sendMessage(JSON.stringify({
            "method": "suspend",
            "agent": agent,
            "goal": selected.name
        }));
        setSelected({ ...selected, type: "suspended" });
    };

    const handleResume = () => {
        const agent = getAgent(selected.id);
        sendMessage(JSON.stringify({
            "method": "resume",
            "agent": agent,
            "goal": selected.name
        }));
        setSelected({ ...selected, type: "active" });
    };

    const handleCancel = () => {
        // We may assume there is a selected goal since the button is disabled
        // if not
        sendMessage(JSON.stringify({
            "method": "cancel",
            "goalId": selected.id.slice(4) // remove "goal" prefix
        }))
        setSelected({ name: "", id: "", type: "" });
    };

    const getTree = () => {
        return (
            <TreeView
                data={flattenTree(goals)}
                className="basic"
                onNodeSelect={handleSelect}
                nodeRenderer={
                    ({ element, getNodeProps, level, isExpanded, isSelected }) => {
                        return (
                            <div
                                {...getNodeProps()}
                                style={{ paddingLeft: 20 * level - 15 }}
                                title={element.name}
                                className={isSelected ? "selected" : "tree-node"}
                            >
                                <p className="text-nowrap">
                                    {level < 3 && // group or agent (not goal)
                                        (isExpanded
                                            ? <>
                                                <FontAwesomeIcon icon={faCaretDown} />
                                                &nbsp;
                                            </>
                                            : <>
                                                <FontAwesomeIcon icon={faCaretRight} />
                                                &nbsp;
                                            </>)}
                                    {level === 1 &&
                                        <FontAwesomeIcon icon={faStopwatch}
                                            color="#009933" />}
                                    {level === 2 &&
                                        <FontAwesomeIcon icon={faRobot}
                                            color="#4d4dff" />}
                                    {level === 3 &&
                                        <FontAwesomeIcon icon={faFlag}
                                            color="#f00" />}
                                    {" " + element.name}
                                </p>
                            </div>
                        )
                    }
                }
            />
        );
    };

    return (
        <div className="flex flex-col w-full min-h-0 basis-0 grow outline
                        outline-1 p-5 outline-[#d1dbe3] justify-stretch
                        shadow-md rounded-md gap-5">
            {/* Actual lists */}
            <div className="shadow-md grow outline outline-1 overflow-x-scroll
                            overflow-y-scroll outline-[#d1dbe3] overflow-auto 
                            rounded-md min-h-0"
            >
                {getTree()}
            </div>

            {/* Button menu */}
            <div className="shadow-md outline outline-1 outline-[#d1dbe3]
                                p-3 flex flex-row justify-center gap-5 rounded-md"
            >
                <Button
                    title="Suspend the selected active goal"
                    onClick={handleSuspend}
                    disabled={!(hasSelected() && selected.type === "active")}
                >
                    Suspend Goal
                </Button>
                <Button
                    title="Resume the selected suspended goal"
                    onClick={handleResume}
                    disabled={!(hasSelected() && selected.type === "suspended")}
                >
                    Resume Goal
                </Button>
                <Button
                    title="Cancel the selected active goal"
                    onClick={handleCancel}
                    disabled={!hasSelected()}
                >
                    Cancel Goal
                </Button>
            </div>

            <ConnectionIndicator readyState={readyState} />
        </div>
    )
}

export default GoalViewer;

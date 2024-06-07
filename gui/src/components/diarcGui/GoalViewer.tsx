/**
 * @author Lucien Bao
 * @version 0.9
 * @date 24 May 2024
 * GoalView. Defines a component which shows a dynamic list of robot goals.
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

import "./GoalViewer.css";
import ConnectionIndicator from "./ConnectionIndicator";
import { Button } from "../Button";



/*
MONDAY MONDAY MONDAY MONDAY MONDAY

    talk w/ Evan about redundancy of
    "submit goal" form --- custom action
    covers this functionality

MONDAY MONDAY MONDAY MONDAY MONDAY
*/

type Node = {
    name: string,
    id?: number | string,
    children: Node[]
};

type Selection = {
    name: string,
    id: string
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
                id: "past", name: "past", children: []
            }
        ]
    });

    const { sendMessage, lastMessage, readyState } =
        useWebSocket("ws://localhost:8080/goalViewer");

    useEffect(() => {
        if (lastMessage !== null) {
            const data = JSON.parse(lastMessage.data);
            setGoals(data);
        }
    }, [lastMessage]);

    // Determines if the node with the given id is in the "active" group.
    const hasActiveAncestor = (id: number | string) => {
        const active = goals.children[0];
        for (const agentIndex in active.children) {
            const agent = active.children[agentIndex];
            for (const taskIndex in agent.children) {
                const task = agent.children[taskIndex]
                if (task.id! == id) {
                    return true;
                }
            }
        }
        return false;
    }

    const [selected, setSelected] = useState<Selection>({ name: "", id: "" });
    const handleSelect = (e) => {
        const { element } = e;
        // A goal and not a group or agent
        if ((element.name as string).includes("(")) {
            // TODO: when we add suspended category, this should allow
            // suspended task as well
            if (hasActiveAncestor(element.id)) {
                setSelected({ name: element.name, id: element.id });
                return;
            }
        }
        setSelected({ name: "", id: "" });
    };

    const hasSelected = () => {
        return selected && selected.name && selected.id;
    }

    const handleSuspend = () => {

    };

    const handleResume = () => {

    };

    const handleCancel = () => {
        // We may assume there is a selected goal since the button is disabled
        // if not
        sendMessage(JSON.stringify({
            "method": "cancel",
            "goalId": selected.id.slice(4)
        }))
        setSelected({ name: "", id: "" });
    };

    const getTree = () => {
        return (
            <TreeView
                data={flattenTree(goals)}
                className="basic"
                onNodeSelect={handleSelect}
                nodeRenderer={
                    ({ element, getNodeProps, level, isExpanded }) => {
                        return (
                            <div
                                {...getNodeProps()}
                                style={{ paddingLeft: 20 * level - 15 }}
                            >
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
                            </div>
                        )
                    }
                }
            />
        );
    };

    return (
        <div className="flex flex-col w-full h-[40rem] outline outline-1
                        outline-[#d1dbe3] justify-between shadow-md rounded-md">
            <div className="flex flex-col p-5 grow gap-5">
                {/* Header */}
                <div className="text-2xl">DIARC Goal Viewer</div>

                {/* Actual lists */}
                <div className="shadow-md grow outline outline-1
                                outline-[#d1dbe3] p-5 overflow-auto rounded-md"
                >
                    {getTree()}
                </div>

                {/* Button menu */}
                <div className="shadow-md outline outline-1 outline-[#d1dbe3]
                                p-3 flex flex-row justify-center gap-5 rounded-md"
                >
                    {/* TODO */}
                    <Button onClick={handleSuspend} disabled>
                        Suspend Goal
                    </Button>
                    {/* TODO */}
                    <Button onClick={handleResume} disabled>
                        Resume Goal
                    </Button>
                    <Button
                        title="Cancel the selected active goal"
                        onClick={handleCancel} disabled={!hasSelected()}>
                        Cancel Goal
                    </Button>
                </div>

                <ConnectionIndicator readyState={readyState} />
            </div>
        </div>
    )
}

export default GoalViewer;

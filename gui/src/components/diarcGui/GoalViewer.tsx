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
    faFolder,
    faRobot,
    faFlag
} from "@fortawesome/free-solid-svg-icons"
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";

import TreeView, { flattenTree } from "react-accessible-treeview"

import "./GoalViewer.css";
import ConnectionIndicator from "./ConnectionIndicator";

const GoalViewer: React.FunctionComponent<{}> = () => {
    const [goals, setGoals] = useState({
        name: "",
        children: [
            {
                id: "1", name: "active", children: [{
                    id: "2", name: "dempster", children: [{
                        id: "3", name: "goal1"
                    }]
                }]
            },
            {
                id: "4", name: "past", children: [{
                    id: "5", name: "shafer", children: [{
                        id: "6", name: "goal2"
                    }]
                }]
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

    const getTree = () => {
        return (
            <TreeView
                data={flattenTree(goals)}
                className="basic"
                // onNodeSelect={getFileOnSelect}
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
                                    <FontAwesomeIcon icon={faFolder}
                                        color="#ffdc55" />}
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
                        outline-[#d1dbe3] justify-between">
            <div className="flex flex-col p-5 grow">
                {/* Header */}
                <div className="pb-3 text-2xl">DIARC Goal Viewer</div>

                {/* Actual lists */}
                <div className="shadow-md grow">
                    {getTree()}
                </div>
            </div>

            <ConnectionIndicator readyState={readyState} />
        </div>
    )
}

export default GoalViewer;

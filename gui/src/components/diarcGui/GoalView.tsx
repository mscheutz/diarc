/**
 * @author Lucien Bao
 * @version 0.9
 * @date 24 May 2024
 * GoalView. Defines a component which shows a dynamic list of robot goals.
 */

import React, { useState, useEffect } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket";

import { faBan, faSync, faCheck, faQuestion } from '@fortawesome/free-solid-svg-icons'

import NestedList from "../NestedList";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import ConnectionIndicator from "./ConnectionIndicator";

const GoalView: React.FunctionComponent<{}> = () => {
    const [activeGoals, setActiveGoals] = useState({})
    const [pastGoals, setPastGoals] = useState({})

    const [activeDOM, setActiveDOM] = useState(
        <div className="px-5">
            <div className="text-xl mb-2">Active goals</div>
        </div>
    )
    const [pastDOM, setPastDOM] = useState(
        <div className="px-5">
            <div className="text-xl mb-2">Past goals</div>
        </div>
    )

    const createList = (goals, isActive: boolean) => {
        return (
            <div className="px-5">
                <div className="text-xl mb-2">
                    {(isActive ? "Active" : "Past") + " goals"}
                </div>
                {NestedList(goals)}
            </div>
        );
    };

    const { sendMessage, lastMessage, readyState } =
        useWebSocket("ws://localhost:8080/goal");

    useEffect(() => {
        if (lastMessage !== null) {
            const data = JSON.parse(lastMessage.data);
            setActiveGoals(data.active);
            setPastGoals(data.past);
            setActiveDOM(createList(activeGoals, true));
            setPastDOM(createList(pastGoals, false));
        }
    }, [activeGoals.toString(), pastGoals.toString(), lastMessage]);

    return (
        <div className="flex flex-col w-full h-[40rem] outline outline-1
                        outline-[#d1dbe3] justify-between">
            <div>
                {/* Header */}
                <div className="p-5 text-2xl">DIARC Goal Viewer</div>

                {/* Actual lists */}
                <div className="grid grid-cols-2">
                    {activeDOM}
                    {pastDOM}
                </div>
            </div>

            <ConnectionIndicator readyState={readyState} />
        </div>
    )
}

export default GoalView;

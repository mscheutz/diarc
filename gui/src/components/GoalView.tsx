/**
 * @author Lucien Bao
 * @version 0.9
 * @date 24 May 2024
 * GoalView. Defines a component which shows a dynamic list of robot goals.
 */

import React, { useState, useEffect } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket";

import { faBan, faSync, faCheck, faQuestion } from '@fortawesome/free-solid-svg-icons'

import NestedList from "./NestedList";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";

const GoalView: React.FunctionComponent<{}> = () => {
    const [activeGoals, setActiveGoals] = useState({ "thing1": ["thing2", "thing3"] })
    const [pastGoals, setPastGoals] = useState({ "foo": ["bar", "baz"] })

    const [activeDOM, setActiveDOM] = useState(<div></div>)
    const [pastDOM, setPastDOM] = useState(<div></div>)

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
    }, [activeGoals, pastGoals, lastMessage]);

    const connectionStatus = {
        [ReadyState.CONNECTING]: 'connecting',
        [ReadyState.OPEN]: 'connected',
        [ReadyState.CLOSING]: 'connection closing',
        [ReadyState.CLOSED]: 'connection closed',
        [ReadyState.UNINSTANTIATED]: 'uninstantiated',
    }[readyState];

    const statusColor = {
        [ReadyState.CONNECTING]: '#efd402',
        [ReadyState.OPEN]: '#00a505',
        [ReadyState.CLOSING]: '#efd402',
        [ReadyState.CLOSED]: '#e00b00',
        [ReadyState.UNINSTANTIATED]: '#efd402',
    }[readyState]

    const statusIcon = {
        [ReadyState.CONNECTING]: faSync,
        [ReadyState.OPEN]: faCheck,
        [ReadyState.CLOSING]: faSync,
        [ReadyState.CLOSED]: faBan,
        [ReadyState.UNINSTANTIATED]: faQuestion,
    }[readyState]

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

            {/* Connection indicator */}
            <div className='outline outline-1 p-2 outline-[#e5e7eb]
                                        text-center'>
                Status: &nbsp;
                <FontAwesomeIcon icon={statusIcon}
                    color={statusColor}></FontAwesomeIcon>
                {" " + connectionStatus}
            </div>
        </div>
    )
}

export default GoalView;

/**
 * @author Lucien Bao
 * @version 0.9
 * @date 24 May 2024
 * GoalView. Defines a component which shows a dynamic list of robot goals.
 */

import React, { useState, useEffect } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket";

const GoalView: React.FunctionComponent<{}> = () => {
    const [goals, setGoals] = useState(["thing1", "thing2", "thing3"])
    const { sendMessage, lastMessage, readyState } =
        useWebSocket("ws://localhost:8080/goal");

    useEffect(() => {
        if (lastMessage !== null) {
            const data = JSON.parse(lastMessage.data);
            setGoals(data);
        }
    }, [lastMessage]);

    const connectionStatus = {
        [ReadyState.CONNECTING]: 'Connecting',
        [ReadyState.OPEN]: 'Open',
        [ReadyState.CLOSING]: 'Closing',
        [ReadyState.CLOSED]: 'Closed',
        [ReadyState.UNINSTANTIATED]: 'Uninstantiated',
    }[readyState];

    return (
        <div className="flex flex-col size-9/12 outline outline-1">
            <ul>
                {goals.map((goal) => <li>{goal}</li>)}
            </ul>
            {connectionStatus === "Open" ? <p>Open</p> : <p>Not open</p>}
        </div>
    )

    // TODO
    /*
    - send past goals
        - separate active and past goals
    - style
    - connected light
    */
}

export default GoalView;

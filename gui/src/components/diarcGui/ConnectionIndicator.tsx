/**
 * @author Lucien Bao
 * @version 0.9
 * @date 3 June 2024
 * ConnectionIndicator. Defines a component which shows the status of a
 * WebSocket connection.
 */

import React from "react";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { ReadyState } from "react-use-websocket";
import { faBan, faCheck, faQuestion, faSync } from "@fortawesome/free-solid-svg-icons";

const ConnectionIndicator = ({ readyState }) => {
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
        <div className='outline outline-1 p-2 outline-[#e5e7eb]
                        shadow-md text-center w-full'>
            Status: &nbsp;
            <FontAwesomeIcon icon={statusIcon}
                color={statusColor}></FontAwesomeIcon>
            {" " + connectionStatus}
        </div>
    );
};

export default ConnectionIndicator;
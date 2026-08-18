// IMPORTS //
import React from "react";

// NPM packages
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { ReadyState } from "react-use-websocket";
import { faBan, faCheck, faQuestion, faSync } from "@fortawesome/free-solid-svg-icons";
import { Button } from "../../Button";

type Props = {
    readyState: ReadyState
}

/**
 * ConnectionIndicator. Defines a component which shows the status of a
 * WebSocket connection.
 * @author Lucien Bao
 * @version 1.0
 */
const ConnectionIndicator: React.FC<Props> = ({
    readyState
}) => {
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

    // DOM //
    return (
        <div className='outline outline-1 p-2 outline-[#e5e7eb]
                        shadow-md text-center w-full rounded-md flex flex-row
                        items-center justify-center gap-3'>
            <p>
                Status: &nbsp;
                <FontAwesomeIcon icon={statusIcon}
                    color={statusColor}></FontAwesomeIcon>
                {" " + connectionStatus}
            </p>
            {readyState === ReadyState.CLOSED &&
                <Button onClick={() => window.location.reload()}>
                    Reload
                </Button>}
        </div>
    );
};

export default ConnectionIndicator;
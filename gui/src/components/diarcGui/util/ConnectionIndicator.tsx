import React from "react";

import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { ReadyState } from "react-use-websocket";
import { faBan, faCheck, faQuestion, faSync } from "@fortawesome/free-solid-svg-icons";
import { Button } from "../../Button";

type Props = {
    readyState: ReadyState
}

const ConnectionIndicator: React.FC<Props> = ({ readyState }) => {
    const config = {
        [ReadyState.CONNECTING]: {
            label: "Connecting",
            icon: faSync,
            className: "text-amber-600 dark:text-amber-400",
            spin: true
        },
        [ReadyState.OPEN]: {
            label: "Connected",
            icon: faCheck,
            className: "text-green-600 dark:text-green-400",
            spin: false
        },
        [ReadyState.CLOSING]: {
            label: "Closing",
            icon: faSync,
            className: "text-amber-600 dark:text-amber-400",
            spin: true
        },
        [ReadyState.CLOSED]: {
            label: "Disconnected",
            icon: faBan,
            className: "text-red-600 dark:text-red-400",
            spin: false
        },
        [ReadyState.UNINSTANTIATED]: {
            label: "Initializing",
            icon: faQuestion,
            className: "text-muted-foreground",
            spin: false
        },
    }[readyState];

    return (
        <div className="flex items-center gap-2 px-3 py-2 rounded-md border border-border bg-card text-sm">
            <span className="text-muted-foreground">Connection:</span>
            <span className={`flex items-center gap-1.5 ${config.className}`}>
                <FontAwesomeIcon icon={config.icon} className="h-3 w-3" spin={config.spin} />
                {config.label}
            </span>
            {readyState === ReadyState.CLOSED && (
                <Button size="sm" variant="outline" onClick={() => window.location.reload()}>
                    Reload
                </Button>
            )}
        </div>
    );
};

export default ConnectionIndicator;

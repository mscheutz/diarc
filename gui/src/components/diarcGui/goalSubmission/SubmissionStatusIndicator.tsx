import React from "react";

import {faSync, faCheck, faQuestion} from "@fortawesome/free-solid-svg-icons";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";

type Props = {
    goal: string,
    submissionStatus: string
};

const SubmissionStatusIndicator: React.FC<Props> = ({ goal, submissionStatus }) => {
    const isIdle = submissionStatus !== "wait" && submissionStatus !== "successful";
    const isWaiting = submissionStatus === "wait";

    return (
        <div className="flex items-center gap-2 px-3 py-2 rounded-md border border-border bg-card text-sm">
            <span className="text-muted-foreground">Status:</span>
            {isIdle ? (
                <span className="flex items-center gap-1.5 text-muted-foreground">
                    <FontAwesomeIcon icon={faQuestion} className="h-3 w-3 text-blue-500" />
                    Ready
                </span>
            ) : isWaiting ? (
                <span className="flex items-center gap-1.5 text-amber-600 dark:text-amber-400">
                    <FontAwesomeIcon icon={faSync} className="h-3 w-3" spin />
                    Executing <code className="text-xs bg-muted px-1 rounded">{goal}</code>
                </span>
            ) : (
                <span className="flex items-center gap-1.5 text-green-600 dark:text-green-400">
                    <FontAwesomeIcon icon={faCheck} className="h-3 w-3" />
                    <code className="text-xs bg-muted px-1 rounded">{goal}</code> completed
                </span>
            )}
        </div>
    );
}

export default SubmissionStatusIndicator;

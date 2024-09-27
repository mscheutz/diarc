import React from "react";

// Libraries //
import { faSync, faCheck } from "@fortawesome/free-solid-svg-icons";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";

type Props = {
    status: string
};

/**
 * SubmissionStatusIndicator. Lightweight component that shows whether a submission is
 * pending or successful. Used in Goal Submission's form panel to provide
 * feedback on submission, since the form doesn't clear its fields on
 * submission.
 * 
 * @author Lucien Bao
 */
const SubmissionStatusIndicator: React.FC<Props> = ({ status }) => {
    if (status !== "wait" && status !== "successful") return <></>;

    return (
        <div className="rounded-md border border-1 border-[#d1dbe3]
                        flex items-center justify-center px-2 gap-2">
            <p>
                Status:
                {status === "wait" ?
                    " waiting..."
                    : " successful!"
                }
            </p>
            {status === "wait" ?
                <FontAwesomeIcon icon={faSync} color="#efd402" spin />
                : <FontAwesomeIcon icon={faCheck} color="#00a505" />
            }
        </div>
    );
}

export default SubmissionStatusIndicator;

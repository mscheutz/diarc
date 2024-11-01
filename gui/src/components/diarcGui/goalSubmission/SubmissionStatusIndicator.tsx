import React from "react";

// Libraries //
import {faSync, faCheck, faQuestion} from "@fortawesome/free-solid-svg-icons";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";

type Props = {
    goal: string,
    submissionStatus: string
};

/**
 * SubmissionStatusIndicator. Lightweight component that shows whether a submission is
 * pending or successful. Used in Goal Submission's form panel to provide
 * feedback on submission, since the form doesn't clear its fields on
 * submission.
 * 
 * @author Lucien Bao
 */
const SubmissionStatusIndicator: React.FC<Props> = ({ goal, submissionStatus }) => {
    if (submissionStatus !== "wait" && submissionStatus !== "successful") {
        return <div className="rounded-md outline outline-1 outline-[#d1dbe3]
                        flex items-center justify-center px-2 gap-2 w-full shadow-md">
            <p className="text-center">
                Goal submission: &nbsp;
                <FontAwesomeIcon icon={faQuestion} color="#2766e5"/>
                &nbsp;none submitted
            </p>
        </div>
    }

    return (
        <div className="rounded-md outline outline-1 outline-[#d1dbe3]
                        flex items-center justify-center px-2 gap-2 w-full shadow-md">
            <p className="text-center">
                <code>{goal}</code>&nbsp;submission: &nbsp;
                {
                    submissionStatus === "wait" ?
                        <FontAwesomeIcon icon={faSync} color="#efd402" spin/>
                        : <FontAwesomeIcon icon={faCheck} color="#00a505"/>
                }
                {
                    submissionStatus === "wait" ?
                        " waiting..."
                        : " successful!"
                }
            </p>
        </div>
    );
}

export default SubmissionStatusIndicator;

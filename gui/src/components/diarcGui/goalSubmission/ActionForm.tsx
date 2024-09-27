// IMPORTS //
import React, { SetStateAction, useContext } from "react";

// NPM packages
import { useForm } from "react-hook-form";
import { faSync, faCheck } from "@fortawesome/free-solid-svg-icons";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";

// Internal imports
import ActionFormContext from "./ActionFormContext";
import { SendMessage } from "react-use-websocket";

// CONSTANTS //
const textBoxClassName = "block box-border w-full rounded mt-1 mb-2 text-sm "
    + "border border-slate-500 p-2 font-mono";
export { textBoxClassName };

// From Button.tsx
const submitClassName = "bg-slate-900 text-white hover:bg-slate-800 "
    + "dark:bg-slate-200 dark:text-slate-900 dark:hover:bg-slate-100 "
    + "active:scale-95 inline-flex items-center justify-center "
    + "rounded-md text-sm font-medium transition-colors "
    + "focus:outline-none focus:ring-2 focus:ring-slate-400 "
    + "focus:ring-offset-2 dark:focus:ring-slate-400 "
    + "disabled:pointer-events-none dark:focus:ring-offset-slate-900 "
    + "disabled:bg-slate-400 disabled:dark:bg-slate-400 h-10 py-2 px-4 "
    + "cursor-pointer"

type Props = {
    sendMessage: SendMessage,
    path: string,
    submissionStatus: string,
    setSubmissionStatus: React.Dispatch<SetStateAction<string>>
};

/**
* ActionForm. Subcomponent which renders a an action submit form. Users have
* the option to submit a custom action using a text area, or to select an action
* from the ActionBrowser and generate a form with fields for each argument.
* 
* @author Lucien Bao
* @version 1.0
*/
const ActionForm: React.FC<Props> = ({
    sendMessage, path, submissionStatus, setSubmissionStatus
}) => {
    // HOOKS & CALLBACKS //
    const custom = useForm();
    const onSubmitCustom = (data: any) => {
        setSubmissionStatus("wait");
        sendMessage(JSON.stringify(
            {
                type: "custom",
                formData: data,
                path: path
            }
        ));
    };

    const generated = useForm();
    const actionFormContext = useContext(ActionFormContext);
    const generateForm = () => {
        return (
            actionFormContext.length > 0 ?
                actionFormContext.slice(1).map((param, index) => {
                    return (
                        <div key={index}>
                            <label className="font-mono">{param}</label>
                            <input type="text" {...generated.register(param)}
                                className={textBoxClassName} required />
                        </div>
                    );
                })
                : <div className="py-2">No action selected.</div>
        );
    };
    const onSubmitGenerated = (data: any) => {
        setSubmissionStatus("wait");
        let array: string[] = [actionFormContext[0]]
        for (let i = 1; i < actionFormContext.length; i++) {
            array.push(data[actionFormContext[i]]);
        }
        sendMessage(JSON.stringify(
            {
                type: "form",
                formData: array,
                path: path
            }
        ));
    };

    // DOM //
    return (
        <div className="flex flex-col gap-1 w-full border border-[#d1dbe3]
                        rounded-md shadow-md p-3">
            {/* Custom action form */}
            <form
                onSubmit={custom.handleSubmit(onSubmitCustom)}
                className="flex flex-col">

                <label className="text-lg">Custom Action</label>
                <textarea {...custom.register("custom")}
                    className={textBoxClassName} required />

                <input
                    type="submit" value="Submit"
                    className={submitClassName}
                />
            </form>

            <div className="separator text-slate-500 text-center italic my-4">
                ——— OR ———
            </div>

            {/* Generated form */}
            <form
                onSubmit={generated.handleSubmit(onSubmitGenerated)}
                className="flex flex-col"
            >
                <label className="text-lg">
                    Selected Action
                    {/* Looks a bit weird in code but I want to make only
                    part of the label monospace */}
                    {actionFormContext.length > 0
                        ? ": "
                        : ""}
                    {actionFormContext.length > 0
                        ? <span className="font-mono">
                            {actionFormContext[0]}
                        </span>
                        : ""}
                </label>

                {generateForm()}

                {actionFormContext.length > 0
                    && <>
                        <label className="text-sm pt-2 pb-2">
                            All fields are required.
                        </label>
                        <div className="flex flex-row gap-2">
                            <input type="submit" value="Submit"
                                // From Button.tsx
                                className={submitClassName}
                            />
                            {submissionStatus ? (
                                <div className="flex flex-row items-center">
                                    {submissionStatus === "wait" ? (
                                        <FontAwesomeIcon icon={faSync} color="#efd402" spin />
                                    ) : (
                                        <FontAwesomeIcon icon={faCheck} color="#00a505" />
                                    )}
                                </div>)
                                : null
                            }
                        </div>
                    </>}
            </form>
        </div>
    );
};

export default ActionForm;

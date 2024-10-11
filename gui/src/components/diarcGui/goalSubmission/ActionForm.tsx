// IMPORTS //
import React, {SetStateAction, useCallback, useContext} from "react";

// NPM packages
import { useForm } from "react-hook-form";

// Internal imports
import ActionFormContext from "./ActionFormContext";
import { SendMessage } from "react-use-websocket";
import SubmissionStatusIndicator from "./SubmissionStatusIndicator";

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
    const LOCAL_STORAGE_KEY: string = "goalSubmissionPresets";

    // HOOKS & CALLBACKS //
    const addToStorage = (action: string) => {
        const current = window.localStorage.getItem(LOCAL_STORAGE_KEY);
        if(!current) {
            const stored = [action];
            window.localStorage.setItem(LOCAL_STORAGE_KEY, JSON.stringify(stored));
            return;
        }
        let stored: string[] = JSON.parse(current);
        if(stored.includes(action)) return;

        stored.push(action);
        window.localStorage.setItem(LOCAL_STORAGE_KEY, JSON.stringify(stored));
    }

    const custom = useForm();
    const onSaveCustom = useCallback(() => {
        const action: string = custom.getValues("custom");
        addToStorage(action);
        console.log(window.localStorage.getItem(LOCAL_STORAGE_KEY));
    }, []);
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
                            <label className="font-mono">
                                {param}<span className="text-red-600">*</span>
                                <input type="text" {...generated.register(param)}
                                       className={textBoxClassName} required/>
                            </label>
                        </div>
                    );
                })
                : <div className="py-2">No action selected.</div>
        );
    };
    const onSaveGenerate = () => {
        // TODO
    }
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

                <label className="text-lg">
                    Custom Action<span className="text-red-600">*</span>
                    <textarea {...custom.register("custom")}
                              className={textBoxClassName} required/>
                </label>

                <div className="flex flex-row gap-2">
                    <button
                        type="button"
                        className={submitClassName + " grow"}
                        onClick={onSaveCustom}>
                        Save preset
                    </button>
                    <input
                        type="submit" value="Submit"
                        className={submitClassName + " grow"}
                    />
                </div>
            </form>

            <div className="separator text-slate-500 text-center italic my-4">
                ——— OR ———
            </div>

            {/* Generated form */}
            <form
                onSubmit={generated.handleSubmit(onSubmitGenerated)}
                className="flex flex-col gap-1"
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
                    && <div className="flex flex-row gap-2">
                        <button
                            type="button"
                            className={submitClassName + " grow"}
                            onClick={() => console.log("hello")}>
                            Save preset
                        </button>
                        <input type="submit" value="Submit"
                            // From Button.tsx
                            className={submitClassName + " grow"}
                        />
                        <SubmissionStatusIndicator status={submissionStatus} />
                    </div>
                }

                {/* Still show submission status even when
                    no actions are selected */}
                {actionFormContext.length == 0
                    && <SubmissionStatusIndicator status={submissionStatus} />}
            </form>
        </div>
    );
};

export default ActionForm;

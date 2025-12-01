import React, {SetStateAction, useCallback, useContext} from "react";

import { useForm } from "react-hook-form";

import ActionFormContext from "./ActionFormContext";
import { SendMessage } from "react-use-websocket";
import {LOCAL_STORAGE_KEY} from "../util/constants";

const inputClassName = "block w-full rounded-md text-sm border border-input "
    + "bg-background text-foreground px-3 py-2 font-mono placeholder:text-muted-foreground "
    + "focus:outline-none focus:ring-1 focus:ring-primary focus:border-primary transition-colors";

const buttonBase = "inline-flex items-center justify-center gap-1.5 rounded-md text-xs font-medium "
    + "transition-all focus:outline-none focus-visible:ring-2 focus-visible:ring-ring "
    + "disabled:pointer-events-none disabled:opacity-50 h-8 px-3 cursor-pointer ";

const submitClassName = buttonBase + "bg-primary text-primary-foreground hover:bg-primary/90 shadow-sm";
const secondaryClassName = buttonBase + "bg-secondary text-secondary-foreground hover:bg-secondary/70 border border-input";

type Props = {
    sendMessage: SendMessage,
    path: string,
    setLastGoalSubmitted: React.Dispatch<SetStateAction<string>>,
    setSubmissionStatus: React.Dispatch<SetStateAction<string>>
};

const ActionForm: React.FC<Props> = ({
    sendMessage, path, setLastGoalSubmitted, setSubmissionStatus
}) => {
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
        window.dispatchEvent(new Event("localStorage"));
    }, [custom]);
    const onSubmitCustom = (data: any) => {
        const goal: string = data.custom;
        const goalName: string = goal.slice(0, goal.indexOf("("));
        setLastGoalSubmitted(goalName);
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
        if (actionFormContext.length === 0) {
            return (
                <div className="py-4 text-center text-sm text-muted-foreground border border-dashed border-border rounded-md">
                    Select an action from the database
                </div>
            );
        }
        return actionFormContext.slice(1).map((param, index) => (
            <div key={index}>
                <label className="block text-xs font-medium text-muted-foreground mb-1.5 font-mono">
                    {param}
                </label>
                <input
                    type="text"
                    {...generated.register(param)}
                    className={inputClassName}
                    placeholder={`Enter ${param}`}
                    required
                />
            </div>
        ));
    };
    const onSaveGenerated = (data: any) => {
        let action: string = actionFormContext[0] + "(";
        for(let i = 1; i < actionFormContext.length - 1; i++)
            action += data[actionFormContext[i]] + ",";
        action += data[actionFormContext[actionFormContext.length - 1]] + ")";
        addToStorage(action);
        window.dispatchEvent(new Event("localStorage"));
    }
    const onSubmitGenerated = (data: any) => {
        setLastGoalSubmitted(actionFormContext[0]);

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

    return (
        <div className="flex flex-col gap-6 w-full">
            <section className="space-y-3">
                <div className="flex items-center gap-2">
                    <div className="h-4 w-1 rounded-full bg-primary" />
                    <h3 className="text-sm font-semibold text-foreground">Custom Action</h3>
                </div>
                <form onSubmit={custom.handleSubmit(onSubmitCustom)} className="space-y-3">
                    <div>
                        <label className="block text-xs font-medium text-muted-foreground mb-1.5">
                            Action String
                        </label>
                        <textarea
                            {...custom.register("custom")}
                            className={inputClassName + " min-h-[80px] resize-none"}
                            placeholder="actionName(arg1, arg2, ...)"
                            required
                        />
                    </div>
                    <div className="flex gap-2 justify-end">
                        <button type="button" className={secondaryClassName} onClick={onSaveCustom}>
                            Save Preset
                        </button>
                        <input type="submit" value="Execute" className={submitClassName} />
                    </div>
                </form>
            </section>

            <div className="flex items-center gap-3">
                <div className="flex-1 h-px bg-border" />
                <span className="text-xs text-muted-foreground">or select from database</span>
                <div className="flex-1 h-px bg-border" />
            </div>

            <section className="space-y-3">
                <div className="flex items-center gap-2">
                    <div className="h-4 w-1 rounded-full bg-accent" />
                    <h3 className="text-sm font-semibold text-foreground">
                        {actionFormContext.length > 0 ? (
                            <span className="font-mono">{actionFormContext[0]}</span>
                        ) : (
                            "Selected Action"
                        )}
                    </h3>
                </div>
                <form onSubmit={generated.handleSubmit(onSubmitGenerated)} className="space-y-3">
                    {generateForm()}
                    {actionFormContext.length > 0 && (
                        <div className="flex gap-2 justify-end pt-2">
                            <button
                                type="button"
                                className={secondaryClassName}
                                onClick={generated.handleSubmit(onSaveGenerated)}
                            >
                                Save Preset
                            </button>
                            <input type="submit" value="Execute" className={submitClassName} />
                        </div>
                    )}
                </form>
            </section>
        </div>
    );
};

export default ActionForm;

import React, {SetStateAction, useEffect, useState} from "react";

import {SendMessage} from "react-use-websocket";
import {LOCAL_STORAGE_KEY} from "../util/constants";
import {Button} from "../../Button";

type Props = {
    sendMessage: SendMessage,
    path: string,
    setLastGoalSubmitted: React.Dispatch<SetStateAction<string>>,
    setSubmissionStatus: React.Dispatch<SetStateAction<string>>
};

/**
 * ManagePresets. Subcomponent which shows a list of selectable stored
 * presets and buttons to submit and remove them.
 * @author Lucien Bao
 * @version 1.0
 */
const ManagePresets: React.FC<Props> = (
    {sendMessage, path, setLastGoalSubmitted, setSubmissionStatus}
) => {
    const [presetsJSON, setPresetsJSON] = useState<string>("[]");
    useEffect(() => {
        setPresetsJSON(localStorage.getItem(LOCAL_STORAGE_KEY) ?? "[]");
    }, []);
    window.addEventListener("localStorage", () => {
        setPresetsJSON(localStorage.getItem(LOCAL_STORAGE_KEY) ?? "[]");
    });
    const presets: string[] = JSON.parse(presetsJSON);

    //@ts-ignore
    const submitCallback = (e: MouseEvent<HTMLButtonElement, MouseEvent>) => {
        const goal: string = e.target.id.slice(1);
        const goalName: string = goal.slice(0, goal.indexOf("("));
        setLastGoalSubmitted(goalName);
        setSubmissionStatus("wait");
        sendMessage(JSON.stringify(
            {
                type: "custom",
                formData: {
                    custom: goal // get rid of starting 's' char
                },
                path: path
            }
        ));
    }
    //@ts-ignore
    const deleteCallback = (e: MouseEvent<HTMLButtonElement, MouseEvent>) => {
        const id: string = e.target.id.slice(1); // get rid of starting 'd' char
        const removed: string[] = presets.filter((value) => value !== id);
        const newJSON: string = JSON.stringify(removed);
        localStorage.setItem(LOCAL_STORAGE_KEY, newJSON);
        setPresetsJSON(newJSON);
    }

    return (
        <div className="flex flex-col overflow-auto">
            {presets.map((value, index) =>
                <div
                    key={index}
                    className="p-1 flex flex-row items-center justify-between
                               even:bg-slate-200"
                >
                    <p className="font-mono px-1">{value}</p>
                    <div className="flex flex-row gap-2">
                        <Button
                            onClick={submitCallback}
                            id={"s" + value}
                        >
                            Submit
                        </Button>
                        <Button
                            onClick={deleteCallback}
                            id={"d" + value}
                        >
                            Delete
                        </Button>
                    </div>
                </div>
            )}
        </div>
    );
};

export default ManagePresets;

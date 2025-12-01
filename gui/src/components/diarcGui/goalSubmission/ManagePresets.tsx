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
                    custom: goal
                },
                path: path
            }
        ));
    }
    //@ts-ignore
    const deleteCallback = (e: MouseEvent<HTMLButtonElement, MouseEvent>) => {
        const id: string = e.target.id.slice(1);
        const removed: string[] = presets.filter((value) => value !== id);
        const newJSON: string = JSON.stringify(removed);
        localStorage.setItem(LOCAL_STORAGE_KEY, newJSON);
        setPresetsJSON(newJSON);
    }

    return (
        <div className="flex flex-col h-full min-h-0">
            {presets.length === 0 ? (
                <div className="flex-1 flex items-center justify-center p-4">
                    <p className="text-sm text-muted-foreground text-center">
                        No saved presets yet
                    </p>
                </div>
            ) : (
                <div className="flex-1 overflow-auto">
                    {presets.map((value, index) => (
                        <div
                            key={index}
                            className="group flex items-center justify-between gap-2 px-3 py-2
                                       border-b border-border last:border-b-0 hover:bg-muted/50 transition-colors"
                        >
                            <code className="text-xs font-mono text-foreground truncate flex-1">
                                {value}
                            </code>
                            <div className="flex items-center gap-1 opacity-70 group-hover:opacity-100 transition-opacity">
                                <Button onClick={submitCallback} id={"s" + value} size="sm">
                                    Run
                                </Button>
                                <Button
                                    onClick={deleteCallback}
                                    id={"d" + value}
                                    variant="ghost"
                                    size="sm"
                                    className="h-8 w-8 p-0 text-muted-foreground hover:text-destructive"
                                >
                                    {"\u00D7"}
                                </Button>
                            </div>
                        </div>
                    ))}
                </div>
            )}
        </div>
    );
};

export default ManagePresets;

import React, { useState, useEffect, useCallback } from "react";
import { ReadyState, SendMessage } from "react-use-websocket";
import { Button } from "../Button";

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState,
};

const getInitialState = () => {
  const cachedCustomBehaviorCommands = sessionStorage.getItem("customBehaviorCommands");
  return cachedCustomBehaviorCommands ? JSON.parse(cachedCustomBehaviorCommands) : []
}

const MyNewComponent: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState,
}) => {

    const [customBehaviorCommands, setCustomBehaviorCommands] = useState<string[]>(getInitialState);
    const [newBehaviorCommand, setNewBehaviorCommand] = useState<string>("");

    useEffect(() => {
        if(lastMessage === null) return;
        const data = JSON.parse(lastMessage.data);
        if(!data.path || data.path !== path) return;

    }, [path, lastMessage]);

    const handleRemoveCustomBehavior = useCallback((command: string) => {
      const newCommands = customBehaviorCommands.filter(c => c !== command)
      setCustomBehaviorCommands(newCommands)
      sessionStorage.setItem("customBehaviorCommands", JSON.stringify(newCommands))
    }, [customBehaviorCommands]);

    const handleSubmit = useCallback((command: string) => {
        sendMessage(JSON.stringify({
            "path": path,
            "input": command,
            "speaker": "webapp",
            "addressee": "ava"
        }));
    }, [sendMessage, path]);

    const generateCustomBehaviors = useCallback(() => {
        return customBehaviorCommands.map((value, index) => (
            <div key={index} className="relative group rounded-md bg-muted overflow-hidden">
                <button
                    onClick={() => handleSubmit(value)}
                    className="w-full px-3 py-2.5 pr-7 text-xs font-medium hover:bg-muted/70 active:bg-muted/50 transition-colors text-left text-foreground break-words"
                >
                    {value}
                </button>
                <button
                    onClick={(e) => { e.stopPropagation(); handleRemoveCustomBehavior(value); }}
                    className="absolute top-1 right-1 h-5 w-5 flex items-center justify-center rounded
                               text-muted-foreground hover:text-destructive hover:bg-destructive/20
                               transition-colors text-sm font-bold"
                    title="Remove"
                >
                    {"\u00D7"}
                </button>
            </div>
        ));
    }, [customBehaviorCommands, handleSubmit, handleRemoveCustomBehavior]);

    const handleAddCustomBehavior = () => {
      if (newBehaviorCommand.length === 0) return;
        const newCommands = [
                                ...customBehaviorCommands,
                                newBehaviorCommand
                            ]
      setCustomBehaviorCommands(newCommands)
      sessionStorage.setItem("customBehaviorCommands", JSON.stringify(newCommands))
      setNewBehaviorCommand("");
    };

    return (
        <div className="flex flex-col w-full min-h-0 flex-1 gap-4 overflow-auto">
            <div className="w-full space-y-4">
                <div className="flex gap-3">
                    <div className="flex-1 border border-border rounded-md bg-card overflow-hidden">
                        <div className="px-3 py-2 border-b border-border bg-muted/30">
                            <span className="text-xs font-medium text-muted-foreground uppercase tracking-wide">
                                Quick Actions
                            </span>
                        </div>
                        <div className="grid grid-cols-2 sm:grid-cols-3 gap-2 p-2">
                            {[
                                { label: "Pause", cmd: "pause" },
                                { label: "Resume", cmd: "resume" },
                                { label: "Go to Dock", cmd: "go home" },
                                { label: "Schedule Task", cmd: "add a new scheduled task" },
                                { label: "Modify Schedule", cmd: "modify a scheduled task" },
                                { label: "Teach Action", cmd: "i will teach you a new task" },
                            ].map(({ label, cmd }) => (
                                <button
                                    key={cmd}
                                    onClick={() => handleSubmit(cmd)}
                                    className="px-3 py-2.5 text-xs font-medium rounded-md bg-muted hover:bg-muted/70 active:bg-muted/50 transition-colors text-left text-foreground"
                                >
                                    {label}
                                </button>
                            ))}
                        </div>
                    </div>
                    <button
                        onClick={() => handleSubmit("stop")}
                        className="shrink-0 self-stretch px-4 rounded-md border border-destructive/40 
                                   text-xs font-medium text-destructive bg-destructive/5 
                                   hover:bg-destructive hover:text-white hover:border-destructive
                                   transition-colors"
                    >
                        Cancel
                    </button>
                </div>

                <div className="border border-border rounded-md bg-card overflow-hidden">
                    <div className="px-3 py-2 border-b border-border bg-muted/30">
                        <span className="text-xs font-medium text-muted-foreground uppercase tracking-wide">
                            Custom Commands
                        </span>
                    </div>
                    {customBehaviorCommands.length === 0 ? (
                        <div className="p-4 text-center text-sm text-muted-foreground">
                            No custom commands yet
                        </div>
                    ) : (
                        <div className="flex flex-wrap gap-2 p-3">
                            {generateCustomBehaviors()}
                        </div>
                    )}
                    <div className="flex gap-2 p-3 border-t border-border bg-muted/30">
                        <input
                            type="text"
                            className="flex-1 h-8 rounded-md text-sm border border-input bg-background 
                                       text-foreground px-3 placeholder:text-muted-foreground
                                       focus:outline-none focus:ring-1 focus:ring-primary focus:border-primary"
                            placeholder="Add custom command..."
                            value={newBehaviorCommand}
                            onChange={(e) => setNewBehaviorCommand(e.target.value)}
                            onKeyDown={(e) => e.key === "Enter" && handleAddCustomBehavior()}
                        />
                        <Button size="sm" onClick={() => handleAddCustomBehavior()}>
                            Add
                        </Button>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default MyNewComponent;

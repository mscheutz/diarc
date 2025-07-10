import React, { useState, useEffect, useCallback } from "react";
import { ReadyState, SendMessage } from "react-use-websocket";
import { Button } from "../Button";

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState,
    // other inputs that you need
};

const getInitialState = () => {
  const cachedCustomBehaviorCommands = sessionStorage.getItem("customBehaviorCommands");
  return cachedCustomBehaviorCommands ? JSON.parse(cachedCustomBehaviorCommands) : []
}

const MyNewComponent: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState, /* other input names */
}) => {

    const [customBehaviorCommands, setCustomBehaviorCommands] = useState<string[]>(getInitialState);
    const [newBehaviorCommand, setNewBehaviorCommand] = useState<string>("");

    useEffect(() => {
        if(lastMessage === null) return;
        const data = JSON.parse(lastMessage.data);
        if(!data.path || data.path !== path) return;

        // Do stuff with data here
    }, [path, lastMessage]);

    const generateCustomBehaviors = useCallback(() => {
        return (
            customBehaviorCommands
                    .map((value) =>
                        <div className="flex flex-row rounded-md shadow-md border border-1
                                        p-2 border-[#d1dbe3] gap-5 items-center
                                        justify-evenly">
                            <label className="w-1/2 text-center">
                                "{value}"
                            </label>
                            <Button
                                type="button"
                                className="w-5/12"
                                title="Submit Action"
                                onClick={() => handleSubmit(value)}
                            >
                                Submit
                            </Button>
                            <Button
                                type="button"
                                className="w-1/12 text-2xl"
                                onClick={() => handleRemoveCustomBehavior(value)}
                            >
                                -
                            </Button>
                        </div>
                    )
        );
    }, [customBehaviorCommands]);

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

    const handleRemoveCustomBehavior = (command) => {
      const newCommands = customBehaviorCommands.filter(c => c !== command)
      setCustomBehaviorCommands(newCommands)
      sessionStorage.setItem("customBehaviorCommands", JSON.stringify(newCommands))
    };

    const handleSubmit = (command) => {
        sendMessage(JSON.stringify({
            "path": path,
            "input": command,
            "speaker": "evan",
            "addressee": "ava"
        }));
    };

    return (
        <div className="flex flex-col w-full min-h-0 basis-0 grow border
                        border-1 p-5 border-[#d1dbe3] justify-stretch
                        shadow-md rounded-md gap-5 overflow-scroll">

            <div className="flex flex-col w-full basis-0 grow border
                            border-1 p-5 border-[#d1dbe3] justify-stretch
                            shadow-md rounded-md gap-5">

                <div className="w-full text-center font-bold">
                        Predefined Behaviors
                </div>

                <div className="flex flex-row rounded-md shadow-md border border-1
                                p-2 border-[#d1dbe3] gap-5 items-center
                                justify-evenly">
                    <label className="text-center w-1/2">
                        Pause Current Task
                    </label>
                    <Button
                        type="button"
                        className="w-1/2"
                        title="Submit Action"
                        onClick={() => handleSubmit("pause")}
                    >
                        Submit
                    </Button>
                </div>

                <div className="flex flex-row rounded-md shadow-md border border-1
                                p-2 border-[#d1dbe3] gap-5 items-center
                                justify-evenly">
                    <label className="w-1/2 text-center">
                        Resume Current Task
                    </label>
                    <Button
                        type="button"
                        className="w-1/2"
                        title="Submit Action"
                        onClick={() => handleSubmit("resume")}
                    >
                        Submit
                    </Button>
                </div>

                <div className="flex flex-row rounded-md shadow-md border border-1
                                p-2 border-[#d1dbe3] gap-5 items-center
                                justify-evenly">
                    <label className="w-1/2 text-center">
                        Cancel Current Task
                    </label>
                    <Button
                        type="button"
                        className="w-1/2"
                        title="Submit Action"
                        onClick={() => handleSubmit("stop")}
                    >
                        Submit
                    </Button>
                </div>

                <div className="flex flex-row rounded-md shadow-md border border-1
                                p-2 border-[#d1dbe3] gap-5 items-center
                                justify-evenly">
                    <label className="w-1/2 text-center">
                        Go to Dock
                    </label>
                    <Button
                        type="button"
                        className="w-1/2"
                        title="Submit Action"
                        onClick={() => handleSubmit("go home")}
                    >
                        Submit
                    </Button>
                </div>

                <div className="flex flex-row rounded-md shadow-md border border-1
                                p-2 border-[#d1dbe3] gap-5 items-center
                                justify-evenly">
                    <label className="w-1/2 text-center">
                        Begin Scheduling a Task
                    </label>
                    <Button
                        type="button"
                        className="w-1/2"
                        title="Submit Action"
                        onClick={() => handleSubmit("add a new scheduled task")}
                    >
                        Submit
                    </Button>
                </div>

                <div className="flex flex-row rounded-md shadow-md border border-1
                                p-2 border-[#d1dbe3] gap-5 items-center
                                justify-evenly">
                    <label className="w-1/2 text-center">
                        Begin Modifying a Scheduled Task
                    </label>
                    <Button
                        type="button"
                        className="w-1/2"
                        title="Submit Action"
                        onClick={() => handleSubmit("modify a scheduled task")}
                    >
                        Submit
                    </Button>
                </div>

                <div className="flex flex-row rounded-md shadow-md border border-1
                                p-2 border-[#d1dbe3] gap-5 items-center
                                justify-evenly">
                    <label className="w-1/2 text-center">
                        Begin Teaching a New Action
                    </label>
                    <Button
                        type="button"
                        className="w-1/2"
                        title="Submit Action"
                        onClick={() => handleSubmit("i will teach you a new task")}
                    >
                        Submit
                    </Button>
                </div>
            </div>

            <div className="flex flex-col w-full basis-0 grow border
                            border-1 p-5 border-[#d1dbe3] justify-stretch
                            shadow-md rounded-md gap-5">

                <div className="w-full text-center font-bold">
                        Custom Behaviors
                </div>

                {generateCustomBehaviors()}

                <div className="flex flex-row
                                p-2 gap-5 items-center
                                justify-evenly">
                        <input
                            type="text"
                            className="block box-border rounded text-sm border
                               border-slate-500 font-mono grow p-2
                               overflow-x-auto self-stretch"
                            placeholder="Behavior Command..."
                            value={newBehaviorCommand}
                            onChange={(e) => {
                                setNewBehaviorCommand(e.target.value);
                            }}
                            autoFocus
                            required
                        >
                        </input>
                    <Button
                        className="text-2xl"
                        onClick={() => handleAddCustomBehavior(newBehaviorCommand)}>
                        +
                    </Button>
                </div>
            </div>
        </div>
    );
};

export default MyNewComponent;
import React, { useState, useEffect } from "react";
import { ReadyState, SendMessage } from "react-use-websocket";

import TreeView, { flattenTree } from "react-accessible-treeview";

import CreatableSelect from 'react-select/creatable';

import { Button } from "../../Button";
import ConnectionIndicator from "../util/ConnectionIndicator";

import VideoStream from "./VideoStream";

type Node = {
    name: string,
    id: string,
    metadata: {
        running: boolean
        constraints: Option[]
        tracker: Option
    }
};

type Option = {
    value: string,
    label: string,
    category: string
}

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState
};

const VisionSearches: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState
}) => {
    const [searches, setSearches] = useState<Node[]>([]);
    const [selectedSearchId, setSelectedSearchId] = useState<string | null>(null);

    const [displayConstraintOptions, setDisplayConstraintOptions] = useState<Option[]>([]);
    const [selectedConstraints, setSelectedConstraints] = useState<Option[]>([]);

    const [trackerDisplay, setTrackerDisplay] = useState(false);

    const [startDisabled, setStartDisabled] = useState(false);
    const [stopDisabled, setStopDisabled] = useState(true);

    const [forceRender, setForceRender] = useState(false);

    const wrapSearches = () => {
        return {
            name: "root",
            id: "root",
            children: searches
        }
    };
    useEffect(() => {
        if (!lastMessage || !lastMessage.data) return;
        const data = JSON.parse(lastMessage.data);
        if (!data.path || data.path !== path) return;

        if (data.error) {
            console.error("Error from server:", data.details || data.error);
            alert(`Error: ${data.details || data.error}`);
            return;
        }

        switch (data.action) {
            case "configOptions":
                if (data.constraintOptions) {
                    setDisplayConstraintOptions(data.constraintOptions);
                }
                break;
            case "updateSearches":
                setSearches(data.searches || []);
                break;
        }
    }, [path, lastMessage]);

    useEffect(() => {
        sendMessage(JSON.stringify({
            path: path,
            action: "init"
        }))
    }, [path, sendMessage]);

    const handleRemoveSearch = () => {
        if (selectedSearchId === null) return;

        sendMessage(JSON.stringify({
            path: path,
            action: "remove",
            id: selectedSearchId
        }));

        clearSelection();
    };

    const handleSelectSearch = ({ element }: { element: any }) => {
        const selectedSearch = searches.find(search => search.id === element.id);
        if (!selectedSearch) return;

        setSelectedSearchId(element.id);

        setStartDisabled(selectedSearch.metadata.running);
        setStopDisabled(!selectedSearch.metadata.running);
    };

    const handleStartSearch = () => {
        if (selectedSearchId === null) {
            console.warn("No search selected.");
            return;
        }

        sendMessage(JSON.stringify({
            path: path,
            action: "startSearch",
            id: selectedSearchId
        }));

        setStartDisabled(true);
        setStopDisabled(false);
    };

    const handleStopSearch = () => {
        if (selectedSearchId === null) {
            console.warn("No search selected to stop.");
            return;
        }

        sendMessage(JSON.stringify({
            path: path,
            action: "stopSearch",
            id: selectedSearchId
        }));

        setStartDisabled(false);
        setStopDisabled(true);
    };

    const clearSelection = () => {
        setSelectedSearchId(null);
        setSelectedConstraints([]);
        setStartDisabled(true);
        setStopDisabled(true);
        setForceRender(prev => !prev);
    };

    const handleApplyConstraints = () => {
        const constraintsToSend = selectedConstraints.map(constraint => constraint.value);

        sendMessage(JSON.stringify({
            path: path,
            action: "applyConstraints",
            constraints: constraintsToSend
        }));

        clearSelection();
    };

    const handleTrackerDisplayChange = (event: React.ChangeEvent<HTMLInputElement>) => {
        const display = event.target.checked;
        setTrackerDisplay(display);
    };

    const generateEntry = ({ name, metadata }: { name: string; metadata: any }) => {
        return name
            + (metadata && (" [" + metadata.constraints.map((c: any) => " " + c.value) + " ]"))
    };

    const isValidFormat = (inputValue: string) => {
        return displayConstraintOptions.some(option => {
            const regex = new RegExp(`^${option.value}\\(.+\\)$`);
            return regex.test(inputValue);
        });
    };

    const handleCreateOption = (inputValue: string) => {
        if (isValidFormat(inputValue)) {
            const newOption = { value: inputValue, label: inputValue, category: '' };
            setSelectedConstraints(prevState => [...prevState, newOption]);
        } else {
            alert('Invalid format. Must be in the format providedOption(VAR)');
        }
    };

    const isOptionDisabled = (option: Option) => {
        return displayConstraintOptions.some(o => o.value === option.value);
    };

    const customStyles = {
        control: (provided: any) => ({
            ...provided,
            backgroundColor: 'hsl(var(--background))',
            borderColor: 'hsl(var(--border))',
        }),
        menu: (provided: any) => ({
            ...provided,
            backgroundColor: 'hsl(var(--popover))',
            borderColor: 'hsl(var(--border))',
        }),
        option: (provided: any, state: any) => ({
            ...provided,
            color: state.isDisabled ? 'hsl(var(--muted-foreground))' : 'hsl(var(--foreground))',
            backgroundColor: state.isFocused ? 'hsl(var(--accent))' : 'hsl(var(--popover))',
            cursor: 'default'
        }),
        singleValue: (provided: any) => ({
            ...provided,
            color: 'hsl(var(--foreground))',
        }),
        input: (provided: any) => ({
            ...provided,
            color: 'hsl(var(--foreground))',
        }),
        multiValue: (provided: any, state: any) => ({
            ...provided,
            display: state.data.isFixed ? 'none' : provided.display,
            backgroundColor: 'hsl(var(--accent))',
        }),
        multiValueLabel: (provided: any) => ({
            ...provided,
            color: 'hsl(var(--accent-foreground))',
        }),
    };

    return (
        <div className="h-full w-full border shadow-md border-border
                        items-stretch p-5 gap-5 rounded-md justify-items-stretch
                        flex flex-col bg-card">
            <div className="min-h-0 grow items-stretch gap-5
                            justify-items-stretch grid grid-cols-1
                            md:grid-cols-2">

                <VideoStream
                    path={path}
                    sendMessage={sendMessage}
                    lastMessage={lastMessage}
                    readyState={readyState}
                    trackerDisplay={trackerDisplay}
                />

                <div className="flex flex-col gap-5">

                    <div className="border border-border items-left
                                justify-evenly rounded-md p-3 w-full
                                flex flex-col gap-3 bg-card" style={{ flex: '3' }}>
                        <p className="text-lg">Type each descriptor, such as: bottle(Y), white(Y)</p>

                        <CreatableSelect
                            value={selectedConstraints}
                            options={displayConstraintOptions}
                            isMulti
                            className="basic-multi-select"
                            classNamePrefix="select"
                            placeholder="providedOption"
                            onCreateOption={handleCreateOption}
                            isOptionDisabled={isOptionDisabled}
                            styles={customStyles}
                        />

                        <div className="flex flex-col gap-1.5">
                            <Button
                                type="button"
                                title="Add search"
                                onClick={handleApplyConstraints}>
                                Add search
                            </Button>
                        </div>
                    </div>

                    <div className="border border-border items-center
                                justify-items-stretch rounded-md p-3 h-full min-h-0
                                flex flex-col gap-3 bg-card" style={{ flex: '6' }}>
                        <p className="text-lg self-start">Available Searches</p>

                        <div className="w-full flex flex-row gap-5">
                            <div className="flex flex-row gap-1">
                                <Button
                                    type="button"
                                    title="Remove search"
                                    onClick={handleRemoveSearch}>
                                    Remove
                                </Button>
                            </div>

                            <Button
                                type="button"
                                title="Start selected search"
                                onClick={handleStartSearch}
                                disabled={startDisabled}
                                className="grow">
                                Start
                            </Button>

                            <Button
                                type="button"
                                title="Stop selected search"
                                onClick={handleStopSearch}
                                disabled={stopDisabled}
                                className="grow">
                                Stop
                            </Button>
                        </div>

                        <div className="max-h-48 overflow-scroll flex grow w-full
                                    border border-border">
                            {searches.length > 0 ?
                                <div className="min-w-full">
                                    <TreeView
                                        data={flattenTree(wrapSearches() as any)}
                                        className="basic w-full hover:cursor-pointer"
                                        onNodeSelect={handleSelectSearch}
                                        clickAction="EXCLUSIVE_SELECT"
                                        nodeRenderer={({ element, isSelected, getNodeProps }) => (
                                            <div
                                                {...getNodeProps()}
                                                className={"w-full px-1 hover:bg-accent " + (isSelected ? "bg-primary text-primary-foreground" : "")}>
                                                <p
                                                    className={"text-nowrap select-none " + (element.metadata && element.metadata.running ? "font-bold" : "")}
                                                    title={element.name}>
                                                    {generateEntry(element as any)}
                                                </p>
                                            </div>
                                        )}
                                        key={forceRender.toString()}
                                    />
                                </div> : <p className="p-2 text-muted-foreground">No searches to view.</p>
                            }
                        </div>
                    </div>

                    <div className="border border-border items-left
                                justify-evenly rounded-md p-3 w-full
                                flex flex-col gap-3 bg-card" style={{ flex: '1' }}>
                        <p className="text-lg">Tracker</p>

                        <div className="flex flex-row gap-2">
                            <input
                                type="checkbox"
                                id="displayTracker"
                                name="displayTracker"
                                checked={trackerDisplay}
                                onChange={handleTrackerDisplayChange}
                            />
                            <label
                                htmlFor="displayTracker"
                                className="select-none hover:cursor-pointer">
                                Display tracker
                            </label>
                        </div>
                    </div>

                </div>
            </div>

            <ConnectionIndicator readyState={readyState} />
        </div>
    );
};

export default VisionSearches;

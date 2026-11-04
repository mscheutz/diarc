/**
 * @author Lucien Bao
 * @version 0.9
 * @date 20 June 2024
 * VisionSearches. Defines the content of the vision manager's first tab, which
 * manages the vision searches.
 */

import React, { useState, useEffect } from "react";

import TreeView, { flattenTree } from "react-accessible-treeview";

import CreatableSelect from 'react-select/creatable';

import { Button } from "../../Button";
import ConnectionIndicator from "../util/ConnectionIndicator";

import VideoStream from "./VideoStream";

type Node = {
    name: string,
    id: string, // "search" + (ID number)
    metadata: {
        running: boolean
        constraints: Option[]
        tracker: Option
    }
};

type Option = {
    value: string,
    label: string,
    category: string  // "SaliencyProcessor", "ValidationProcessor", "Detector" or "Tracker"
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
    // STATE //
    const [searches, setSearches] = useState<Node[]>([]);
    const [selectedSearchId, setSelectedSearchId] = useState<string | null>(null);

    const [displayConstraintOptions, setDisplayConstraintOptions] = useState<Option[]>([]);
    const [selectedConstraints, setSelectedConstraints] = useState<Option[]>([]);

    const [trackerDisplay, setTrackerDisplay] = useState(false);

    const [startDisabled, setStartDisabled] = useState(false);
    const [stopDisabled, setStopDisabled] = useState(true);

    const [forceRender, setForceRender] = useState(false);

    // UTILITY //
    // Helper function to wrap search array into a tree node.
    const wrapSearches = () => {
        return {
            name: "root",
            id: "root",
            children: searches
        }
    };

    // INPUT //
    // Handle incoming WebSocket messages
    useEffect(() => {
        if (!lastMessage || !lastMessage.data) return;
        const data = JSON.parse(lastMessage.data);
        if (!data.path || data.path !== path) return;
        // DEBUG
//         console.log("Received WebSocket message:", lastMessage.data.substring(0, 30));

        if (data.error) {
            console.error("Error from server:", data.details || data.error);
            alert(`Error: ${data.details || data.error}`);
            // additional logic to handle specific errors, like state rollback or user notifications
            return; // Optionally return here to stop processing any other keys in this message
        }

        switch (data.action) {
            case "configOptions":
                if (data.constraintOptions) {
                    setDisplayConstraintOptions(data.constraintOptions); // Initially, display all options
                }
                break;
            case "updateSearches":
                setSearches(data.searches || []);
                break;
            // Other cases...
        }
    }, [path, lastMessage]);

    // DEBUG
    // Effect to log IDs after updates
//     useEffect(() => {
//         console.log('Current Searches IDs:', searches.map(search => search.id));
//     }, [searches]);

    // Log the value to ensure it changes as expected
//     useEffect(() => {
//         console.log('Selected Search ID:', selectedSearchId);
//     }, [selectedSearchId]);

    useEffect(() => {
        sendMessage(JSON.stringify({
            path: path,
            action: "init"
        }))
    }, [path, sendMessage]);

    // BUTTON CALLBACKS //
    // Remove a search from the list.
    const handleRemoveSearch = () => {
        if (selectedSearchId === null) return;

        sendMessage(JSON.stringify({
            path: path,
            action: "remove",
            id: selectedSearchId
        }));

        // Reset
        clearSelection();
    };

    // Select a search on click.
    const handleSelectSearch = ({ element }) => {
        // Find the search with the matching ID
        const selectedSearch = searches.find(search => search.id === element.id);
        if (!selectedSearch) return;

        setSelectedSearchId(element.id);

        // Update button states based on the running status
        setStartDisabled(selectedSearch.metadata.running);
        setStopDisabled(!selectedSearch.metadata.running);
    };

    // Start the selected search.
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

        //  (dis/en)able start/stop/suspend buttons
        setStartDisabled(true);
        setStopDisabled(false);
    };

    // Stop the selected search.
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

        // Enable the start button and disable the stop button
        setStartDisabled(false);
        setStopDisabled(true);
    };

    // Clear selection
    const clearSelection = () => {
        setSelectedSearchId(null);
        setSelectedConstraints([]);
        setStartDisabled(true);
        setStopDisabled(true);
        setForceRender(prev => !prev);  // Toggle forceRender to force re-render TreeView
    };

    // Apply the selected constraints to the selected search.
    const handleApplyConstraints = () => {

        const constraintsToSend = selectedConstraints.map(constraint => constraint.value);
//         console.log("Applying constraints:", constraintsToSend);

        sendMessage(JSON.stringify({
            path: path,
            action: "applyConstraints",
            constraints: constraintsToSend
        }));

        clearSelection();
    };

    const handleTrackerDisplayChange = (event) => {
        const display = event.target.checked;
        setTrackerDisplay(display);
    };

    // Generate the entry string in the search list
    const generateEntry = ({ name, metadata }) => {
        return name
            + (metadata && (" [" + metadata.constraints.map((c) => " " + c.value) + " ]"))
    };

    const isValidFormat = (inputValue) => {
        // Check if inputValue matches the format predefinedOption(something)
        return displayConstraintOptions.some(option => {
            const regex = new RegExp(`^${option.value}\\(.+\\)$`);
            return regex.test(inputValue);
        });
    };

    const handleCreateOption = (inputValue) => {
        if (isValidFormat(inputValue)) {
            const newOption = { value: inputValue, label: inputValue };
            setSelectedConstraints(prevState => [...prevState, newOption]);
        } else {
            alert('Invalid format. Must be in the format providedOption(VAR)');
        }
    };

    const isOptionDisabled = (option) => {
        // Disable all predefined options
        return displayConstraintOptions.some(o => o.value === option.value);
    };

    const customStyles = {
        option: (provided, state) => ({
            ...provided,
            color: state.isDisabled ? 'black' : provided.color,
            backgroundColor: state.isDisabled ? 'white' : provided.backgroundColor,
            cursor: 'default'
        }),
        multiValue: (provided, state) => ({
            ...provided,
            display: state.data.isFixed ? 'none' : provided.display
        })
    };

    return (
        <div className="h-full w-full border shadow-md border-1 border-[#d1dbe3]
                        items-stretch p-5 gap-5 rounded-md justify-items-stretch
                        flex flex-col">
            <div className="min-h-0 grow items-stretch gap-5
                            justify-items-stretch grid grid-cols-1
                            md:grid-cols-2">

                {/* Video panel */}
                <VideoStream
                    path={path}
                    sendMessage={sendMessage}
                    lastMessage={lastMessage}
                    readyState={readyState}
                    trackerDisplay={trackerDisplay}
                />

                {/* Search options */}
                <div className="flex flex-col gap-5">

                    {/* Constraint options */}
                    <div className="border border-1 border-[#d1dbe3] items-left
                                justify-evenly rounded-md p-3 w-full
                                flex flex-col gap-3" style={{ flex: '3' }}>
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

                    {/* Search list panel */}
                    <div className="border border-1 border-[#d1dbe3] items-center
                                justify-items-stretch rounded-md p-3 h-full min-h-0
                                flex flex-col gap-3" style={{ flex: '6' }}>
                        <p className="text-lg self-start">Available Searches</p>

                        {/* Button bar */}
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
                                    border border-1 border-[#d1dbe3]">
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
                                                className={"w-full px-1 hover:bg-yellow-200 " + (isSelected ? "bg-sky-300" : "")}>
                                                <p
                                                    className={"text-nowrap select-none " + (element.metadata && element.metadata.running ? "font-bold" : "")}
                                                    title={element.name}>
                                                    {generateEntry(element as any)}
                                                </p>
                                            </div>
                                        )}
                                        isSelected={(element) => element.id === selectedSearchId}
                                        key={forceRender}  // Force re-render when forceRender changes
                                    />
                                </div> : <p className="p-2">No searches to view.</p>
                            }
                        </div>
                    </div>

                    {/* Tracker options */}
                    <div className="border border-1 border-[#d1dbe3] items-left
                                justify-evenly rounded-md p-3 w-full
                                flex flex-col gap-3" style={{ flex: '1' }}>
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
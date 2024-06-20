/**
 * @author Lucien Bao
 * @version 0.9
 * @date 20 June 2024
 * VisionSearches. Defines the content of the vision manager's first tab, which
 * manages the vision searches.
 */

import React, { useState, useEffect } from "react";

import { Button } from "../../Button";

const VisionSearches = () => {
    // ----------------^
    // TODO: need to receive props for send message callback and for server
    // messages

    // TODO: useEffect() to handle server messages

    // State for form (Add Searches), top
    const [predefinedValue, setPredefinedValue] = useState<string>("");
    const [predicateValue, setPredicateValue] = useState<string>("");

    const handleAddPredefined = () => {
        console.log(predefinedValue);
        setPredefinedValue("");
        // TODO: send message
    };

    const handleAddPredicate = () => {
        console.log(predicateValue);
        setPredicateValue("");
        // TODO: send message
    };

    // State for manager (Available Searches), bottom
    const [searchList, setSearchList] = useState<string[]>([]);

    return (
        <div className="min-h-0 w-full flex flex-col outline shadow-md outline-1
                        outline-[#d1dbe3] items-center p-5 gap-5 rounded-md
                        justify-items-stretch">
            {/* Add searches */}
            <div className="min-h-0 w-full grid grid-cols-1 outline
                            shadow-md outline-1 outline-[#d1dbe3] items-center
                            p-5 gap-2 rounded-md justify-items-stretch
                            lg:px-40 xl:px-56 2xl:px-72">
                <p className="text-lg">Add Searches</p>
                <div className="grid grid-cols-1 md:grid-cols-2 gap-1 items-center">
                    <label id="predefined-search">Add predefined search</label>
                    <div className="flex flex-row justify-items-stretch gap-3">
                        <select
                            name="predefined-search"
                            className="p-2 grow hover:cursor-pointer"
                            value={predefinedValue}
                            onChange={(e) => setPredefinedValue(e.target.value)}>
                            <option value="">Choose...</option>
                            <option value="1">Option 1</option>
                            <option value="2">Option 2</option>
                        </select>
                        <Button
                            type="button"
                            title="Add predefined search"
                            onClick={handleAddPredefined}
                            disabled={predefinedValue === ""}>
                            Add
                        </Button>
                    </div>
                </div>

                <div className="grid grid-cols-1 md:grid-cols-2 gap-1 items-center">
                    <label
                        id="predicate-search">
                        Add search from predicate
                    </label>
                    <div className="flex flex-row justify-items-stretch gap-3">
                        <input
                            name="predicate-search"
                            type="text"
                            className="block box-border w-full rounded text-sm
                                   border border-slate-500 p-2"
                            placeholder="Predicate..."
                            value={predicateValue}
                            onChange={(e) => setPredicateValue(e.target.value)}
                            required />
                        <Button
                            type="button"
                            title="Add predicate search"
                            onClick={handleAddPredicate}
                            disabled={predicateValue === ""}>
                            Add
                        </Button>
                    </div>
                </div>
            </div>

            {/* Search viewer */}
            <div className="min-h-0 w-full grid grid-cols-1 outline
                        shadow-md outline-1 outline-[#d1dbe3] items-center
                        p-5 gap-2 rounded-md justify-items-stretch
                        lg:px-40 xl:px-56 2xl:px-72">
                <p className="text-lg">Available Searches</p>
                <div className="flex flex-row items-center justify-items-stretch">
                    <div>fjdskls</div>
                </div>
            </div>
        </div>
    );
};

export default VisionSearches;
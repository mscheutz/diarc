/**
 * @author Lucien Bao
 * @version 0.9
 * @date 3 June 2024
 * ActionBrowser. Subcomponent which renders a list view of available actions.
 */

import React from "react";

const ActionBrowser = ({ actionList, setActionFormContext }) => {
    // Returns a string[] whose first el't is the name of the action and any
    // further el'ts are its parameters
    const handleOnClick = (e) => {
        const actionSignature: string = e.target.innerHTML;
        const openParenthesis = actionSignature.indexOf("(");
        const actionName = actionSignature.slice(0, openParenthesis);
        const parameterString = actionSignature.slice(openParenthesis + 1, -1);
        const parameters = parameterString.split(", ");
        setActionFormContext(
            [actionName].concat(
                // take off the "?" at the start of each parameter
                parameters.map((str) => (str.slice(1)))
            )
        );
    };

    return (
        <div className="w-full h-full overflow-auto p-5">
            {actionList.map((item: string, index) => (
                <p
                    key={index}
                    className="w-full hover:bg-[#C6E3FA] px-1 font-mono
                        hover:cursor-pointer text-sm leading-4 py-0.5 truncate"
                    onClick={handleOnClick}
                >
                    {item}
                </p>
            ))}
        </div>
    );
};

export default ActionBrowser;
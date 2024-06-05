/**
 * @author Lucien Bao
 * @version 0.9
 * @date 3 June 2024
 * ActionBrowser. Subcomponent which renders a list view of available actions.
 */

import React from "react";

const ActionBrowser = ({ actionList, setActionFormContext }) => {
    const handleOnClick = (e) => {
        let actionSignature: string = e.target.innerHTML;
        const openParenthesis = actionSignature.indexOf("(");
        actionSignature = actionSignature.slice(openParenthesis + 1, -1);
        const parameters = actionSignature.split(", ");
        // take off the "?" at the start of each parameter
        setActionFormContext(parameters.map((str) => (str.slice(1))));
    };

    return (
        <div className="w-full h-full overflow-auto">
            {actionList.map((item: string, index) => (
                <p
                    key={index}
                    className="w-full hover:bg-[#C6E3FA] px-1
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
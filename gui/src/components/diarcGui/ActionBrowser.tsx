/**
 * @author Lucien Bao
 * @version 0.9
 * @date 3 June 2024
 * ActionBrowser. Subcomponent which renders a list view of available actions.
 */

import React from "react";

const ActionBrowser = ({ actionList }) => {
    return (
        <div className="w-full h-full overflow-auto">
            {actionList.map((item: string, index) => (
                <p key={index} className="w-full hover:bg-[#C6E3FA] px-1
                hover:cursor-pointer text-sm leading-4 py-0.5 truncate">
                    {item}
                </p>
            ))}
        </div>
    );
};

export default ActionBrowser;
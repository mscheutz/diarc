// IMPORTS //
import React from "react";

type Props = {
    groups: string[]
};

/**
 * TableOfContents. Defines a subcomponent which shows a list of links to
 * each group.
 * @author Lucien Bao
 * @version 1.0
 */
const TradeServiceViewer: React.FC<Props> = ({
    groups
}) => {
    return (
        <div className="flex flex-col w-1/6 min-h-0 border
                        border-1 border-[#d1dbe3] justify-stretch shadow-md
                        rounded-md">
            <p className="text-2xl font-bold p-5">Groups</p>
            <div className="flex flex-col p-5 pt-0 gap-2 overflow-auto">
                {groups.map((group, index) =>
                    <a
                        key={index}
                        href={"#" + group}
                        className="text-blue-500 hover:underline"
                    >
                        {group}
                    </a>
                )}
            </div>
        </div>
    );
};

export default TradeServiceViewer;
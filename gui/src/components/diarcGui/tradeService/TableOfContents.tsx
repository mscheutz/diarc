import React from "react";

type Props = {
    groups: string[]
};

const TradeServiceViewer: React.FC<Props> = ({
    groups
}) => {
    return (
        <div className="flex flex-col w-1/6 min-h-0 border
                        border-border justify-stretch shadow-md
                        rounded-md bg-card">
            <p className="text-2xl font-bold p-5">Groups</p>
            <div className="flex flex-col p-5 pt-0 gap-2 overflow-auto">
                {groups.map((group, index) =>
                    <a
                        key={index}
                        href={"#" + group}
                        className="text-primary hover:underline"
                    >
                        {group}
                    </a>
                )}
            </div>
        </div>
    );
};

export default TradeServiceViewer;

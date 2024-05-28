/**
 * @author Lucien Bao
 * @version 0.9
 * @date 28 May 2024
 * NestedList. Defines a two-layer nest list with indenting.
 */

import React from "react";

const InnerList: React.FunctionComponent = (props) => {
    // jankety jank. Could not find a better way to do this.
    let array = props["prop"] as string[];

    return array.map((element) =>
        <li>{element}</li>
    );
}

const NestedList = (props) => {
    const entries = Object.entries(props);

    return (
        <ul className="list-disc list-inside">
            {entries.map((entry) => {
                return (
                    <li>
                        {entry[0]}
                        <ul className="list-disc list-inside pl-5">
                            {/* unspeakable jank :D */}
                            {InnerList({ "prop": entry[1] })}
                        </ul>
                    </li>
                );
            })}
        </ul>
    );
};

export default NestedList;
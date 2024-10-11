import React from "react";

import {useLocalStorage} from "@uidotdev/usehooks";
import {SendMessage} from "react-use-websocket";

type Props = {
    sendMessage: SendMessage,
    path: string
};

/**
 * ManagePresets. Subcomponent which shows a list of selectable stored
 * presets and buttons to submit and remove them.
 * @author Lucien Bao
 * @version 1.0
 */
const ManagePresets: React.FC<Props> = (
    {sendMessage, path}
) => {
    const [presetsJSON, setPresetsJSON] = useLocalStorage(
        "goalSubmissionPresets", "[]");
    const presets: string[] = JSON.parse(presetsJSON);

    // TODO: left off here testing ManagePresets
    return (
        <div>
            {presets.map((value, index) =>
                <div key={index}>
                    value
                </div>
            )}
        </div>
    );
};

export default ManagePresets;

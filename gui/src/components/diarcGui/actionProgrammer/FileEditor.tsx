// IMPORTS //
import React from "react";

// NPM packages
import Editor from "@monaco-editor/react";
import { Button } from "../../Button";
import { SendMessage } from "react-use-websocket";

type Props = {
    fileName: string,
    setFileName: React.Dispatch<React.SetStateAction<string>>,
    fileContents: string,
    setFileContents: React.Dispatch<React.SetStateAction<string>>,
    isCustomFile: boolean,
    setIsCustomFile: React.Dispatch<React.SetStateAction<boolean>>,
    sendMessage: SendMessage,
    path: string
};

/**
 * FileEditor. Subcomponent which renders a code editor for ASL files.
 * @author Lucien Bao
 * @version 1.0
 */
const FileEditor: React.FC<Props> = ({
    fileName, setFileName, fileContents, setFileContents,
    isCustomFile, setIsCustomFile, sendMessage, path
}) => {
    const handleChange = (newValue) => {
        setFileContents(newValue as string);
    };

    const newFile = () => {
        setFileName("custom");
        setFileContents("");
        setIsCustomFile(true);
    };

    const saveAs = () => {
        if (fileName === "") {
            alert("Please edit a file before saving.");
            return;
        }

        const location = prompt(
            "Location to save in custom/ (warning: may overwrite existing files!):",
            isCustomFile ? fileName + ".asl"
                : fileName.slice(0, -4) + "-edited.asl"
        );

        if (location === null) return;

        sendMessage(JSON.stringify({
            path: path,
            method: "editor",
            location: location,
            contents: fileContents
        }));
    };

    // DOM //
    return (
        <div className="w-full h-full py-3
                        border-1 border-[#d1dbe3] border rounded-md
                        shadow-md md:py-5 gap-3 flex flex-col">
            <div className="flex md:flex-row p-3 border border-1 border-[#d1dbe3]
                            rounded-md shadow-md mx-3 md:mx-5 items-center
                            flex-col gap-3">
                {isCustomFile ?
                    <p className="grow">Editing custom file</p>
                    : (fileName === "" ?
                        <p className="italic grow">No file selected</p>
                        : <p className="grow">
                            Editing:&nbsp;
                            <span className="font-mono">{fileName}</span>
                        </p>)}
                <div className="flex flex-row gap-3">
                    <Button
                        type="button"
                        title="New file"
                        onClick={newFile}
                    >
                        New file
                    </Button>
                    <Button
                        type="button"
                        title="Save as..."
                        onClick={saveAs}
                    >
                        Save as...
                    </Button>
                </div>
            </div>
            {fileName !== "" && <Editor
                defaultLanguage="java"
                value={fileContents}
                onChange={handleChange}
            />}
        </div>
    );
};

export default FileEditor;
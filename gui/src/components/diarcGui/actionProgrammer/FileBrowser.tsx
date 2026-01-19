// IMPORTS //
import React from "react";

// NPM packages
import { faFolderClosed, faFolderOpen, faFile } from "@fortawesome/free-solid-svg-icons";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import TreeView, { flattenTree, ITreeViewOnNodeSelectProps } from "react-accessible-treeview";
import { SendMessage } from "react-use-websocket";

// Internal imports
import "../util/TreeStyle.css";

type Props = {
    fileTree: any,
    sendMessage: SendMessage,
    path: string
};

/**
 * FileBrowser. Subcomponent which renders a tree view of ASL files.
 * @author Lucien Bao
 * @version 1.0
 */
const FileBrowser: React.FC<Props> = ({
    fileTree, sendMessage, path
}) => {
    const getFileOnSelect = (e: ITreeViewOnNodeSelectProps) => {
        const element = e.element;
        const name: string = element.name;
        if (name.length < 5 || name.slice(-4) !== ".asl")
            return;

        sendMessage(JSON.stringify({
            path: path,
            method: "browser",
            fileId: element.id
        }));
    }

    // DOM //
    return (
        <div className="w-full h-full 
                        outline outline-1 outline-[#d1dbe3] rounded-l-md
                        shadow-md md:outline-0 md:p-5 gap-3 flex flex-col
                        p-3">
            <p className="text-lg">File Browser</p>

            <TreeView
                data={flattenTree(fileTree)}
                className="basic overflow-auto outline outline-1 grow
                           outline-[#d1dbe3] rounded rounded-md shadow-md"
                onNodeSelect={getFileOnSelect}
                nodeRenderer={
                    ({ element, getNodeProps, level, isBranch, isExpanded, isSelected }) => {
                        return (
                            <div
                                {...getNodeProps()}
                                style={{ paddingLeft: 20 * level - 15 }}
                                title={element.name}
                                className={isSelected ? "selected" : "tree-node"}
                            >
                                <p className="text-nowrap select-none">
                                    {isBranch ?
                                        isExpanded ?
                                            <FontAwesomeIcon icon={faFolderOpen}
                                                color="#006ec2" />
                                            : <FontAwesomeIcon icon={faFolderClosed}
                                                color="#006ec2" />
                                        : <FontAwesomeIcon icon={faFile}
                                            color="#e8be00" />
                                    }
                                    {" " + element.name}
                                </p>
                            </div>
                        )
                    }
                }
            />
        </div>
    );
}

export default FileBrowser;
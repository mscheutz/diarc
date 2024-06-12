/**
 * @author Lucien Bao
 * @version 0.9
 * @date 3 June 2024
 * FileBrowser. Subcomponent which renders a tree view of ASL files.
 */

import React from "react";

import { faFolderClosed, faFolderOpen, faFile } from "@fortawesome/free-solid-svg-icons";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";

import TreeView, { flattenTree } from "react-accessible-treeview";

import "./TreeStyle.css";

const FileBrowser = ({ fileTree }) => {
    return (
        <div className="w-full h-full overflow-x-scroll overflow-y-scroll">
            <TreeView
                data={flattenTree(fileTree)}
                className="basic"
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
/**
 * @author Lucien Bao
 * @version 0.9
 * @date 3 June 2024
 * FileBrowser. Subcomponent which renders a tree view of ASL files.
 */

import React from "react";

import { Tree } from "react-arborist"

const FileBrowser = ({ fileTree, width, height }) => {

    return (
        <Tree
            data={[fileTree]}
            width={width}
            height={height}
        >
            {Node}
        </Tree>
    );
};

const Node = ({ node, style }) => {
    return (
        <div style={style}>
            {node.isLeaf ? "📄" : "📁"}
            {node.data.name}
        </div>
    );
}

export default FileBrowser;
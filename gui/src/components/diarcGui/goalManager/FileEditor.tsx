/**
 * @author Lucien Bao
 * @version 0.9
 * @date 5 June 2024
 * FileEditor. Subcomponent which renders a code editor for ASL files.
 */

import React from "react";

import Editor from "@monaco-editor/react";

const FileEditor = ({ fileContents }) => {
    return (
        <Editor
            defaultLanguage="java"
            value={fileContents}
            options={{ readOnly: true }}
        />
    );
};

export default FileEditor;
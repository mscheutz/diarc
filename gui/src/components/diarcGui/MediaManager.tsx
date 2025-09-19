// @ts-ignore
import React, {useState, useEffect, useCallback, ChangeEvent} from "react";
import { ReadyState, SendMessage } from "react-use-websocket";
import { Button } from "../Button";
import {mediaColumns} from "./util/constants";

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState,
    baseUrl: string,
    // other inputs that you need
};

type Entry = {
    name: string,
    type: string,
};

const toArray = (e: Entry) => {
    return [
        e.name,
        e.type,
    ];
};

type TableProps = {
    data: Entry[],
    handleSelect: any,
}

type FileManagementProps = {
    handleUpload: () => void,
    handleDelete: () => void,
}

const FileManagment: React.FC<FileManagementProps> = ({
                                                       handleUpload, handleDelete
                                                   }) => {
    return (
        <div className="flex flex-row gap-2 shadow-md border border-1
                        border-[#d1dbe3] rounded-md p-3">
            <div className="grow flex flex-row gap-2">
                <Button
                    className="grow"
                    title="Upload a new piece of media"
                    onClick={handleUpload}>
                    Upload
                </Button>
                <Button
                    className="grow"
                    title="Delete selected media"
                    onClick={handleDelete}>
                    Delete
                </Button>
            </div>
        </div>
    );
};

const MediaManager: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState, baseUrl/* other input names */
}) => {

    //STATE & CALLBACKS
    const [data, setData] = useState<Entry[]>([]);
    const [selectedIds, setSelectedIds] = useState<String[]>([]);

    useEffect(() => {

        if (lastMessage === null) return;
        const newData = JSON.parse(lastMessage.data);
        if (!newData.path || newData.path !== path) return;

        console.log(lastMessage.data)
        console.log(newData)

        const media = newData.media;

        setData(media);

    }, [path, lastMessage]);

    const handleDelete = () => {
        sendMessage(JSON.stringify({
            "path": path,
            "method": "delete",
            "ids": selectedIds
        }));
        selectedIds.forEach(id => {
            document.getElementById(id).checked = false;
        })
        setSelectedIds([])
    };

    const handleUpload = () => {
        if (!file) {
            return;
        }

        const formData = new FormData()
        formData.append("file", file);
        fetch(baseUrl + 'api/files/upload', {
            method: 'POST',
            body: formData,
            mode: 'no-cors',
        }).then((res) => res.json())
            .then((data) => console.log(data))
            .catch((err) => console.error(err));

        sendMessage(JSON.stringify({
            path: path,
            method: "refresh"
        }));
    };

    const handleItemSelect = (e: React.ChangeEvent<HTMLInputElement>) => {
        if (e.target.checked)
            setSelectedIds(selectedIds.concat(e.target.value))
        else
            setSelectedIds(selectedIds.filter((element) =>
                element !== e.target.value))
    };

    const [file, setFile] = useState<File>();

    const handleFileChange = (e: ChangeEvent<HTMLInputElement>) => {
        if (e.target.files) {
            setFile(e.target.files[0]);
        }
    };

    const MediaTable: React.FC<TableProps> = (
        { data, handleSelect}
    ) => {
        let videos = [], images = []
        data.forEach((g) => (g.mediaType === "image" ? images : videos).push(g));
        return (
            <div className="grow flex flex-col min-h-0 basis-0">
                <table className="border border-[#d1dbe3]">
                    <caption className="hidden">Goal table</caption>
                    <thead>
                    <tr className="bg-sky-100">
                        <th className="border border-[#d1dbe3] p-1">
                            Select
                        </th>
                        {mediaColumns.map((col, index) =>
                            <th className="border border-[#d1dbe3] p-1"
                                key={index}>
                                {col}
                            </th>
                        )}
                    </tr>
                    </thead>

                    <tbody>
                    {data.map((row, index ) =>
                            <tr
                                className="even:bg-slate-100"
                                key={index}>
                                <td className="border border-[#d1dbe3] px-1 text-center">
                                    <input
                                        type="checkbox"
                                        defaultChecked={selectedIds.includes(row.name)}
                                        value={row.name}
                                        id={row.name}
                                        onChange={handleSelect}
                                        className="cursor-pointer"
                                    />
                                </td>
                                {toArray(row).map((prop, index) =>
                                    <td className="border border-[#d1dbe3] px-1"
                                        key={index}>
                                        <label
                                            htmlFor={row.name}
                                            className="cursor-pointer"
                                            title="">
                                            {prop + ""}&nbsp;
                                        </label>
                                    </td>
                                )}
                            </tr>
                    )}
                    </tbody>
                </table>
            </div>
        );
    };

    useEffect(() => {
        sendMessage(JSON.stringify({
            path: path,
            method: "refresh"
        }));
    }, [path, sendMessage]);

    // @ts-ignore
    return (
        <div className="shadow-md border border-1 border-[#d1dbe3]
                                        p-3 flex flex-col justify-start gap-
                                        rounded-md grow w-3/4">
            <p className="text-lg">Available Media</p>
            <div className="grow flex flex-col overflow-auto min-h-0
                                            basis-0">
                <MediaTable
                    data={data}
                    handleSelect={handleItemSelect}
                />
            </div>
            <input type="file" onChange={handleFileChange} />
            <FileManagment handleUpload={handleUpload} handleDelete={handleDelete}/>
        </div>
    );
};

export default MediaManager;
import React, {useState, useEffect, ChangeEvent, useRef} from "react";
import { ReadyState, SendMessage } from "react-use-websocket";
import { Button } from "../Button";
import {mediaColumns} from "./util/constants";

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState,
    baseUrl: string,
};

type Entry = {
    name: string,
    type: string,
    mediaType?: string,
};

const toArray = (e: Entry) => {
    return [
        e.name,
        e.type,
    ];
};

type TableProps = {
    data: Entry[],
    handleSelect: (e: React.ChangeEvent<HTMLInputElement>) => void,
}

type FileManagementProps = {
    handleUpload: () => void,
    handleDelete: () => void,
    handleFileSelect: () => void,
    selectedFileName: string | null,
    isUploading: boolean,
    uploadStatus: 'idle' | 'success' | 'error',
}

const FileManagement: React.FC<FileManagementProps> = ({
                                                       handleUpload, handleDelete, handleFileSelect, selectedFileName, isUploading, uploadStatus
                                                   }) => {
    return (
        <div className="flex flex-col gap-3 shadow-md border border-1
                        border-[#d1dbe3] rounded-md p-4 bg-white">
            <div className="flex flex-row gap-2 items-center">
                <Button
                    className="px-4 py-2 bg-blue-600 hover:bg-blue-700 text-white font-medium rounded-md transition-colors"
                    title="Select a file to upload"
                    onClick={handleFileSelect}
                    disabled={isUploading}>
                    📁 Choose File
                </Button>
                {selectedFileName && (
                    <span className="text-sm text-gray-700 px-3 py-1 bg-gray-100 rounded-md border border-gray-300">
                        {selectedFileName}
                    </span>
                )}
                {uploadStatus === 'success' && (
                    <span className="text-sm text-green-700 px-3 py-1 bg-green-100 rounded-md border border-green-300">
                        ✓ Upload successful
                    </span>
                )}
                {uploadStatus === 'error' && (
                    <span className="text-sm text-red-700 px-3 py-1 bg-red-100 rounded-md border border-red-300">
                        ✗ Upload failed
                    </span>
                )}
            </div>
            <div className="flex flex-row gap-2">
                <Button
                    className="grow px-4 py-2 bg-green-600 hover:bg-green-700 text-white font-medium rounded-md transition-colors disabled:bg-gray-400 disabled:cursor-not-allowed"
                    title="Upload the selected file"
                    onClick={handleUpload}
                    disabled={!selectedFileName || isUploading}
                    isLoading={isUploading}>
                    ⬆️ Upload
                </Button>
                <Button
                    className="grow px-4 py-2 bg-red-600 hover:bg-red-700 text-white font-medium rounded-md transition-colors"
                    title="Delete selected media"
                    onClick={handleDelete}
                    disabled={isUploading}>
                    🗑️ Delete Selected
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
    const [selectedIds, setSelectedIds] = useState<string[]>([]);
    const [mediaKey, setMediaKey] = useState(0); // ★ bump this to hard-remount table
    const [isUploading, setIsUploading] = useState(false);
    const [uploadStatus, setUploadStatus] = useState<'idle' | 'success' | 'error'>('idle');

    useEffect(() => {

        if (lastMessage === null) return;
        const newData = JSON.parse(lastMessage.data);
        if (!newData.path || newData.path !== path) return;

        console.log(lastMessage.data)
        console.log(newData)

        const media = newData.media;

        setData(media);

        setMediaKey(k => k + 1);

    }, [path, lastMessage]);

    const handleDelete = () => {
        sendMessage(JSON.stringify({
            "path": path,
            "method": "delete",
            "ids": selectedIds
        }));
        selectedIds.forEach(id => {
            const element = document.getElementById(id) as HTMLInputElement | null;
            if (element) {
                element.checked = false;
            }
        });
        setSelectedIds([]);
        setMediaKey(k => k + 1);
    };

    const handleUpload = async () => {
        console.log(baseUrl);
        if (!file) {
            return;
        }

        setIsUploading(true);
        setUploadStatus('idle');

        try {
            const formData = new FormData();
            formData.append("file", file);
            
            await fetch(baseUrl + '/api/files/upload', {
                method: 'POST',
                body: formData,
                mode: 'no-cors',
            });

            setUploadStatus('success');
            console.log('Upload completed');

            setTimeout(() => {
                sendMessage(JSON.stringify({
                    path: path,
                    method: "refresh",
                    nonce: Date.now()
                }));
            }, 400);

            // Clear the file selection after upload
            setFile(null);
            if (fileInputRef.current) {
                fileInputRef.current.value = '';
            }

            setMediaKey(k => k + 1);

            // Clear success message after 3 seconds
            setTimeout(() => {
                setUploadStatus('idle');
            }, 3000);
        } catch (err) {
            console.error('Upload error:', err);
            setUploadStatus('error');
            setTimeout(() => {
                setUploadStatus('idle');
            }, 5000);
        } finally {
            setIsUploading(false);
        }
    };

    const handleItemSelect = (e: React.ChangeEvent<HTMLInputElement>) => {
        if (e.target.checked)
            setSelectedIds(selectedIds.concat(e.target.value))
        else
            setSelectedIds(selectedIds.filter((element) =>
                element !== e.target.value))
    };

    const [file, setFile] = useState<File | null>(null);
    const fileInputRef = useRef<HTMLInputElement>(null);

    const handleFileChange = (e: ChangeEvent<HTMLInputElement>) => {
        if (e.target.files && e.target.files[0]) {
            setFile(e.target.files[0]);
        }
    };

    const handleFileSelect = () => {
        fileInputRef.current?.click();
    };

    const MediaTable: React.FC<TableProps> = (
        { data, handleSelect}
    ) => {
        return (
            <div className="grow flex flex-col min-h-0 basis-0">
                <table className="border border-[#d1dbe3]">
                    <caption className="hidden">Media table</caption>
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

    return (
        <div className="shadow-md border border-1 border-[#d1dbe3]
                                        p-3 flex flex-col justify-start gap-3
                                        rounded-md grow w-3/4">
            <p className="text-lg font-semibold text-gray-800">Available Media</p>
            <div className="grow flex flex-col overflow-auto min-h-0
                                            basis-0">
                <MediaTable
                    key={mediaKey}
                    data={data}
                    handleSelect={handleItemSelect}
                />
            </div>
            <input 
                type="file" 
                ref={fileInputRef}
                onChange={handleFileChange}
                className="hidden"
                accept="image/*,video/*,audio/*"
            />
            <FileManagement 
                handleUpload={handleUpload} 
                handleDelete={handleDelete}
                handleFileSelect={handleFileSelect}
                selectedFileName={file?.name || null}
                isUploading={isUploading}
                uploadStatus={uploadStatus}
            />
        </div>
    );
};

export default MediaManager;
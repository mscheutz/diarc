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
    handleDelete: () => void,
    handleFileSelect: () => void,
    isUploading: boolean,
    uploadStatus: 'idle' | 'success' | 'error',
    hasSelection: boolean,
}

const FileManagement: React.FC<FileManagementProps> = ({
    handleDelete, handleFileSelect, isUploading, uploadStatus, hasSelection
}) => {
    return (
        <div className="flex flex-row gap-2 items-center">
            <Button
                title="Upload a file"
                onClick={handleFileSelect}
                disabled={isUploading}
                isLoading={isUploading}>
                {isUploading ? "Uploading..." : "Upload File"}
            </Button>
            <Button
                variant="destructive"
                title="Delete selected media"
                onClick={handleDelete}
                disabled={isUploading || !hasSelection}>
                Delete Selected
            </Button>
            {uploadStatus === 'success' && (
                <span className="text-sm text-green-700 dark:text-green-400 px-3 py-1 bg-green-100 dark:bg-green-900/30 rounded-md border border-green-300 dark:border-green-700">
                    {"\u2713"} Uploaded
                </span>
            )}
            {uploadStatus === 'error' && (
                <span className="text-sm text-red-700 dark:text-red-400 px-3 py-1 bg-red-100 dark:bg-red-900/30 rounded-md border border-red-300 dark:border-red-700">
                    {"\u2717"} Failed
                </span>
            )}
        </div>
    );
};

const MediaManager: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState, baseUrl
}) => {
    const [data, setData] = useState<Entry[]>([]);
    const [selectedIds, setSelectedIds] = useState<string[]>([]);
    const [mediaKey, setMediaKey] = useState(0);
    const [isUploading, setIsUploading] = useState(false);
    const [uploadStatus, setUploadStatus] = useState<'idle' | 'success' | 'error'>('idle');

    useEffect(() => {
        if (lastMessage === null) return;
        const newData = JSON.parse(lastMessage.data);
        if (!newData.path || newData.path !== path) return;

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

    const uploadFile = async (fileToUpload: File) => {
        setIsUploading(true);
        setUploadStatus('idle');

        try {
            const formData = new FormData();
            formData.append("file", fileToUpload);
            
            await fetch(baseUrl + '/api/files/upload', {
                method: 'POST',
                body: formData,
                mode: 'no-cors',
            });

            setUploadStatus('success');

            setTimeout(() => {
                sendMessage(JSON.stringify({
                    path: path,
                    method: "refresh",
                    nonce: Date.now()
                }));
            }, 400);

            setFile(null);
            if (fileInputRef.current) {
                fileInputRef.current.value = '';
            }

            setMediaKey(k => k + 1);

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

    const handleFileChange = async (e: ChangeEvent<HTMLInputElement>) => {
        if (e.target.files && e.target.files[0]) {
            const selectedFile = e.target.files[0];
            setFile(selectedFile);
            await uploadFile(selectedFile);
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
                <table className="border border-border">
                    <caption className="hidden">Media table</caption>
                    <thead>
                    <tr className="bg-accent">
                        <th className="border border-border p-1">
                            Select
                        </th>
                        {mediaColumns.map((col, index) =>
                            <th className="border border-border p-1"
                                key={index}>
                                {col}
                            </th>
                        )}
                    </tr>
                    </thead>

                    <tbody>
                    {data.map((row, index ) =>
                            <tr
                                className="even:bg-muted"
                                key={index}>
                                <td className="border border-border px-1 text-center">
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
                                    <td className="border border-border px-1"
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
        <div className="shadow-md border border-border
                                        p-3 flex flex-col justify-start gap-3
                                        rounded-md grow w-3/4 bg-card">
            <p className="text-lg font-semibold">Available Media</p>
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
                handleDelete={handleDelete}
                handleFileSelect={handleFileSelect}
                isUploading={isUploading}
                uploadStatus={uploadStatus}
                hasSelection={selectedIds.length > 0}
            />
        </div>
    );
};

export default MediaManager;

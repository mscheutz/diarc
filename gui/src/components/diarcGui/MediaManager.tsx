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
    handleCancelUpload: () => void,
    isUploading: boolean,
    uploadStatus: 'idle' | 'success' | 'error',
    uploadError: string | null,
    uploadProgress: number,
    hasSelection: boolean,
}

const CHUNK_SIZE = 5 * 1024 * 1024;

const ProgressBar: React.FC<{ progress: number }> = ({ progress }) => (
    <div className="flex items-center gap-2 min-w-[200px]">
        <div className="flex-1 h-2 bg-muted rounded-full overflow-hidden">
            <div 
                className="h-full bg-primary transition-all duration-300 ease-out"
                style={{ width: `${progress}%` }}
            />
        </div>
        <span className="text-sm text-muted-foreground min-w-[3ch]">
            {Math.round(progress)}%
        </span>
    </div>
);

const FileManagement: React.FC<FileManagementProps> = ({
    handleDelete, handleFileSelect, handleCancelUpload, isUploading, 
    uploadStatus, uploadError, uploadProgress, hasSelection
}) => {
    return (
        <div className="flex flex-row gap-2 items-center flex-wrap">
            <Button
                title="Upload a file"
                onClick={handleFileSelect}
                disabled={isUploading}
                isLoading={isUploading}>
                {isUploading ? "Uploading..." : "Upload File"}
            </Button>
            {isUploading && (
                <Button
                    variant="outline"
                    title="Cancel upload"
                    onClick={handleCancelUpload}>
                    Cancel
                </Button>
            )}
            <Button
                variant="destructive"
                title="Delete selected media"
                onClick={handleDelete}
                disabled={isUploading || !hasSelection}>
                Delete Selected
            </Button>
            {isUploading && (
                <ProgressBar progress={uploadProgress} />
            )}
            {uploadStatus === 'success' && (
                <span className="text-sm text-green-700 dark:text-green-400 px-3 py-1 bg-green-100 dark:bg-green-900/30 rounded-md border border-green-300 dark:border-green-700">
                    {"\u2713"} Uploaded
                </span>
            )}
            {uploadStatus === 'error' && (
                <span className="text-sm text-red-700 dark:text-red-400 px-3 py-1 bg-red-100 dark:bg-red-900/30 rounded-md border border-red-300 dark:border-red-700">
                    {"\u2717"} {uploadError || 'Upload failed'}
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
    const [uploadError, setUploadError] = useState<string | null>(null);
    const [uploadProgress, setUploadProgress] = useState(0);
    const currentUploadRef = useRef<{ uploadId: string; cancelled: boolean } | null>(null);

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

    const generateUploadId = () => {
        return `${Date.now()}-${Math.random().toString(36).substring(2, 11)}`;
    };

    const uploadFileChunked = async (fileToUpload: File) => {
        const uploadId = generateUploadId();
        currentUploadRef.current = { uploadId, cancelled: false };
        
        setIsUploading(true);
        setUploadStatus('idle');
        setUploadError(null);
        setUploadProgress(0);

        const totalChunks = Math.ceil(fileToUpload.size / CHUNK_SIZE);

        try {
            const initUrl = new URL('api/files/upload/init', baseUrl);
            const initParams = new URLSearchParams({
                uploadId,
                fileName: fileToUpload.name,
                totalChunks: totalChunks.toString(),
            });

            const initResponse = await fetch(`${initUrl}?${initParams}`, {
                method: 'POST',
            });

            if (!initResponse.ok) {
                throw new Error(`Failed to initialize upload: ${initResponse.status}`);
            }

            for (let chunkIndex = 0; chunkIndex < totalChunks; chunkIndex++) {
                if (currentUploadRef.current?.cancelled) {
                    throw new Error('Upload cancelled');
                }

                const start = chunkIndex * CHUNK_SIZE;
                const end = Math.min(start + CHUNK_SIZE, fileToUpload.size);
                const chunk = fileToUpload.slice(start, end);

                const chunkUrl = new URL('api/files/upload/chunk', baseUrl);
                const formData = new FormData();
                formData.append('chunk', chunk, `chunk-${chunkIndex}`);

                const chunkParams = new URLSearchParams({
                    uploadId,
                    chunkIndex: chunkIndex.toString(),
                });

                const chunkResponse = await fetch(`${chunkUrl}?${chunkParams}`, {
                    method: 'POST',
                    body: formData,
                });

                if (!chunkResponse.ok) {
                    throw new Error(`Failed to upload chunk ${chunkIndex}: ${chunkResponse.status}`);
                }

                const progress = ((chunkIndex + 1) / totalChunks) * 95;
                setUploadProgress(progress);
            }

            if (currentUploadRef.current?.cancelled) {
                throw new Error('Upload cancelled');
            }

            const completeUrl = new URL('api/files/upload/complete', baseUrl);
            const completeParams = new URLSearchParams({ uploadId });

            const completeResponse = await fetch(`${completeUrl}?${completeParams}`, {
                method: 'POST',
            });

            if (!completeResponse.ok) {
                throw new Error(`Failed to complete upload: ${completeResponse.status}`);
            }

            setUploadProgress(100);
            setUploadStatus('success');

            setTimeout(() => {
                sendMessage(JSON.stringify({
                    path: path,
                    method: "refresh",
                    nonce: Date.now()
                }));
            }, 400);

            if (fileInputRef.current) {
                fileInputRef.current.value = '';
            }

            setMediaKey(k => k + 1);

            setTimeout(() => {
                setUploadStatus('idle');
                setUploadProgress(0);
            }, 3000);

        } catch (err) {
            console.error('Upload error:', err);
            if (currentUploadRef.current && !currentUploadRef.current.cancelled) {
                try {
                    const cancelUrl = new URL('api/files/upload/cancel', baseUrl);
                    const cancelParams = new URLSearchParams({ uploadId });
                    await fetch(`${cancelUrl}?${cancelParams}`, { method: 'POST' });
                } catch {
                    // Ignore cancel errors
                }
            }

            const isCancelled = err instanceof Error && err.message === 'Upload cancelled';
            
            if (!isCancelled) {
                setUploadStatus('error');
                setUploadError(err instanceof Error ? err.message : 'Upload failed');
                setTimeout(() => {
                    setUploadStatus('idle');
                    setUploadError(null);
                }, 5000);
            } else {
                setUploadStatus('idle');
            }
            setUploadProgress(0);
        } finally {
            setIsUploading(false);
            currentUploadRef.current = null;
        }
    };

    const handleCancelUpload = async () => {
        if (currentUploadRef.current) {
            currentUploadRef.current.cancelled = true;
            try {
                const cancelUrl = new URL('api/files/upload/cancel', baseUrl);
                const cancelParams = new URLSearchParams({ 
                    uploadId: currentUploadRef.current.uploadId 
                });
                await fetch(`${cancelUrl}?${cancelParams}`, { method: 'POST' });
            } catch {
                // Ignore errors
            }
        }
    };

    const handleItemSelect = (e: React.ChangeEvent<HTMLInputElement>) => {
        if (e.target.checked)
            setSelectedIds(selectedIds.concat(e.target.value))
        else
            setSelectedIds(selectedIds.filter((element) =>
                element !== e.target.value))
    };

    const fileInputRef = useRef<HTMLInputElement>(null);

    const handleFileChange = async (e: ChangeEvent<HTMLInputElement>) => {
        if (e.target.files && e.target.files[0]) {
            const selectedFile = e.target.files[0];
            await uploadFileChunked(selectedFile);
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
                handleCancelUpload={handleCancelUpload}
                isUploading={isUploading}
                uploadStatus={uploadStatus}
                uploadError={uploadError}
                uploadProgress={uploadProgress}
                hasSelection={selectedIds.length > 0}
            />
        </div>
    );
};

export default MediaManager;

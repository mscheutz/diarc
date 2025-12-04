import React, {useState, useEffect, ChangeEvent, useRef} from "react";
import { ReadyState, SendMessage } from "react-use-websocket";
import { Button } from "../Button";

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
    <div className="flex items-center gap-2 w-full sm:w-auto sm:min-w-[160px]">
        <div className="flex-1 h-2 bg-muted rounded-full overflow-hidden">
            <div 
                className="h-full bg-primary transition-all duration-300 ease-out"
                style={{ width: `${progress}%` }}
            />
        </div>
        <span className="text-xs text-muted-foreground tabular-nums">
            {Math.round(progress)}%
        </span>
    </div>
);

const FileManagement: React.FC<FileManagementProps> = ({
    handleDelete, handleFileSelect, handleCancelUpload, isUploading, 
    uploadStatus, uploadError, uploadProgress, hasSelection
}) => {
    return (
        <div className="space-y-3 pt-3 border-t border-border">
            {/* Action buttons - responsive grid */}
            <div className="flex flex-col sm:flex-row gap-2">
                <div className="flex gap-2 flex-1">
                    <Button
                        size="sm"
                        className="flex-1 sm:flex-none"
                        title="Upload a file"
                        onClick={handleFileSelect}
                        disabled={isUploading}
                        isLoading={isUploading}>
                        {isUploading ? "Uploading..." : "Upload File"}
                    </Button>
                    {isUploading && (
                        <Button
                            size="sm"
                            variant="outline"
                            title="Cancel upload"
                            onClick={handleCancelUpload}>
                            Cancel
                        </Button>
                    )}
                </div>
                <Button
                    size="sm"
                    variant="destructive"
                    className="w-full sm:w-auto"
                    title="Delete selected media"
                    onClick={handleDelete}
                    disabled={isUploading || !hasSelection}>
                    Delete Selected
                </Button>
            </div>
            
            {/* Progress and status indicators */}
            {isUploading && <ProgressBar progress={uploadProgress} />}
            
            {uploadStatus === 'success' && (
                <span className="text-xs text-muted-foreground">✓ Done</span>
            )}
            {uploadStatus === 'error' && (
                <span className="text-xs text-destructive">✗ {uploadError || 'Failed'}</span>
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


    const MediaList: React.FC<{ items: Entry[], handleSelect: (e: React.ChangeEvent<HTMLInputElement>) => void }> = ({ items, handleSelect }) => {
        if (items.length === 0) {
            return <p className="text-xs text-muted-foreground py-2">—</p>;
        }
        return (
            <div className="space-y-0.5">
                {items.map((row) => (
                    <label
                        key={row.name}
                        className={`flex items-center gap-1.5 py-0.5 px-1 -mx-1 rounded-sm text-xs cursor-pointer hover:bg-muted/50 ${
                            selectedIds.includes(row.name) ? "bg-primary/10" : ""
                        }`}
                    >
                        <input
                            type="checkbox"
                            defaultChecked={selectedIds.includes(row.name)}
                            value={row.name}
                            id={row.name}
                            onChange={handleSelect}
                            className="shrink-0"
                        />
                        <span className="truncate">{row.name}</span>
                    </label>
                ))}
            </div>
        );
    };

    const sortedData = [...data].sort((a, b) => a.name.localeCompare(b.name));
    const images = sortedData.filter(item => item.type.startsWith('image'));
    const videos = sortedData.filter(item => item.type.startsWith('video'));
    const other = sortedData.filter(item => !item.type.startsWith('image') && !item.type.startsWith('video'));

    useEffect(() => {
        sendMessage(JSON.stringify({
            path: path,
            method: "refresh"
        }));
    }, [path, sendMessage]);

    return (
        <div className="flex flex-col w-full min-h-0 flex-1 p-2 sm:p-3 gap-2 overflow-hidden">
            <div className="flex-1 min-h-0 flex flex-col border border-border rounded bg-card overflow-hidden">
                <div className="flex-1 min-h-0 overflow-y-auto p-2 sm:p-3">
                    {selectedIds.length > 0 && (
                        <div className="text-xs text-muted-foreground mb-2">{selectedIds.length} selected</div>
                    )}
                    
                    <div key={mediaKey} className="grid grid-cols-1 sm:grid-cols-2 gap-3">
                        <div>
                            <div className="text-[10px] text-muted-foreground border-b border-border pb-1 mb-1">
                                IMAGES {images.length > 0 && `(${images.length})`}
                            </div>
                            <MediaList items={images} handleSelect={handleItemSelect} />
                        </div>
                        <div>
                            <div className="text-[10px] text-muted-foreground border-b border-border pb-1 mb-1">
                                VIDEOS {videos.length > 0 && `(${videos.length})`}
                            </div>
                            <MediaList items={videos} handleSelect={handleItemSelect} />
                        </div>
                    </div>
                    
                    {other.length > 0 && (
                        <div className="mt-3">
                            <div className="text-[10px] text-muted-foreground border-b border-border pb-1 mb-1">
                                OTHER {`(${other.length})`}
                            </div>
                            <MediaList items={other} handleSelect={handleItemSelect} />
                        </div>
                    )}
                </div>
                
                <input 
                    type="file" 
                    ref={fileInputRef}
                    onChange={handleFileChange}
                    className="hidden"
                    accept="image/*,video/*,audio/*"
                />
                
                <div className="px-2 sm:px-3 pb-2 sm:pb-3 shrink-0">
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
            </div>
        </div>
    );
};

export default MediaManager;

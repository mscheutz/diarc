import React from "react";

type Props = {
    sender: string;
    message: string;
    timestamp: number;
    onOpenChat?: (robotName: string) => void;
    onDismiss?: () => void;
};

const RobotMessageToast: React.FC<Props> = ({ sender, message, timestamp, onOpenChat, onDismiss }) => {
    const handleClick = () => {
        onOpenChat?.(sender);
        onDismiss?.();
    };

    if (!message) return null;

    return (
        <div key={timestamp} className="fixed bottom-8 right-8 w-96 z-50 animate-in slide-in-from-bottom-4 fade-in duration-300">
            <div className="bg-card border border-border rounded-xl shadow-xl overflow-hidden">
                <div className="flex items-center gap-2.5 px-4 py-2.5 border-b border-border bg-muted/50">
                    <svg
                        xmlns="http://www.w3.org/2000/svg"
                        viewBox="0 0 24 24"
                        fill="none"
                        stroke="currentColor"
                        strokeWidth="2"
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        className="h-4 w-4 text-primary"
                    >
                        <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
                    </svg>
                    <span className="text-sm font-medium text-muted-foreground">New Message</span>
                    <button
                        onClick={(e) => { e.stopPropagation(); onDismiss?.(); }}
                        className="ml-auto text-muted-foreground hover:text-foreground transition-colors text-lg leading-none px-1"
                    >
                        {"\u00D7"}
                    </button>
                </div>
                <button
                    onClick={handleClick}
                    className="flex items-start gap-4 p-4 w-full text-left hover:bg-muted/50 transition-colors cursor-pointer"
                >
                    <div className="shrink-0 h-10 w-10 rounded-full bg-primary/10 flex items-center justify-center">
                        <span className="text-sm font-bold text-primary">
                            {sender.charAt(0).toUpperCase()}
                        </span>
                    </div>
                    <div className="flex-1 min-w-0">
                        <p className="text-sm font-semibold text-foreground mb-1">
                            {sender}
                        </p>
                        <p className="text-base text-muted-foreground line-clamp-4">
                            {message}
                        </p>
                    </div>
                </button>
            </div>
        </div>
    );
};

export default RobotMessageToast;


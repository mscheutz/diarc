import React from "react";
import { Button } from "./Button";

interface ResultModalProps {
    result: string | null,  
    changeVisibility: React.Dispatch<React.SetStateAction<boolean>>, 
    isOpen: boolean, 
}

const ResultModal = ({ result, changeVisibility, isOpen } : ResultModalProps) => {
    return (
        <>
            {isOpen && result !== null ? (
                <div className="p-1 w-4/6 h-5/6 bg-gradient-to-r from-cyan-500 to-blue-500 rounded-md dark:bg-gradient-to-r dark:from-cyan-500 dark:to-blue-600">
                    <div className="w-full h-full flex-row items-center justify-center bg-slate-100 p-6 text-black rounded-md shadow-md overflow-scroll dark:bg-slate-900 dark:text-white">
                        <div className="relative">
                            <Button
                                className="absolute top-0 right-0"
                                onClick={() => changeVisibility(!isOpen)}
                            >
                                X
                            </Button>
                        </div>
                        <h1 className="mb-6">Results of query</h1>
                        <div className="text-sm">{result}</div>
                    </div>
                </div>
            ) : (
                <div className="p-1 w-4/6 h-5/6 bg-gradient-to-r from-cyan-500 to-blue-500 rounded-md dark:bg-gradient-to-r dark:from-cyan-500 dark:to-blue-600">
                    <div className="overflow-hidden w-full h-full flex-row items-center justify-center bg-slate-100 p-6 text-black rounded-md shadow-md dark:bg-slate-900 dark:text-white">
                        <div className="relative">
                            <Button
                                className="absolute top-0 right-0"
                                onClick={() => changeVisibility(!isOpen)}
                            >
                                X
                            </Button>
                        </div>
                        <h1 className="mb-6">SOMETHING WENT WRONG</h1>
                        <div className="text-sm">{result}</div>
                    </div>
                </div>
            )}
        </>
    );
};

export default ResultModal;

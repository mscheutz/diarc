import React, { SetStateAction } from "react";

import { useForm } from "react-hook-form";
import { SendMessage } from "react-use-websocket";

const inputClassName = "block w-full rounded-md text-sm border border-input "
    + "bg-background text-foreground px-3 py-2 font-mono placeholder:text-muted-foreground "
    + "focus:outline-none focus:ring-1 focus:ring-primary focus:border-primary transition-colors";

const submitClassName = "inline-flex items-center justify-center gap-1.5 rounded-md text-xs font-medium "
    + "transition-all focus:outline-none focus-visible:ring-2 focus-visible:ring-ring "
    + "disabled:pointer-events-none disabled:opacity-50 h-8 px-3 cursor-pointer "
    + "bg-primary text-primary-foreground hover:bg-primary/90 shadow-sm";

type Props = {
    sendMessage: SendMessage,
    path: string,
    setLastGoalSubmitted: React.Dispatch<SetStateAction<string>>,
    setSubmissionStatus: React.Dispatch<SetStateAction<string>>
};

const GoalForm: React.FC<Props> = ({
    sendMessage, path, setLastGoalSubmitted, setSubmissionStatus
}) => {
    const { register, handleSubmit } = useForm();
    const onSubmitGoal = (data: any) => {
        const goal: string = data.goal;
        setLastGoalSubmitted(goal);

        setSubmissionStatus("wait");
        sendMessage(JSON.stringify(
            {
                type: "goal",
                formData: data,
                path: path
            }
        ));
    };

    return (
        <div className="flex flex-col gap-6 w-full">
            <section className="space-y-3">
                <div className="flex items-center gap-2">
                    <div className="h-4 w-1 rounded-full bg-primary" />
                    <h3 className="text-sm font-semibold text-foreground">Submit Goal</h3>
                </div>
                <form onSubmit={handleSubmit(onSubmitGoal)} className="space-y-3">
                    <div>
                        <label className="block text-xs font-medium text-muted-foreground mb-1.5">
                            Agent
                        </label>
                        <input
                            type="text"
                            defaultValue="self"
                            {...register("agent")}
                            className={inputClassName}
                            required
                        />
                    </div>
                    <div>
                        <label className="block text-xs font-medium text-muted-foreground mb-1.5">
                            Goal
                        </label>
                        <input
                            type="text"
                            {...register("goal")}
                            className={inputClassName}
                            placeholder="Enter goal predicate"
                            required
                        />
                    </div>
                    <div className="flex justify-end pt-2">
                        <input type="submit" value="Execute" className={submitClassName} />
                    </div>
                </form>
            </section>
        </div>
    );
};

export default GoalForm;

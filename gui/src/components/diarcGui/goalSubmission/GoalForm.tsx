// IMPORTS //
import React, { SetStateAction } from "react";

// NPM packages
import { useForm } from "react-hook-form";
import { SendMessage } from "react-use-websocket";

// CONSTANTS //
const textBoxClassName = "block box-border w-full rounded mt-1 mb-2 text-sm "
    + "border border-slate-500 p-2 font-mono";
export { textBoxClassName };
const submitClassName = "bg-slate-900 text-white hover:bg-slate-800 "
    + "dark:bg-slate-200 dark:text-slate-900 dark:hover:bg-slate-100 "
    + "active:scale-95 inline-flex items-center justify-center "
    + "rounded-md text-sm font-medium transition-colors "
    + "focus:outline-none focus:ring-2 focus:ring-slate-400 "
    + "focus:ring-offset-2 dark:focus:ring-slate-400 "
    + "disabled:pointer-events-none dark:focus:ring-offset-slate-900 "
    + "disabled:bg-slate-400 disabled:dark:bg-slate-400 h-10 py-2 px-4 "
    + "cursor-pointer"

type Props = {
    sendMessage: SendMessage,
    path: string,
    setLastGoalSubmitted: React.Dispatch<SetStateAction<string>>,
    setSubmissionStatus: React.Dispatch<SetStateAction<string>>
};

/**
 * GoalForm. Subcomponent which renders a goal submit form.
 * @author Lucien Bao
 * @version 1.0
 */
const GoalForm: React.FC<Props> = ({
    sendMessage, path, setLastGoalSubmitted, setSubmissionStatus
}) => {
    // HOOKS & CALLBACKS //
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

    // DOM //
    return (
        <form
            onSubmit={handleSubmit(onSubmitGoal)}
            className="flex flex-col gap-1 w-full h-full border border-[#d1dbe3]
            rounded-md shadow-md p-3">

            <label>
                Agent<span className="text-red-600">*</span>
                <input type="text" defaultValue="self" {...register("agent")}
                       className={textBoxClassName} required/>
            </label>

            <label>
                Goal<span className="text-red-600">*</span>
                <input type="text" {...register("goal")}
                       className={textBoxClassName} required/>
            </label>

            <input type="submit" value="Submit"
                // From Button.tsx
                   className={submitClassName}
            />
        </form>
    );
};

export default GoalForm;
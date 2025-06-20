import ActionProgrammer from "../actionProgrammer/ActionProgrammer";
import BeliefViewer from "../BeliefViewer";
import ChatViewer from "../ChatViewer";
import GoalViewer from "../GoalViewer";
import GoalQueueViewer from "../GoalQueueViewer";
import MapViewer from "../MapViewer";
import GoalSubmission from "../goalSubmission/GoalSubmission";
import VisionManager from "../vision/VisionManager";
import TradeServiceViewer from "../tradeService/TradeServiceViewer";
import AvaBehaviorTab from "../AvaBehaviorTab"

const handlerRoots: string[] =
    //["actionProgrammer", "belief", "chat", "goalViewer", "goalManager", "map", "vision", "tradeService"];
    ["actionProgrammer", "chat", "goalViewer", "goalQueueViewer", "vision", "tradeService", "avaBehaviors"];

const handlers = {
    "actionProgrammer": {
        tabName: "Action Programmer",
        component: ActionProgrammer
    },
    "belief": {
        tabName: "Belief Viewer",
        component: BeliefViewer
    },
    "chat": {
        tabName: "Chat Viewer",
        component: ChatViewer
    },
    "goalViewer": {
        tabName: "Goal Viewer",
//         component: GoalViewer
        component: GoalQueueViewer
    },
    "goalManager": {
        tabName: "Goal Submission",
        component: GoalSubmission
    },
    "map": {
        tabName: "Map Viewer",
        component: MapViewer
    },
    "tradeService": {
        tabName: "TRADE Service Viewer",
        component: TradeServiceViewer
    },
    "vision": {
        tabName: "Vision Manager",
        component: VisionManager
    },
    "avaBehaviors": {
        tabName: "Task Submission",
        component: AvaBehaviorTab
    },
};

export { handlerRoots, handlers };

const columns = [
    "Goal",
    "Status",
    "Start time",
//     "Priority",
//     "ID",
];

const pastColumns = [
    "Goal",
    "Status",
    "Start time",
    "End time",
];

const scheduledColumns = [
    "Goal",
    "Scheduled Time",
    "Repeat Interval",
    "Repeat Until"
]

type Option = {
    value: string,
    label: string
};

const sortCriteria: Option[] = [
    { value: "name", label: "Goal" },
    { value: "actor", label: "Actor" },
    { value: "status", label: "Status" },
    { value: "start", label: "Start time" },
    { value: "end", label: "End time" },
//     { value: "priority", label: "Priority" },
//     { value: "id", label: "ID" },
]

const statuses: string[] = [
    "unknown",
    "pending",
    "active",
    "suspended",
    "canceled",
    "succeeded",
    "failed"
]

export {columns, pastColumns, scheduledColumns, sortCriteria, statuses};
export type { Option };

const LOCAL_STORAGE_KEY: string = "goalSubmissionPresets";
export {LOCAL_STORAGE_KEY};

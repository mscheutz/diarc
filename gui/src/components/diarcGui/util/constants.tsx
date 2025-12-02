import ActionProgrammer from "../actionProgrammer/ActionProgrammer";
import ChatViewer from "../ChatViewer";
import GoalQueueViewer from "../GoalQueueViewer";
import VisionManager from "../vision/VisionManager";
import TradeServiceViewer from "../tradeService/TradeServiceViewer";
import AvaBehaviorTab from "../AvaBehaviorTab";
import MediaManager from "../MediaManager";

const handlerRoots: string[] = [
    "actionProgrammer",
    "chat",
    "goalViewer",
    "goalQueueViewer",
    "vision",
    "tradeService",
    "avaBehaviors",
    "media"
];

const handlers = {
    "actionProgrammer": {
        tabName: "Action Programmer",
        component: ActionProgrammer
    },
    "chat": {
        tabName: "Chat Viewer",
        component: ChatViewer
    },
    "goalViewer": {
        tabName: "Goal Viewer",
        component: GoalQueueViewer
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
    "media": {
        tabName: "Media Management",
        component: MediaManager
    },
};

export { handlerRoots, handlers };

const columns = [
    "Goal",
    "Status",
    "Start time",
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

const mediaColumns = [
    "Filename",
    "File type"
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

export {columns, pastColumns, scheduledColumns, sortCriteria, statuses, mediaColumns};
export type { Option };

const LOCAL_STORAGE_KEY: string = "goalSubmissionPresets";
export {LOCAL_STORAGE_KEY};

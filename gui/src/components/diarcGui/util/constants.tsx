import ActionProgrammer from "../actionProgrammer/ActionProgrammer";
import BeliefViewer from "../BeliefViewer";
import ChatViewer from "../ChatViewer";
import GoalViewer from "../GoalViewer";
import MapViewer from "../MapViewer";
import GoalManager from "../goalManager/GoalManager";
import VisionManager from "../vision/VisionManager";
import TradeServiceViewer from "../tradeService/TradeServiceViewer";

const handlerRoots: string[] =
    ["actionProgrammer", "belief", "chat", "goalViewer", "goalManager", "map",
        "vision", "tradeService"];

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
        component: GoalViewer
    },
    "goalManager": {
        tabName: "Goal Manager",
        component: GoalManager
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
    }
};

export { handlerRoots, handlers };

const columns = [
    "Goal",
    "Actor",
    "Status",
    "Start time",
    "End time",
    "Priority",
    "ID"
];

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
    { value: "priority", label: "Priority" },
    { value: "id", label: "ID" },
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

export { columns, Option, sortCriteria, statuses }

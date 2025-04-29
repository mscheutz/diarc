import { createContext } from "react";

/**
 * ActionFormContext. Defines a context that contains a list of argument
 * strings for use in generating a form.
 * @author Lucien Bao
 * @version 1.0
 */
const ActionFormContext = createContext<string[]>([]);

export default ActionFormContext;
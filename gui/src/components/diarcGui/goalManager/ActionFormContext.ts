/**
 * @author Lucien Bao
 * @version 0.9
 * @date 3 June 2024
 * ActionFormContext. Defines a context that contains a list of argument
 * strings for use in generating a form.
 */

import { createContext } from "react";

const ActionFormContext = createContext<string[]>([]);
export default ActionFormContext;
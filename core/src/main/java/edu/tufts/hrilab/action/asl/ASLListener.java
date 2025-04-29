// Generated from /home/evan/code/diarc/diarc/core/src/main/java/edu/tufts/hrilab/action/asl/ASL.g4 by ANTLR 4.13.2
package edu.tufts.hrilab.action.asl;
import org.antlr.v4.runtime.tree.ParseTreeListener;

/**
 * This interface defines a complete listener for a parse tree produced by
 * {@link ASLParser}.
 */
public interface ASLListener extends ParseTreeListener {
	/**
	 * Enter a parse tree produced by {@link ASLParser#actionScript}.
	 * @param ctx the parse tree
	 */
	void enterActionScript(ASLParser.ActionScriptContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#actionScript}.
	 * @param ctx the parse tree
	 */
	void exitActionScript(ASLParser.ActionScriptContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#imports}.
	 * @param ctx the parse tree
	 */
	void enterImports(ASLParser.ImportsContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#imports}.
	 * @param ctx the parse tree
	 */
	void exitImports(ASLParser.ImportsContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#alias}.
	 * @param ctx the parse tree
	 */
	void enterAlias(ASLParser.AliasContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#alias}.
	 * @param ctx the parse tree
	 */
	void exitAlias(ASLParser.AliasContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#definitions}.
	 * @param ctx the parse tree
	 */
	void enterDefinitions(ASLParser.DefinitionsContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#definitions}.
	 * @param ctx the parse tree
	 */
	void exitDefinitions(ASLParser.DefinitionsContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#reserved}.
	 * @param ctx the parse tree
	 */
	void enterReserved(ASLParser.ReservedContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#reserved}.
	 * @param ctx the parse tree
	 */
	void exitReserved(ASLParser.ReservedContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#name}.
	 * @param ctx the parse tree
	 */
	void enterName(ASLParser.NameContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#name}.
	 * @param ctx the parse tree
	 */
	void exitName(ASLParser.NameContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#definition}.
	 * @param ctx the parse tree
	 */
	void enterDefinition(ASLParser.DefinitionContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#definition}.
	 * @param ctx the parse tree
	 */
	void exitDefinition(ASLParser.DefinitionContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#outputParameter}.
	 * @param ctx the parse tree
	 */
	void enterOutputParameter(ASLParser.OutputParameterContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#outputParameter}.
	 * @param ctx the parse tree
	 */
	void exitOutputParameter(ASLParser.OutputParameterContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#inputParameter}.
	 * @param ctx the parse tree
	 */
	void enterInputParameter(ASLParser.InputParameterContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#inputParameter}.
	 * @param ctx the parse tree
	 */
	void exitInputParameter(ASLParser.InputParameterContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#description}.
	 * @param ctx the parse tree
	 */
	void enterDescription(ASLParser.DescriptionContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#description}.
	 * @param ctx the parse tree
	 */
	void exitDescription(ASLParser.DescriptionContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#parameterList}.
	 * @param ctx the parse tree
	 */
	void enterParameterList(ASLParser.ParameterListContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#parameterList}.
	 * @param ctx the parse tree
	 */
	void exitParameterList(ASLParser.ParameterListContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#parameter}.
	 * @param ctx the parse tree
	 */
	void enterParameter(ASLParser.ParameterContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#parameter}.
	 * @param ctx the parse tree
	 */
	void exitParameter(ASLParser.ParameterContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#parameterTail}.
	 * @param ctx the parse tree
	 */
	void enterParameterTail(ASLParser.ParameterTailContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#parameterTail}.
	 * @param ctx the parse tree
	 */
	void exitParameterTail(ASLParser.ParameterTailContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#javaTypeList}.
	 * @param ctx the parse tree
	 */
	void enterJavaTypeList(ASLParser.JavaTypeListContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#javaTypeList}.
	 * @param ctx the parse tree
	 */
	void exitJavaTypeList(ASLParser.JavaTypeListContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#javaType}.
	 * @param ctx the parse tree
	 */
	void enterJavaType(ASLParser.JavaTypeContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#javaType}.
	 * @param ctx the parse tree
	 */
	void exitJavaType(ASLParser.JavaTypeContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#typeTail}.
	 * @param ctx the parse tree
	 */
	void enterTypeTail(ASLParser.TypeTailContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#typeTail}.
	 * @param ctx the parse tree
	 */
	void exitTypeTail(ASLParser.TypeTailContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#javaTypeListTail}.
	 * @param ctx the parse tree
	 */
	void enterJavaTypeListTail(ASLParser.JavaTypeListTailContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#javaTypeListTail}.
	 * @param ctx the parse tree
	 */
	void exitJavaTypeListTail(ASLParser.JavaTypeListTailContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#generic}.
	 * @param ctx the parse tree
	 */
	void enterGeneric(ASLParser.GenericContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#generic}.
	 * @param ctx the parse tree
	 */
	void exitGeneric(ASLParser.GenericContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#assignment}.
	 * @param ctx the parse tree
	 */
	void enterAssignment(ASLParser.AssignmentContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#assignment}.
	 * @param ctx the parse tree
	 */
	void exitAssignment(ASLParser.AssignmentContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#block}.
	 * @param ctx the parse tree
	 */
	void enterBlock(ASLParser.BlockContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#block}.
	 * @param ctx the parse tree
	 */
	void exitBlock(ASLParser.BlockContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#content}.
	 * @param ctx the parse tree
	 */
	void enterContent(ASLParser.ContentContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#content}.
	 * @param ctx the parse tree
	 */
	void exitContent(ASLParser.ContentContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#localvar}.
	 * @param ctx the parse tree
	 */
	void enterLocalvar(ASLParser.LocalvarContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#localvar}.
	 * @param ctx the parse tree
	 */
	void exitLocalvar(ASLParser.LocalvarContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#setting}.
	 * @param ctx the parse tree
	 */
	void enterSetting(ASLParser.SettingContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#setting}.
	 * @param ctx the parse tree
	 */
	void exitSetting(ASLParser.SettingContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#conditions}.
	 * @param ctx the parse tree
	 */
	void enterConditions(ASLParser.ConditionsContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#conditions}.
	 * @param ctx the parse tree
	 */
	void exitConditions(ASLParser.ConditionsContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#orCondition}.
	 * @param ctx the parse tree
	 */
	void enterOrCondition(ASLParser.OrConditionContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#orCondition}.
	 * @param ctx the parse tree
	 */
	void exitOrCondition(ASLParser.OrConditionContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#preCondition}.
	 * @param ctx the parse tree
	 */
	void enterPreCondition(ASLParser.PreConditionContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#preCondition}.
	 * @param ctx the parse tree
	 */
	void exitPreCondition(ASLParser.PreConditionContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#overallCondition}.
	 * @param ctx the parse tree
	 */
	void enterOverallCondition(ASLParser.OverallConditionContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#overallCondition}.
	 * @param ctx the parse tree
	 */
	void exitOverallCondition(ASLParser.OverallConditionContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#obligationCondition}.
	 * @param ctx the parse tree
	 */
	void enterObligationCondition(ASLParser.ObligationConditionContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#obligationCondition}.
	 * @param ctx the parse tree
	 */
	void exitObligationCondition(ASLParser.ObligationConditionContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#predicate}.
	 * @param ctx the parse tree
	 */
	void enterPredicate(ASLParser.PredicateContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#predicate}.
	 * @param ctx the parse tree
	 */
	void exitPredicate(ASLParser.PredicateContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#predicateList}.
	 * @param ctx the parse tree
	 */
	void enterPredicateList(ASLParser.PredicateListContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#predicateList}.
	 * @param ctx the parse tree
	 */
	void exitPredicateList(ASLParser.PredicateListContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#inputList}.
	 * @param ctx the parse tree
	 */
	void enterInputList(ASLParser.InputListContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#inputList}.
	 * @param ctx the parse tree
	 */
	void exitInputList(ASLParser.InputListContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#input}.
	 * @param ctx the parse tree
	 */
	void enterInput(ASLParser.InputContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#input}.
	 * @param ctx the parse tree
	 */
	void exitInput(ASLParser.InputContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#inputTail}.
	 * @param ctx the parse tree
	 */
	void enterInputTail(ASLParser.InputTailContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#inputTail}.
	 * @param ctx the parse tree
	 */
	void exitInputTail(ASLParser.InputTailContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#outputList}.
	 * @param ctx the parse tree
	 */
	void enterOutputList(ASLParser.OutputListContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#outputList}.
	 * @param ctx the parse tree
	 */
	void exitOutputList(ASLParser.OutputListContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#output}.
	 * @param ctx the parse tree
	 */
	void enterOutput(ASLParser.OutputContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#output}.
	 * @param ctx the parse tree
	 */
	void exitOutput(ASLParser.OutputContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#outputTail}.
	 * @param ctx the parse tree
	 */
	void enterOutputTail(ASLParser.OutputTailContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#outputTail}.
	 * @param ctx the parse tree
	 */
	void exitOutputTail(ASLParser.OutputTailContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#effects}.
	 * @param ctx the parse tree
	 */
	void enterEffects(ASLParser.EffectsContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#effects}.
	 * @param ctx the parse tree
	 */
	void exitEffects(ASLParser.EffectsContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#alwaysEffect}.
	 * @param ctx the parse tree
	 */
	void enterAlwaysEffect(ASLParser.AlwaysEffectContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#alwaysEffect}.
	 * @param ctx the parse tree
	 */
	void exitAlwaysEffect(ASLParser.AlwaysEffectContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#successEffect}.
	 * @param ctx the parse tree
	 */
	void enterSuccessEffect(ASLParser.SuccessEffectContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#successEffect}.
	 * @param ctx the parse tree
	 */
	void exitSuccessEffect(ASLParser.SuccessEffectContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#failureEffect}.
	 * @param ctx the parse tree
	 */
	void enterFailureEffect(ASLParser.FailureEffectContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#failureEffect}.
	 * @param ctx the parse tree
	 */
	void exitFailureEffect(ASLParser.FailureEffectContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#nonperfEffect}.
	 * @param ctx the parse tree
	 */
	void enterNonperfEffect(ASLParser.NonperfEffectContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#nonperfEffect}.
	 * @param ctx the parse tree
	 */
	void exitNonperfEffect(ASLParser.NonperfEffectContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#observes}.
	 * @param ctx the parse tree
	 */
	void enterObserves(ASLParser.ObservesContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#observes}.
	 * @param ctx the parse tree
	 */
	void exitObserves(ASLParser.ObservesContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#interrupts}.
	 * @param ctx the parse tree
	 */
	void enterInterrupts(ASLParser.InterruptsContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#interrupts}.
	 * @param ctx the parse tree
	 */
	void exitInterrupts(ASLParser.InterruptsContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#onCancel}.
	 * @param ctx the parse tree
	 */
	void enterOnCancel(ASLParser.OnCancelContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#onCancel}.
	 * @param ctx the parse tree
	 */
	void exitOnCancel(ASLParser.OnCancelContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#onSuspend}.
	 * @param ctx the parse tree
	 */
	void enterOnSuspend(ASLParser.OnSuspendContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#onSuspend}.
	 * @param ctx the parse tree
	 */
	void exitOnSuspend(ASLParser.OnSuspendContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#onResume}.
	 * @param ctx the parse tree
	 */
	void enterOnResume(ASLParser.OnResumeContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#onResume}.
	 * @param ctx the parse tree
	 */
	void exitOnResume(ASLParser.OnResumeContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#resumeResetSpec}.
	 * @param ctx the parse tree
	 */
	void enterResumeResetSpec(ASLParser.ResumeResetSpecContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#resumeResetSpec}.
	 * @param ctx the parse tree
	 */
	void exitResumeResetSpec(ASLParser.ResumeResetSpecContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#interruptSpec}.
	 * @param ctx the parse tree
	 */
	void enterInterruptSpec(ASLParser.InterruptSpecContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#interruptSpec}.
	 * @param ctx the parse tree
	 */
	void exitInterruptSpec(ASLParser.InterruptSpecContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#interruptSpecType}.
	 * @param ctx the parse tree
	 */
	void enterInterruptSpecType(ASLParser.InterruptSpecTypeContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#interruptSpecType}.
	 * @param ctx the parse tree
	 */
	void exitInterruptSpecType(ASLParser.InterruptSpecTypeContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#recovery}.
	 * @param ctx the parse tree
	 */
	void enterRecovery(ASLParser.RecoveryContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#recovery}.
	 * @param ctx the parse tree
	 */
	void exitRecovery(ASLParser.RecoveryContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#recoveryGoals}.
	 * @param ctx the parse tree
	 */
	void enterRecoveryGoals(ASLParser.RecoveryGoalsContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#recoveryGoals}.
	 * @param ctx the parse tree
	 */
	void exitRecoveryGoals(ASLParser.RecoveryGoalsContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#recoveryExcludedGoals}.
	 * @param ctx the parse tree
	 */
	void enterRecoveryExcludedGoals(ASLParser.RecoveryExcludedGoalsContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#recoveryExcludedGoals}.
	 * @param ctx the parse tree
	 */
	void exitRecoveryExcludedGoals(ASLParser.RecoveryExcludedGoalsContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#recoveryFailedActions}.
	 * @param ctx the parse tree
	 */
	void enterRecoveryFailedActions(ASLParser.RecoveryFailedActionsContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#recoveryFailedActions}.
	 * @param ctx the parse tree
	 */
	void exitRecoveryFailedActions(ASLParser.RecoveryFailedActionsContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#recoveryExcludedFailedActions}.
	 * @param ctx the parse tree
	 */
	void enterRecoveryExcludedFailedActions(ASLParser.RecoveryExcludedFailedActionsContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#recoveryExcludedFailedActions}.
	 * @param ctx the parse tree
	 */
	void exitRecoveryExcludedFailedActions(ASLParser.RecoveryExcludedFailedActionsContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#recoveryFailureReasons}.
	 * @param ctx the parse tree
	 */
	void enterRecoveryFailureReasons(ASLParser.RecoveryFailureReasonsContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#recoveryFailureReasons}.
	 * @param ctx the parse tree
	 */
	void exitRecoveryFailureReasons(ASLParser.RecoveryFailureReasonsContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#recoveryExcludedFailureReasons}.
	 * @param ctx the parse tree
	 */
	void enterRecoveryExcludedFailureReasons(ASLParser.RecoveryExcludedFailureReasonsContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#recoveryExcludedFailureReasons}.
	 * @param ctx the parse tree
	 */
	void exitRecoveryExcludedFailureReasons(ASLParser.RecoveryExcludedFailureReasonsContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#recoveryActionStatuses}.
	 * @param ctx the parse tree
	 */
	void enterRecoveryActionStatuses(ASLParser.RecoveryActionStatusesContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#recoveryActionStatuses}.
	 * @param ctx the parse tree
	 */
	void exitRecoveryActionStatuses(ASLParser.RecoveryActionStatusesContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#locks}.
	 * @param ctx the parse tree
	 */
	void enterLocks(ASLParser.LocksContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#locks}.
	 * @param ctx the parse tree
	 */
	void exitLocks(ASLParser.LocksContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#spec}.
	 * @param ctx the parse tree
	 */
	void enterSpec(ASLParser.SpecContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#spec}.
	 * @param ctx the parse tree
	 */
	void exitSpec(ASLParser.SpecContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#specReturn}.
	 * @param ctx the parse tree
	 */
	void enterSpecReturn(ASLParser.SpecReturnContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#specReturn}.
	 * @param ctx the parse tree
	 */
	void exitSpecReturn(ASLParser.SpecReturnContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#singleArgReturn}.
	 * @param ctx the parse tree
	 */
	void enterSingleArgReturn(ASLParser.SingleArgReturnContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#singleArgReturn}.
	 * @param ctx the parse tree
	 */
	void exitSingleArgReturn(ASLParser.SingleArgReturnContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#specFunction}.
	 * @param ctx the parse tree
	 */
	void enterSpecFunction(ASLParser.SpecFunctionContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#specFunction}.
	 * @param ctx the parse tree
	 */
	void exitSpecFunction(ASLParser.SpecFunctionContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#specType}.
	 * @param ctx the parse tree
	 */
	void enterSpecType(ASLParser.SpecTypeContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#specType}.
	 * @param ctx the parse tree
	 */
	void exitSpecType(ASLParser.SpecTypeContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#function}.
	 * @param ctx the parse tree
	 */
	void enterFunction(ASLParser.FunctionContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#function}.
	 * @param ctx the parse tree
	 */
	void exitFunction(ASLParser.FunctionContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#operation}.
	 * @param ctx the parse tree
	 */
	void enterOperation(ASLParser.OperationContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#operation}.
	 * @param ctx the parse tree
	 */
	void exitOperation(ASLParser.OperationContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#control}.
	 * @param ctx the parse tree
	 */
	void enterControl(ASLParser.ControlContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#control}.
	 * @param ctx the parse tree
	 */
	void exitControl(ASLParser.ControlContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#ifStatement}.
	 * @param ctx the parse tree
	 */
	void enterIfStatement(ASLParser.IfStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#ifStatement}.
	 * @param ctx the parse tree
	 */
	void exitIfStatement(ASLParser.IfStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#booleanExpression}.
	 * @param ctx the parse tree
	 */
	void enterBooleanExpression(ASLParser.BooleanExpressionContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#booleanExpression}.
	 * @param ctx the parse tree
	 */
	void exitBooleanExpression(ASLParser.BooleanExpressionContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#orExpression}.
	 * @param ctx the parse tree
	 */
	void enterOrExpression(ASLParser.OrExpressionContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#orExpression}.
	 * @param ctx the parse tree
	 */
	void exitOrExpression(ASLParser.OrExpressionContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#andExpression}.
	 * @param ctx the parse tree
	 */
	void enterAndExpression(ASLParser.AndExpressionContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#andExpression}.
	 * @param ctx the parse tree
	 */
	void exitAndExpression(ASLParser.AndExpressionContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#primaryExpression}.
	 * @param ctx the parse tree
	 */
	void enterPrimaryExpression(ASLParser.PrimaryExpressionContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#primaryExpression}.
	 * @param ctx the parse tree
	 */
	void exitPrimaryExpression(ASLParser.PrimaryExpressionContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#controlContent}.
	 * @param ctx the parse tree
	 */
	void enterControlContent(ASLParser.ControlContentContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#controlContent}.
	 * @param ctx the parse tree
	 */
	void exitControlContent(ASLParser.ControlContentContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#elifStatement}.
	 * @param ctx the parse tree
	 */
	void enterElifStatement(ASLParser.ElifStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#elifStatement}.
	 * @param ctx the parse tree
	 */
	void exitElifStatement(ASLParser.ElifStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#elseStatement}.
	 * @param ctx the parse tree
	 */
	void enterElseStatement(ASLParser.ElseStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#elseStatement}.
	 * @param ctx the parse tree
	 */
	void exitElseStatement(ASLParser.ElseStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#whileStatement}.
	 * @param ctx the parse tree
	 */
	void enterWhileStatement(ASLParser.WhileStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#whileStatement}.
	 * @param ctx the parse tree
	 */
	void exitWhileStatement(ASLParser.WhileStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#forStatement}.
	 * @param ctx the parse tree
	 */
	void enterForStatement(ASLParser.ForStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#forStatement}.
	 * @param ctx the parse tree
	 */
	void exitForStatement(ASLParser.ForStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#forInitial}.
	 * @param ctx the parse tree
	 */
	void enterForInitial(ASLParser.ForInitialContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#forInitial}.
	 * @param ctx the parse tree
	 */
	void exitForInitial(ASLParser.ForInitialContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#forCondition}.
	 * @param ctx the parse tree
	 */
	void enterForCondition(ASLParser.ForConditionContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#forCondition}.
	 * @param ctx the parse tree
	 */
	void exitForCondition(ASLParser.ForConditionContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#forUpdate}.
	 * @param ctx the parse tree
	 */
	void enterForUpdate(ASLParser.ForUpdateContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#forUpdate}.
	 * @param ctx the parse tree
	 */
	void exitForUpdate(ASLParser.ForUpdateContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#forEachStatement}.
	 * @param ctx the parse tree
	 */
	void enterForEachStatement(ASLParser.ForEachStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#forEachStatement}.
	 * @param ctx the parse tree
	 */
	void exitForEachStatement(ASLParser.ForEachStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#exitStatement}.
	 * @param ctx the parse tree
	 */
	void enterExitStatement(ASLParser.ExitStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#exitStatement}.
	 * @param ctx the parse tree
	 */
	void exitExitStatement(ASLParser.ExitStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#tryStatement}.
	 * @param ctx the parse tree
	 */
	void enterTryStatement(ASLParser.TryStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#tryStatement}.
	 * @param ctx the parse tree
	 */
	void exitTryStatement(ASLParser.TryStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#catchStatement}.
	 * @param ctx the parse tree
	 */
	void enterCatchStatement(ASLParser.CatchStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#catchStatement}.
	 * @param ctx the parse tree
	 */
	void exitCatchStatement(ASLParser.CatchStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#catchParameter}.
	 * @param ctx the parse tree
	 */
	void enterCatchParameter(ASLParser.CatchParameterContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#catchParameter}.
	 * @param ctx the parse tree
	 */
	void exitCatchParameter(ASLParser.CatchParameterContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#finallyStatement}.
	 * @param ctx the parse tree
	 */
	void enterFinallyStatement(ASLParser.FinallyStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#finallyStatement}.
	 * @param ctx the parse tree
	 */
	void exitFinallyStatement(ASLParser.FinallyStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#returnStatement}.
	 * @param ctx the parse tree
	 */
	void enterReturnStatement(ASLParser.ReturnStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#returnStatement}.
	 * @param ctx the parse tree
	 */
	void exitReturnStatement(ASLParser.ReturnStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#asyncStatement}.
	 * @param ctx the parse tree
	 */
	void enterAsyncStatement(ASLParser.AsyncStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#asyncStatement}.
	 * @param ctx the parse tree
	 */
	void exitAsyncStatement(ASLParser.AsyncStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link ASLParser#joinStatement}.
	 * @param ctx the parse tree
	 */
	void enterJoinStatement(ASLParser.JoinStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link ASLParser#joinStatement}.
	 * @param ctx the parse tree
	 */
	void exitJoinStatement(ASLParser.JoinStatementContext ctx);
}
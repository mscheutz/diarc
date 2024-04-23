/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

// Generated from /home/evan/code/diarc/src/main/java/edu/tufts/hrilab/action/asl/ASL.g4 by ANTLR 4.13.1
package edu.tufts.hrilab.action.asl;
import org.antlr.v4.runtime.tree.ParseTreeVisitor;

/**
 * This interface defines a complete generic visitor for a parse tree produced
 * by {@link ASLParser}.
 *
 * @param <T> The return type of the visit operation. Use {@link Void} for
 * operations with no return type.
 */
public interface ASLVisitor<T> extends ParseTreeVisitor<T> {
	/**
	 * Visit a parse tree produced by {@link ASLParser#actionScript}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitActionScript(ASLParser.ActionScriptContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#imports}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitImports(ASLParser.ImportsContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#alias}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitAlias(ASLParser.AliasContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#definitions}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitDefinitions(ASLParser.DefinitionsContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#reserved}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitReserved(ASLParser.ReservedContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#name}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitName(ASLParser.NameContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#definition}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitDefinition(ASLParser.DefinitionContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#outputParameter}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitOutputParameter(ASLParser.OutputParameterContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#inputParameter}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitInputParameter(ASLParser.InputParameterContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#description}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitDescription(ASLParser.DescriptionContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#parameterList}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitParameterList(ASLParser.ParameterListContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#parameter}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitParameter(ASLParser.ParameterContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#parameterTail}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitParameterTail(ASLParser.ParameterTailContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#javaTypeList}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitJavaTypeList(ASLParser.JavaTypeListContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#javaType}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitJavaType(ASLParser.JavaTypeContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#typeTail}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitTypeTail(ASLParser.TypeTailContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#javaTypeListTail}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitJavaTypeListTail(ASLParser.JavaTypeListTailContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#generic}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitGeneric(ASLParser.GenericContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#assignment}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitAssignment(ASLParser.AssignmentContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#block}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitBlock(ASLParser.BlockContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#content}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitContent(ASLParser.ContentContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#localvar}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitLocalvar(ASLParser.LocalvarContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#setting}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitSetting(ASLParser.SettingContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#conditions}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitConditions(ASLParser.ConditionsContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#orCondition}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitOrCondition(ASLParser.OrConditionContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#preCondition}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitPreCondition(ASLParser.PreConditionContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#overallCondition}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitOverallCondition(ASLParser.OverallConditionContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#obligationCondition}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitObligationCondition(ASLParser.ObligationConditionContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#predicate}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitPredicate(ASLParser.PredicateContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#predicateList}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitPredicateList(ASLParser.PredicateListContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#inputList}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitInputList(ASLParser.InputListContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#input}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitInput(ASLParser.InputContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#inputTail}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitInputTail(ASLParser.InputTailContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#outputList}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitOutputList(ASLParser.OutputListContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#output}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitOutput(ASLParser.OutputContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#outputTail}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitOutputTail(ASLParser.OutputTailContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#effects}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitEffects(ASLParser.EffectsContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#alwaysEffect}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitAlwaysEffect(ASLParser.AlwaysEffectContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#successEffect}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitSuccessEffect(ASLParser.SuccessEffectContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#failureEffect}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitFailureEffect(ASLParser.FailureEffectContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#nonperfEffect}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitNonperfEffect(ASLParser.NonperfEffectContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#observes}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitObserves(ASLParser.ObservesContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#interrupts}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitInterrupts(ASLParser.InterruptsContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#onCancel}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitOnCancel(ASLParser.OnCancelContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#onSuspend}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitOnSuspend(ASLParser.OnSuspendContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#onResume}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitOnResume(ASLParser.OnResumeContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#interruptSpec}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitInterruptSpec(ASLParser.InterruptSpecContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#interruptSpecType}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitInterruptSpecType(ASLParser.InterruptSpecTypeContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#recovery}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitRecovery(ASLParser.RecoveryContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#recoveryGoals}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitRecoveryGoals(ASLParser.RecoveryGoalsContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#recoveryExcludedGoals}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitRecoveryExcludedGoals(ASLParser.RecoveryExcludedGoalsContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#recoveryFailedActions}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitRecoveryFailedActions(ASLParser.RecoveryFailedActionsContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#recoveryExcludedFailedActions}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitRecoveryExcludedFailedActions(ASLParser.RecoveryExcludedFailedActionsContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#recoveryFailureReasons}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitRecoveryFailureReasons(ASLParser.RecoveryFailureReasonsContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#recoveryExcludedFailureReasons}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitRecoveryExcludedFailureReasons(ASLParser.RecoveryExcludedFailureReasonsContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#recoveryActionStatuses}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitRecoveryActionStatuses(ASLParser.RecoveryActionStatusesContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#locks}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitLocks(ASLParser.LocksContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#spec}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitSpec(ASLParser.SpecContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#specReturn}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitSpecReturn(ASLParser.SpecReturnContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#singleArgReturn}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitSingleArgReturn(ASLParser.SingleArgReturnContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#specFunction}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitSpecFunction(ASLParser.SpecFunctionContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#specType}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitSpecType(ASLParser.SpecTypeContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#function}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitFunction(ASLParser.FunctionContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#operation}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitOperation(ASLParser.OperationContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#control}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitControl(ASLParser.ControlContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#ifStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitIfStatement(ASLParser.IfStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#booleanExpression}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitBooleanExpression(ASLParser.BooleanExpressionContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#orExpression}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitOrExpression(ASLParser.OrExpressionContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#andExpression}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitAndExpression(ASLParser.AndExpressionContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#primaryExpression}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitPrimaryExpression(ASLParser.PrimaryExpressionContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#controlContent}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitControlContent(ASLParser.ControlContentContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#elifStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitElifStatement(ASLParser.ElifStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#elseStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitElseStatement(ASLParser.ElseStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#whileStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitWhileStatement(ASLParser.WhileStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#forStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitForStatement(ASLParser.ForStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#forInitial}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitForInitial(ASLParser.ForInitialContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#forCondition}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitForCondition(ASLParser.ForConditionContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#forUpdate}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitForUpdate(ASLParser.ForUpdateContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#forEachStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitForEachStatement(ASLParser.ForEachStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#exitStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitExitStatement(ASLParser.ExitStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#tryStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitTryStatement(ASLParser.TryStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#catchStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitCatchStatement(ASLParser.CatchStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#catchParameter}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitCatchParameter(ASLParser.CatchParameterContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#finallyStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitFinallyStatement(ASLParser.FinallyStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#returnStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitReturnStatement(ASLParser.ReturnStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#asyncStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitAsyncStatement(ASLParser.AsyncStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link ASLParser#joinStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitJoinStatement(ASLParser.JoinStatementContext ctx);
}
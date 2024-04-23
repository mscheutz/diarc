/*
 * @author Zhouqi
 */

grammar ASL;

// WS
WS: [ \n\t\r]+ -> skip;

// Comments
LINECOMMENT: '//' ~( '\r' | '\n' )* -> skip;
BLOCKCOMMENT: '/*' .*? '*/' -> skip;

// Reserved words
IMPORT : 'import';
AS : 'as';
ACTION : 'act';
OPERATOR : 'op';
OBSERVATION : 'obs';
GOAL : 'goal';
TSC : 'tsc';
INFER : 'infer';
VAR : 'var';
SET : 'setting';
OR : 'or';
PRE : 'pre';
POST : 'post';
OVERALL : 'overall';
OBLIGATION : 'obligation';
EFFECT : 'effects';
ALWAYS : 'always';
SUCCESS : 'success';
FAILURE : 'failure';
NONPERF : 'nonperf';
OBSERVES : 'observes';
RECOVERY : 'recovery';
LOCKS : 'locks';
CONDITION : 'conditions';
IF : 'if';
ELIF : 'elseif';
ELSE : 'else';
WHILE : 'while';
FOR : 'for';
FOREACH : 'foreach';
TRUE : 'true';
FALSE : 'false';
EXIT : 'exit';
TRY : 'try';
CATCH : 'catch';
FINALLY : 'finally';
NOT : '~';
RETURN : 'return';
ASYNC : 'async';
JOIN : 'join';
INTERRUPT : 'onInterrupt';
SUSPEND : 'suspend';
CANCEL : 'cancel';
RESUME : 'resume';

// fragments
fragment NAMECHAR : LETTER | DIGIT | HEADCHAR;
fragment HEADCHAR : '_' | '$';
fragment LETTER : 'a'..'z' | 'A'..'Z';
fragment LOWERCASELETTER : 'a'..'z';
fragment DIGIT : '0'..'9';
fragment NONZERO : '1'..'9';
fragment DECIMAL : '.' DIGIT+;
fragment INTEGER : '0' | '-'? NONZERO DIGIT*;
fragment FLOAT : INTEGER DECIMAL | '-0' DECIMAL | '-'? DECIMAL;

// operator
ARIOPT : '+' | '-' | '*' | '/' | '%' | 'round';
OROPT : '||';
ANDOPT : '&&';
INCOPT : '++' | '--';
GRWOPT : '+=' | '-=' | '*=' | '/=';
CMPOPT : '==' | '!=' | 'gt' | 'ge' | 'lt' | 'le' | 'geq' | 'leq';

// lexer rules;
NUMBER : INTEGER LETTER? | FLOAT LETTER?;
NAME : HEADCHAR NAMECHAR+ | LETTER NAMECHAR*;
GLOBALVAR : '?' LOWERCASELETTER NAMECHAR* | '?' LOWERCASELETTER NAMECHAR* ':' LOWERCASELETTER NAMECHAR*;
LOCALVAR : '!' LOWERCASELETTER NAMECHAR* | '!' LOWERCASELETTER NAMECHAR* ':' LOWERCASELETTER NAMECHAR*;
SENTENCE : '"' ('\\"'|'\\\\'|~('"' | '\\'))*? '"';

// Parser rules
actionScript : imports* definitions EOF;

imports : IMPORT javaType alias? ';';

alias : AS javaType;

definitions : definition+;

reserved : TSC | ACTION | OPERATOR | GOAL | OBSERVATION | VAR | SET | OR | PRE | POST | OVERALL | OBLIGATION | EFFECT
        | ALWAYS | SUCCESS | FAILURE | NONPERF | OBSERVES | INTERRUPT| SUSPEND | CANCEL | RESUME | LOCKS | CONDITION | RECOVERY | IF | ELIF | ELSE | WHILE
        | FOR | FOREACH | TRUE | FALSE | EXIT | TRY | CATCH | FINALLY | NOT | RETURN | ASYNC | JOIN;

// TODO: cleanup usage of name vs NAME
name : NAME | GLOBALVAR | LOCALVAR | reserved | operation;

definition : outputParameter '=' NAME description? inputParameter block;

outputParameter : parameterList;

inputParameter : parameterList;

description : '[' SENTENCE ']';

parameterList : '(' ( | parameter parameterTail*) ')';

parameter : javaType GLOBALVAR assignment?;

parameterTail : ',' parameter;

javaTypeList : '(' ( | javaType javaTypeListTail*) ')';

javaType : name typeTail* generic?;

typeTail : '.' name;

javaTypeListTail : ',' javaType;

generic : '<' javaType (','javaType)*'>';

assignment : '=' (input | specType ':' function);

block : '{' content '}';

content : (
          localvar
        | setting
        | conditions
        | effects
        | observes
        | interrupts
        | recovery
        | locks
        | spec
        | control
        )*;

localvar : (javaType LOCALVAR assignment? | javaType GLOBALVAR assignment?) ';';

// TODO: restrict to benefit, cost, minurg, maxurg, timeout instead of NAME
setting : NAME '=' NUMBER ';';

conditions : CONDITION ':' '{' (orCondition | preCondition | overallCondition | obligationCondition)* '}';

orCondition : OR ':' '{' (preCondition preCondition+ | overallCondition overallCondition+ | obligationCondition obligationCondition+) '}';

preCondition : PRE (OBSERVATION|INFER)? ':' predicate ';';

overallCondition : OVERALL (OBSERVATION|INFER)? ':' predicate ';';

obligationCondition : OBLIGATION (OBSERVATION|INFER)? ':' predicate ';';

predicate : NOT? name inputList;

predicateList : '{' predicate (',' predicate)* '}';

inputList : '(' ( | input inputTail*) ')';

input : NUMBER | name | NAME ':' NAME+ | SENTENCE | TRUE | FALSE | javaType | predicate;

inputTail : ',' input;

outputList : '(' output outputTail* ')' | output;

output : GLOBALVAR | LOCALVAR;

outputTail : ',' output;

effects : EFFECT ':' '{' (alwaysEffect | successEffect | failureEffect | nonperfEffect)* '}';

alwaysEffect : ALWAYS (OBSERVATION|INFER)? ':' predicate ';';

successEffect : SUCCESS (OBSERVATION|INFER)? ':' predicate ';';

failureEffect : FAILURE (OBSERVATION|INFER)? ':' predicate ';';

nonperfEffect : NONPERF (OBSERVATION|INFER)? ':' predicate ';';

//TODO: make this a collection of predicates
observes : OBSERVES ':' predicate ';';

interrupts : INTERRUPT ':' '{' (onCancel | onSuspend | onResume)* '}';

onCancel : CANCEL ':' interruptSpec;

onSuspend : SUSPEND ':' interruptSpec;

onResume : RESUME ':' interruptSpec;

interruptSpec : interruptSpecType ':' function ';';

interruptSpecType : TSC | ACTION | GOAL | name '.' TSC | name '.' ACTION  | name '.' GOAL;

recovery : RECOVERY ':' '{' (recoveryGoals | recoveryExcludedGoals)? (recoveryFailedActions | recoveryExcludedFailedActions)? (recoveryFailureReasons | recoveryExcludedFailureReasons)? recoveryActionStatuses? '}';

recoveryGoals : 'goals' ':' predicateList;

recoveryExcludedGoals : 'excludedGoals' ':' predicateList;

recoveryFailedActions : 'failedActions' ':' predicateList;

recoveryExcludedFailedActions : 'excludedFailedActions' ':' predicateList;

recoveryFailureReasons : 'failureReasons' ':' predicateList;

recoveryExcludedFailureReasons : 'excludedFailureReasons' ':' predicateList;

// TODO: replace NAME here with allowed ACTION_STATUS values
recoveryActionStatuses : 'actionStatuses' ':' '{' NAME (',' NAME)* '}';

locks : LOCKS ':' name ';';

spec : specFunction ';';

specReturn : outputList '=';

singleArgReturn : ('(' output ')' | output) '=';

specFunction : specReturn? specType ':' function;

specType : TSC | ACTION | GOAL | OBSERVATION | OPERATOR | name '.' TSC | name '.' ACTION  | name '.' GOAL | name '.' OBSERVATION;

function : name inputList?;

operation : OROPT | ANDOPT | ARIOPT | CMPOPT | INCOPT | GRWOPT;

control : (
          ifStatement
        | forStatement
        | forEachStatement
        | whileStatement
        | exitStatement
        | tryStatement
        | asyncStatement
        | joinStatement
        | returnStatement
        );

ifStatement : IF '(' booleanExpression ')' controlContent elifStatement* elseStatement?;

booleanExpression : orExpression;

orExpression : andExpression (OROPT andExpression)*;

andExpression : primaryExpression (ANDOPT primaryExpression)*;

primaryExpression : '(' booleanExpression ')' | TRUE | FALSE | specFunction | NOT primaryExpression;

controlContent : '{' (control|spec)* '}';

elifStatement : ELIF '(' booleanExpression ')' controlContent;

elseStatement : ELSE controlContent;

whileStatement : WHILE '(' booleanExpression ')' controlContent;

forStatement : FOR '(' forInitial ';' forCondition ';' forUpdate ')' controlContent;

forInitial : name ('=' (name|NUMBER))?;

forCondition : name CMPOPT (name|NUMBER);

forUpdate : name (INCOPT | GRWOPT (name|NUMBER));

forEachStatement : FOREACH '(' name ':' name ')' controlContent;

exitStatement : EXIT inputList ';';

tryStatement : TRY controlContent catchStatement* finallyStatement?;

catchStatement : CATCH '(' catchParameter? ')' controlContent;

// TODO: replace NAME here with allowed ACTION_STATUS values
catchParameter : (NAME | output) | NAME ',' output;

finallyStatement : FINALLY controlContent;

returnStatement : RETURN ';';

asyncStatement : singleArgReturn? ASYNC controlContent;

joinStatement : singleArgReturn? JOIN inputList ';';

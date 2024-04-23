0. Introduction
There are two type of rules, lexer and parser. In most of the cases only parser rules have to be changed,
so we talk mostly to parser rules.

1. Basic Structure
The basic structure of the file is defined in parser rule: "actionScript". In this rule, we can see
three parts: imports, definitions (of functions), and EOF. In most of the cases, developer will add
new capability to "definitions", or, in some cases, they might want to modify the basic structure
itself (like global variable).

2. Usage of existing rules
        a. NUMBER
        All kind of numbers are specified here: integer, and floating numbers.Support omit 0 in floating
        number f that -1 < f < 1 (i.e. .3, -.58). Do not support number with unnecessary leading 0s (i.e. 005, 00.3).
        Do not support unnecessary floating points (i.e. -4., 0.). Do not support negative 0.

        b. SENTENCE
        This is a list of any characters between two "s (double-quotes). Also known as String in java.

        c. NAME
        Java-like naming rule but expand leading characters from '$', '_' to '$', '_', '?', '!'. Do not
        support the usage of any reserved words exist in lexer rules.

        d. name
        This is a parser rule that contains NAME and reserved words in lexer rules.

        e. reserved
        All the reserved words is listed here. If new reserved words are added, they must also be added into this list.

        f. type
        A type is a java-like type declaration. It contains package name, class name and generics
        (i.e. java.util.ArrayList<Integer>). All type can be imported for the ease of use.

        g. parameter
        A parameter is a java-like variable declaration, which is "type name assignment".

        h. input
        A input can be name, NUMBER, TRUE, FALSE, SENTENCE, predicate or type.

        i. inputList
        This is zero or more input between "(" and ")".

        j. content
        Most important parser rule that contains everything that can exist in a function. A content must
        exist between "{" and "}" and called "block". Every "definition" (function) have and only have one "block".

        k. control
        Most important parser rule that contains every control statement (i.e. if-statement, while-loop)
        that can exist in a function. Note that if one statement is marked as control in ActionDBEntry,
        we usually define it also in control.

        l. controlContent
        This is the block followed after the conditional controls. Note that controlContent is different
        from a content: it only contains specs and other controls.

        m. booleanExpression
        Evaluate the boolean value of multiple specs, true and false with different boolean operators.
        The priority of boolean operators are () > ~ > && > ||.

3. Adding key words
Find the reserved words list, define new key words, add new key words to "reserved".

4. Add new script parameter
New number specification (i.e. benefit, timeout) have nothing to do with the g4 file. If there is a new
part that should be specified to an action script file (i.e. pre-condition, overall-effect), add
them under content. If there is new type of of effect and condition, specify under "effects" and "conditions".

5. Add new spec
For now we have act-spec, ob-spec, op-spec and state-spec. If there is a new spec type, simply add a
new reserved word (as mentioned before), and add the new spec name to "specType".

6. Add new control
Write new control under "control". If the new control have a block followed, put a "controlContent" at
the end. For example, a while loop is just simple "WHILE ( booleanExpression ) controlContent".

7. Semi-colon
Note that every single-line code (no matter it is in the setting or spec or control), should be end with a semi-colon.

8. Generate recognizer
This is only for intellij users, for people who are using other kind of ide, please checkout related
instructions online. First download antlr g4 plug-in to intellij. Then right click on g4 file and choose
"Configure ANTLR", and type in the package name and location (relative path starts from package location)
for the grammar recognizer files. For other options please checkout instructions online. Right click again
on g4 file and choose "Generate ANTLR Recognizer".

9. Test parser rule
Right click on a parser rule, choose "Test Rule XXX". The test component will be auto-updated every time
the g4 file is actively saved. No need to update recognizer for rule test (unless update upon save in
configuration is checked).

10. Update Parser
Every time a rule is modified, developer should update ActionScriptLanguageParser. For every parser rule,
there are two useful methods to override: "enterRuleName" and "exitRuleName". The method will be run every
time a rule is entered and exited respectively. If a rule's corresponding function is not overridden,
then nothing will happen.

11. Update Writer
Every time a rule is modified, developer should update ActionScriptLanguageWriter. Find the position
where the code should appear is essential. If it is a new script parameter, then edit write action
(or write conditions, write effects). If the change relates to the main function part, then edit
writeMainSpecs. This function will deal with all the new specs or control statements. Developer
should not deal with indentation if they are following the "indent inside a block" rule. There
are three functions to call:
        a. cat
        Pass in the one-line content to write, the function will do auto-indent and auto new-line.

        b. open block
        Pass in the content before a block. Say if writing a infinite while loop, we use
        openBlock("while(true)"). The function will open a block (add "{") and indent for two spaces.

        c. close block
        The function will close a block (add "}") and reduce indent for two spaces.

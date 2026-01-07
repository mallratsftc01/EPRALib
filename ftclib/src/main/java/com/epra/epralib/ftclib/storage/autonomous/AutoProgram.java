package com.epra.epralib.ftclib.storage.autonomous;

import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.regex.Pattern;

/// Represents an entire autonomous program as accessed via a directory.
///
/// Autonomous file structure:
/// <code><pre>
/// auto
/// ├ program.json
/// ├ conditionals.json
/// ├ init.json
/// ├ FIRST_MOVEMENT.json
/// ├ SECOND_MOVEMENT.json
/// ┆
/// └ LAST_MOVEMENT.json
/// </pre></code>
///
/// `program.json` structure:
/// <code><pre>
/// {
///     "init.json": { "TRUE": "FIRST_MOVEMENT.json" },
///     "FIRST_MOVEMENT.json": {
///         "CONDITIONAL_NAME": "NEXT_MOVEMENT_IF_CONDITION.json",
///         ...
///     },
///     "SECOND_MOVEMENT.json": \[ ... \],
///     ...
///     "LAST_MOVEMENT.json": \[\]
/// }
/// </pre></code>
///
/// `conditionals.json` structure:
/// <code><pre>
/// {
///     "CONDITIONAL_1_NAME": "\[OBJECT.DATA\] == 0",
///     "CONDITIONAL_2_NAME": "\[OBJECT_1.DATA\] >= \[OBJECT_2.DATA\]",
///     "CONDITIONAL_3_NAME": "\[OBJECT_1.DATA_1\] < \[OBJECT_2.DATA_1\] && \[OBJECT_1.DATA_2\] != \[OBJECT_2.DATA_2\]",
///     "CONDITIONAL_4_NAME": "\[Conditional.CONDITIONAL_1_NAME\] || \[Conditional.CONDITIONAL_2_NAME\],
///     ...
/// }
/// </pre></code>
/// Conditionals should always return a boolean. Conditionals `TRUE` and `FALSE` are initialized by default,
/// which each return their respective values.
///
/// Conditionals can use comparison (`==`, `>`, `<`, `>=`, `<=`, `!=`), basic logic
/// (`&&`, `||`), and basic arithmatic (`+`, `-`, `*`, `/`, `//`, `%`, `^`).
/// Conditionals can also use parentheticals.
///
/// Data suppliers must be set up upon construction and are 2 deep.
/// Data is accessed through `\[OBJECT.DATA\]`. `OBJECT` is the object to access data from,
/// or the first layer's key. `DATA` is the data type to be accessed from the `OBJECT`.
/// This data will always be a double. (When a data supplier is another conditional,
/// that conditional will be treated as 1.0 when it is `True` and 0.0 when it is `False`.)
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public class AutoProgram {

    private final String directoryPath;
    private final Gson gson = new Gson();
    private final HashMap<String, Supplier<Double>> dataSuppliers;
    private final HashMap<String, Supplier<Boolean>> conditionals;
    private final HashMap<String, ArrayList<AutoStep>> movementPaths;
    private String currentMovement;
    private int currentStepIndex;
    private final HashMap<String, Supplier<String>> nextMovementSuppliers;
    private int numParens, numAriths, numComps, numLogis;

    private final Pattern varPattern = Pattern.compile("\\[[^\\[\\]]*]");
    private final Pattern numericPattern = Pattern.compile("-?\\d+(\\.\\d*)?");
    private final Pattern comparisonPattern = Pattern.compile("[<>=!]*");
    private final Pattern logicPattern = Pattern.compile("[&|]*");

    /// Represents an entire autonomous program as accessed via a directory.
    ///
    /// Autonomous file structure:
    /// <code><pre>
    /// auto
    /// ├ program.json
    /// ├ conditionals.json
    /// ├ init.json
    /// ├ FIRST_MOVEMENT.json
    /// ├ SECOND_MOVEMENT.json
    /// ┆
    /// └ LAST_MOVEMENT.json
    /// </pre></code>
    /// @param directoryPath The path to the directory for this autonomous relative to the `Settings` directory
    /// @param dataSuppliers Data suppliers that provide data used by conditionals
    /// @see #parseConditionals()
    /// @see #parseProgram()
    public AutoProgram(String directoryPath, HashMap<String, Supplier<Double>> dataSuppliers) {
        this.directoryPath = directoryPath;
        this.dataSuppliers = dataSuppliers;
        numParens = numAriths = numComps = numLogis = 0;
        this.conditionals = new HashMap<>();
        parseConditionals();

        this.movementPaths = new HashMap<>();
        this.nextMovementSuppliers = new HashMap<>();
        parseProgram();
    }
    /// A builder for an [AutoProgram].
    public static class Builder {
        String directoryPath;
        HashMap<String, Supplier<Double>> dataSuppliers;

        /// A builder for an [AutoProgram].
        ///
        /// Starts with no data suppliers.
        /// @param directoryPath The path to the directory for this autonomous relative to the `Settings` directory
        /// @see #dataSupplier(String, Supplier)
        /// @see #dataSupplier(String, String, Supplier)
        /// @see #build()
        public Builder(String directoryPath) {
            this.directoryPath = directoryPath;
            this.dataSuppliers = new HashMap<>();
        }

        /// Adds a data [Supplier] with the given `key`.
        ///
        /// This data supplier can be referenced by conditionals with the format `\[key\]`.
        /// @param key The key to reference this data supplier
        /// @param supplier A data supplier
        /// @return This builder
        public Builder dataSupplier(String key, Supplier<Double> supplier) {
            dataSuppliers.put(key, supplier);
            return this;
        }
        /// Adds a data [Supplier] with the given `objectId` and `dataId`.
        ///
        /// This data supplier can be referenced by conditionals with the format `\[objectId.dataId\]`.
        /// @param objectId The object id of the key to reference this data supplier
        /// @param dataId The data id of the key to reference this data supplier
        /// @param supplier A data supplier
        /// @return This builder
        public Builder dataSupplier(String objectId, String dataId, Supplier<Double> supplier) {
            return dataSupplier(objectId + "." + dataId, supplier);
        }

        /// Builds an [AutoProgram] from this builder.
        /// @return The auto program built from this builder
        public AutoProgram build() {
            return new AutoProgram(directoryPath, dataSuppliers);
        }
    }

    /// Reads and parses all conditionals from the `conditionals.json` file.
    ///
    /// `conditionals.json` structure:
    /// <code><pre>
    /// {
    ///     "CONDITIONAL_1_NAME": "\[OBJECT.DATA\] == 0",
    ///     "CONDITIONAL_2_NAME": "\[OBJECT_1.DATA\] >= \[OBJECT_2.DATA\]",
    ///     "CONDITIONAL_3_NAME": "\[OBJECT_1.DATA_1\] < \[OBJECT_2.DATA_1\] && \[OBJECT_1.DATA_2\] != \[OBJECT_2.DATA_2\]",
    ///     "CONDITIONAL_4_NAME": "\[Conditional.CONDITIONAL_1_NAME\] || \[Conditional.CONDITIONAL_2_NAME\]",
    ///     ...
    /// }
    /// </pre></code>
    /// Conditionals should always return a boolean. Conditionals `TRUE` and `FALSE` are initialized by default,
    /// which each returns their respective values.
    ///
    /// Conditionals can use comparison (`==`, `>`, `<`, `>=`, `<=`, `!=`), basic logic
    /// (`&&`, `||`), and basic arithmatic (`+`, `-`, `*`, `/`, `//`, `%`, `^`).
    /// Conditionals can also use parentheticals.
    ///
    /// Data suppliers must be set up upon construction and are 2 deep.
    /// Data is accessed through `\[OBJECT.DATA\]`. `OBJECT` is the object to access data from,
    /// or the first layer's key. `DATA` is the data type to be accessed from the `OBJECT`.
    /// This data will always be a double. (When a data supplier is another conditional,
    /// that conditional will be treated as 1.0 when it is `True` and 0.0 when it is `False`.)
    ///
    /// Flushes stored references to data suppliers upon completion.
    /// @return If the conditionals were parsed successfully
    /// @see #parseLogic(String)
    /// @see #parseComparison(String)
    /// @see #parseArithmatic(String)
    private boolean parseConditionals() {
        // Reads all conditionals from the conditional.json file
        try (FileReader file = new FileReader(AppUtil.getInstance().getSettingsFile(directoryPath + "/conditionals.json"))) {
            LinkedHashMap<String, String> tempConditionals = gson.fromJson(file, new TypeToken<LinkedHashMap<String, String>>() {}.getType());

            // Adds default TRUE and FALSE
            this.conditionals.put("TRUE", () -> true);
            this.conditionals.put("FALSE", () -> false);

            // Parses all conditionals and adds them
            for (String id : tempConditionals.keySet()) {
                Supplier<Double> temp = parseLogic(tempConditionals.get(id));
                this.conditionals.put(id, () -> temp.get() == 1.0);
                this.dataSuppliers.put("Conditional." + id, temp);
            }
            dataSuppliers.clear();
        } catch (Exception e) {
            LogController.logError("Error parsing conditionals.json: " + e.getMessage());
            return false;
        }
        LogController.logInfo("Successfully parsed conditionals.json");
        return true;
    }

    /// Checks if a given string input matches the pattern `[OBJECT.DATA]`.
    /// @param str The string to be checked
    /// @return If the input string is a valid variable reference
    private boolean isVar(String str) {
        if (str == null) {
            return false;
        }
        return varPattern.matcher(str).matches();
    }
    /// Checks if a given string input is numeric.
    /// @param str The string to be checked
    /// @return If the input string is numeric
    private boolean isNumeric(String str) {
        if (str == null) {
            return false;
        }
        return numericPattern.matcher(str).matches();
    }
    /// Checks if a given string is composed solely of comparison symbols (=, <, >, !).
    /// @param str The string to be checked
    /// @return If the input string is a valid comparison operation
    private boolean isComparison(String str) {
        if (str == null) {
            return false;
        }
        return comparisonPattern.matcher(str).matches();
    }
    /// Checks if a given string is composed solely of logical symbols (&, |).
    /// @param str The string to be checked
    /// @return If the input string is a valid logical operation
    private boolean isLogic(String str) {
        if (str == null) {
            return false;
        }
        return logicPattern.matcher(str).matches();
    }

    /// Returns the supplier for a `[OBJECT.DATA]` path or returns a [Supplier] for a numeric input.
    ///
    /// Data suppliers must be set up upon construction and are 2 deep.
    /// Data is accessed through `[OBJECT.DATA]`. `OBJECT` is the object to access data from,
    /// or the first layer's key. `DATA` is the data type to be accessed from the `OBJECT`.
    /// This data will always be a double.
    /// @param path The path to the data or a numeric string
    /// @return A supplier for the value associated with the path
    private Supplier<Double> fetchData(String path) {
        if (isNumeric(path)) {
            return () -> Double.parseDouble(path);
        }
        return dataSuppliers.get(path.substring(1, path.length() - 1));
    }
    /// Parses the parenthetical statements within an arithmetic statement.
    ///
    /// Converts each parenthetical into a variable reference to that parenthetical.
    /// @param arithmatic The arithmetic statement to parse
    /// @return The input statement with all parentheticals replaced with variable references
    /// @see #parseArithmatic(String)
    private String parseArithmaticParenthetical(String arithmatic) {
        int count = 0;
        int start = -1;
        for (int i = 0; i < arithmatic.length(); i++) {
            if (arithmatic.charAt(i) == '(') {
                count++;
                if (start == -1) { start = i; }
            }
            if (arithmatic.charAt(i) == ')') {
                count--;
                if (count == 0) {
                    String id = String.valueOf(numParens++);
                    dataSuppliers.put("Parenthetical." + id, parseArithmatic(arithmatic.substring(start + 1, i)));
                    arithmatic = arithmatic.substring(0, start) + "[Parentheticals." + id + "]" + arithmatic.substring(i + 1);

                    i = start;
                    start = -1;
                }
            }
        }
        return arithmatic;
    }
    /// Deconstructs an arithmetic statement into an [ArrayList] of
    /// numeric, variable, and operational components.
    /// @param arithmatic The arithmetic statement to deconstruct
    /// @return The deconstructed statement
    /// @see #parseArithmatic(String)
    private ArrayList<String> arithmaticToComponents(String arithmatic) {
        ArrayList<String> components = new ArrayList<>();
        Function<String, String> handleNumeric = (arith) -> {
            for (int i = 1; i <= arith.length(); i++) {
                if (i == arith.length()) {
                    components.add(arith);
                    return "";
                }
                if (!isNumeric(arith.substring(0, i))) {
                    components.add(arith.substring(0, i - 1));
                    arith = arith.substring(i - 1);
                    break;
                }
            }
            return arith;
        };

        while (!arithmatic.isEmpty()) {
            switch (arithmatic.charAt(0)) {
                case '[':
                    int end = arithmatic.indexOf(']');
                    components.add(arithmatic.substring(0, end + 1));
                    arithmatic = arithmatic.substring(end + 1);
                    break;
                case '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.':
                    arithmatic = handleNumeric.apply(arithmatic);
                    break;
                case '-':
                    String last = components.get(components.size() - 1);
                    if (arithmatic.length() >= 2 && isNumeric(arithmatic.substring(0, 2)) && !isNumeric(last) && !isVar(last)) {
                        arithmatic = handleNumeric.apply(arithmatic.substring(1));
                    } else {
                        components.add("-");
                        arithmatic = arithmatic.substring(1);
                    }
                    break;
                case '/':
                    if (arithmatic.length() >= 2 && arithmatic.charAt(1) == '/') {
                        components.add("//");
                        arithmatic = arithmatic.substring(2);
                    } else {
                        components.add("/");
                        arithmatic = arithmatic.substring(1);
                    }
                    break;
                case '+', '*', '%', '^':
                    components.add(arithmatic.substring(0, 1));
                    arithmatic = arithmatic.substring(1);
                    break;
                default:
                    arithmatic = arithmatic.substring(1);
            }
        }
        return components;
    }
    /// Parses an [ArrayList] of arithmetic components generated by [#arithmaticToComponents(String)].
    /// Searches for all instances of a specified operation reference and applies that operation
    /// to the components directly before and after the operation reference.
    /// Replaces those components and the operation reference with a variable reference.
    /// @param arithmaticComponents A list of arithmetic components
    /// @param operationChar The string that serves as a reference to the given operation
    /// @param operation The operation to be applied wherever it is referenced
    /// @see #parseArithmatic(String)
    private void parseArithmaticOperation(ArrayList<String> arithmaticComponents,
                                                 String operationChar,
                                                 BiFunction<Double, Double, Double> operation) {
        if (arithmaticComponents.size() == 1) { return; }
        for (int i = 1; i < arithmaticComponents.size() - 1; i++) {
            if (arithmaticComponents.get(i).equals(operationChar)) {
                String id = String.valueOf(numAriths++);
                final Supplier<Double> A = fetchData(arithmaticComponents.get(i-1));
                final Supplier<Double> B = fetchData(arithmaticComponents.get(i+1));
                dataSuppliers.put("Arithmatic." + id, () ->
                        operation.apply(A.get(), B.get()));
                arithmaticComponents.remove(i-1);
                arithmaticComponents.remove(i-1);
                arithmaticComponents.remove(i-1);
                arithmaticComponents.add(i-1, "[Arithmatic." + id + "]");
                i = 0;
            }
        }
    }
    /// Parses an arithmetic statement into a [Supplier] by following these steps:
    /// 1) Parses parentheticals using [#parseArithmaticParenthetical(String)].
    /// 2) Decomposes the statement into components using [#arithmaticToComponents(String)].
    /// 3) Evaluates exponents using [#parseArithmaticOperation(ArrayList, String, BiFunction)].
    /// 4) Evaluates multiplication, integer division, division, and modulus.
    /// 5) Evaluates addition and subtraction.
    /// @param arithmatic The arithmetic statement to be parsed
    /// @return A supplier that returns the result of the parsed arithmetic statement
    private Supplier<Double> parseArithmatic(String arithmatic) {
        // 1) Parse parentheticals
        arithmatic = parseArithmaticParenthetical(arithmatic);

        // 2) Break into components
        ArrayList<String> components = arithmaticToComponents(arithmatic);

        // 3) Exponents
        parseArithmaticOperation(components, "^", Math::pow);

        // 4) Multiplication, Integer Division, Division, Mod
        parseArithmaticOperation(components, "*", (a, b) -> a * b);
        parseArithmaticOperation(components, "//", (a, b) -> Math.floor(a / b));
        parseArithmaticOperation(components, "/", (a, b) -> a / b);
        parseArithmaticOperation(components, "%", (a, b) -> a % b);

        // 5) Addition, Subtraction
        parseArithmaticOperation(components, "+", Double::sum);
        parseArithmaticOperation(components, "-", (a, b) -> a - b);

        return fetchData(components.get(0));
    }

    /// Parses the parenthetical statements within a comparison statement.
    ///
    /// Converts each parenthetical into a variable reference to that parenthetical.
    /// @param comparison The comparison statement to parse
    /// @return The input statement with all parentheticals replaced with variable references
    /// @see #parseComparison(String)
    private String parseComparisonParenthetical(String comparison) {
        int count = 0;
        int start = -1;
        for (int i = 0; i < comparison.length(); i++) {
            if (comparison.charAt(i) == '(') {
                count++;
                if (start == -1) { start = i; }
            }
            if (comparison.charAt(i) == ')') {
                count--;
                String sub = comparison.substring(start + 1, i);
                if (count == 0 && !(sub.contains("=") || sub.contains("<") || sub.contains(">"))) {
                    start = -1;
                } else if (count == 0) {
                    String id = String.valueOf(numParens++);
                    dataSuppliers.put("Comparison." + id, parseComparison(sub));
                    comparison = comparison.substring(0, start) + "[Parentheticals." + id + "]" + comparison.substring(i + 1);

                    i = start;
                    start = -1;
                }
            }
        }
        return comparison;
    }
    /// Deconstructs a comparison statement into an [ArrayList] of
    /// numeric, variable, and operational components.
    /// @param comparison The comparison statement to deconstruct
    /// @return The deconstructed statement
    /// @see #parseComparison(String)
    private ArrayList<String> comparisonToComponents(String comparison) {
        ArrayList<String> components = new ArrayList<>();
        String sub = "";
        for (int i = 0; i < comparison.length(); i++) {
            String comp = comparison.substring(i, i + 1);
            if (!isComparison(comp) && !comp.equals(" ")) {
                sub += comparison.substring(i, i + 1);
                continue;
            }
            if (comparison.charAt(i) == ' ') { continue; }
            if (!sub.isEmpty()) { components.add(sub); sub = ""; }
            if (i + 1 < comparison.length() && isComparison(comparison.substring(i, i + 2))) {
                components.add(comparison.substring(i, i + 2));
                i++;
            }
            else { components.add(comp); }
        }
        if (!sub.isEmpty()) { components.add(sub); }
        return components;
    }
    /// Parses an [ArrayList] of comparison components generated by [#comparisonToComponents(String)].
    /// Searches for all instances of a specified operation reference and applies that operation
    /// to the components directly before and after the operation reference.
    /// Replaces those components and the operation reference with a variable reference.
    /// @param comparisonComponents A list of comparison components
    /// @param operationChar The string that serves as a reference to the given operation
    /// @param operation The operation to be applied wherever it is referenced
    /// @see #parseComparison(String)
    private void parseComparisonOperation(ArrayList<String> comparisonComponents,
                                                 String operationChar,
                                                 BiFunction<Double, Double, Boolean> operation) {
        if (comparisonComponents.size() == 1 && !isVar(comparisonComponents.get(0))) {
            final Supplier<Double> A = parseArithmatic(comparisonComponents.get(0));
            String id = String.valueOf(numComps++);
            dataSuppliers.put("Comparison." + id, () -> A.get().equals(1.0) ? 1.0 : 0.0);
            comparisonComponents.remove(0);
            comparisonComponents.add("[Comparison." + id + "]");
        }
        for (int i = 1; i < comparisonComponents.size() - 1; i++) {
            if (comparisonComponents.get(i).equals(operationChar)) {
                String id = String.valueOf(numComps++);
                final Supplier<Double> A = parseArithmatic(comparisonComponents.get(i-1));
                final Supplier<Double> B = parseArithmatic(comparisonComponents.get(i+1));
                dataSuppliers.put("Comparison." + id, () ->
                        operation.apply(A.get(), B.get()) ? 1.0 : 0.0);
                comparisonComponents.remove(i-1);
                comparisonComponents.remove(i-1);
                comparisonComponents.remove(i-1);
                comparisonComponents.add(i-1, "[Comparison." + id + "]");
                i = 0;
            }
        }
    }
    /// Parses a comparison statement into a [Supplier] by following these steps:
    /// 1) Parses parentheticals using [#parseComparisonParenthetical(String)].
    /// 2) Decomposes the statement into components using [#comparisonToComponents(String)].
    /// 3) Evaluates all comparisons using [#parseComparisonOperation(ArrayList, String, BiFunction)].
    /// @param comparison The comparison statement to be parsed
    /// @return A supplier that returns the result of the parsed comparison statement
    private Supplier<Double> parseComparison(String comparison) {
        // 1) Parse parentheticals
        comparison = parseComparisonParenthetical(comparison);

        // 2) Break into components
        ArrayList<String> components = comparisonToComponents(comparison);

        // 3) Comparisons
        parseComparisonOperation(components, "==", Double::equals);
        parseComparisonOperation(components, "!=", (a, b) -> !a.equals(b));
        parseComparisonOperation(components, "<", (a, b) -> a < b);
        parseComparisonOperation(components, ">", (a, b) -> a > b);
        parseComparisonOperation(components, "<=", (a, b) -> a <= b);
        parseComparisonOperation(components, ">=", (a, b) -> a >= b);

        return fetchData(components.get(0));
    }

    /// Parses the parenthetical statements within a logical statement.
    ///
    /// Converts each parenthetical into a variable reference to that parenthetical.
    /// @param logic The logical statement to parse
    /// @return The input statement with all parentheticals replaced with variable references
    /// @see #parseLogic(String)
    private String parseLogicParenthetical(String logic) {
        int count = 0;
        int start = -1;
        for (int i = 0; i < logic.length(); i++) {
            if (logic.charAt(i) == '(') {
                count++;
                if (start == -1) { start = i; }
            }
            if (logic.charAt(i) == ')') {
                count--;
                String sub = logic.substring(start + 1, i);
                if (count == 0 && !(sub.contains("&&") || sub.contains("||"))) {
                    start = -1;
                } else if (count == 0 && (sub.contains("&&") || sub.contains("||"))) {
                    String id = String.valueOf(numParens++);
                    dataSuppliers.put("Parenthetical." + id, parseLogic(sub));
                    logic = logic.substring(0, start) + "[Parentheticals." + id + "]" + logic.substring(i + 1);

                    i = start;
                    start = -1;
                }
            }
        }

        return logic;
    }
    /// Deconstructs a logical statement into an [ArrayList] of
    /// variable and operational components.
    /// @param logic The logical statement to deconstruct
    /// @return The deconstructed statement
    /// @see #parseLogic(String)
    private ArrayList<String> logicToComponents(String logic) {
        ArrayList<String> components = new ArrayList<>();
        String sub = "";
        for (int i = 0; i < logic.length(); i++) {
            String comp = logic.substring(i, i + 1);
            if (isLogic(comp) != isLogic(sub)) {
                if (!sub.isEmpty()) { components.add(sub); sub = ""; }
            } else if (logic.charAt(i) == ' ') { continue; }
            sub += comp;
        }
        if (!sub.isEmpty()) { components.add(sub); }
        return components;
    }
    /// Parses an [ArrayList] of logical components generated by [#logicToComponents(String)].
    /// Searches for all instances of a specified operation reference and applies that operation
    /// to the components directly before and after the operation reference.
    /// Replaces those components and the operation reference with a variable reference.
    /// @param logicComponents A list of logical components
    /// @param operationChar The string that serves as a reference to the given operation
    /// @param operation The operation to be applied wherever it is referenced
    /// @see #parseLogic(String)
    private void parseLogicOperation(ArrayList<String> logicComponents,
                                            String operationChar,
                                            BiFunction<Double, Double, Boolean> operation) {
        if (logicComponents.size() == 1 && !isVar(logicComponents.get(0))) {
            final Supplier<Double> A = parseComparison(logicComponents.get(0));
            String id = String.valueOf(numLogis++);
            dataSuppliers.put("Logic." + id, () -> A.get().equals(1.0) ? 1.0 : 0.0);
            logicComponents.remove(0);
            logicComponents.add("[Logic." + id + "]");
        }
        for (int i = 1; i < logicComponents.size() - 1; i++) {
            if (logicComponents.get(i).equals(operationChar)) {
                String id = String.valueOf(numLogis++);
                final Supplier<Double> A = parseComparison(logicComponents.get(i-1));
                final Supplier<Double> B = parseComparison(logicComponents.get(i+1));
                dataSuppliers.put("Logic." + id, () ->
                        operation.apply(A.get(), B.get()) ? 1.0 : 0.0);
                logicComponents.remove(i-1);
                logicComponents.remove(i-1);
                logicComponents.remove(i-1);
                logicComponents.add(i-1, "[Logic." + id + "]");
                i = 0;
            }
        }
    }
    /// Parses a logic statement into a [Supplier] by following these steps:
    /// 1) Parses parentheticals using [#parseLogicParenthetical(String)].
    /// 2) Decomposes the statement into components using [#logicToComponents(String)].
    /// 3) Evaluates all logical operations using [#parseLogicOperation(ArrayList, String, BiFunction)].
    /// @param logic The logical statement to be parsed
    /// @return A supplier that returns the result of the parsed logical statement
    private Supplier<Double> parseLogic(String logic) {
        // 1) Parse parentheticals
        logic = parseLogicParenthetical(logic);

        // 2) Break into components
        ArrayList<String> components = logicToComponents(logic);

        // 3) Logic
        parseLogicOperation(components, "&&", (a, b) -> a.equals(1.0) && b.equals(1.0));
        parseLogicOperation(components, "||", (a, b) -> a.equals(1.0) || b.equals(1.0));

        return fetchData(components.get(0));
    }

    /// Parses the `program.json` file and all movement files to initialize the program.
    ///
    /// Sets up all movements using [#parseMovementFile(String)] and [#parseNextMovement(String, HashMap)].
    ///
    /// `program.json` structure:
    /// <code><pre>
    /// {
    ///     "init.json": { "TRUE": "FIRST_MOVEMENT.json" },
    ///     "FIRST_MOVEMENT.json": {
    ///         "CONDITIONAL_NAME": "NEXT_MOVEMENT_IF_CONDITION.json",
    ///         ...
    ///     },
    ///     "SECOND_MOVEMENT.json": \[ ... \],
    ///     ...
    ///     "LAST_MOVEMENT.json": \[\]
    /// }
    /// </pre></code>
    /// @return If the program is properly parsed an initialized
    private boolean parseProgram() {
        // Reads all items from the program.json file
        try (FileReader file = new FileReader(AppUtil.getInstance().getSettingsFile(directoryPath + "/program.json"))) {
            HashMap<String, LinkedHashMap<String, String>> tempProgram =
                    gson.fromJson(file, new TypeToken<HashMap<String, LinkedHashMap<String, String>>>() {}.getType());

            if (!tempProgram.containsKey("init.json")) {
                LogController.logError("No init.json file found in program.json.");
                return false;
            }
            currentMovement = "init.json";
            currentStepIndex = 0;
            // Parses all movement files and their next movements
            for (String filename : tempProgram.keySet()) {
                if (!parseMovementFile(filename)) {
                    return false;
                }
                HashMap<String, String> nextMovements = tempProgram.get(filename);
                parseNextMovement(filename, nextMovements);
            }
        } catch (Exception e) {
            LogController.logError("Error parsing program.json: " + e.getMessage());
            return false;
        }
        LogController.logInfo("Successfully parsed program.json");
        return true;
    }
    /// Parses a movement file given that file's filename.
    ///
    /// Separates the movement file into an [ArrayList] of [AutoStep]s.
    /// @param filename The filename of the movement file
    /// @return If the movement file was successfully parsed
    /// @see #parseProgram()
    private boolean parseMovementFile(String filename) {
       try (FileReader file = new FileReader(AppUtil.getInstance().getSettingsFile(directoryPath + "/" + filename))) {
           List<AutoStep> tempMovement =
                   gson.fromJson(file, new TypeToken<List<AutoStep>>() {}.getType());
           for (AutoStep step : tempMovement) {
               step.setEndCondition(conditionals.get(step.endConditionVar));
           }
           movementPaths.put(filename, new ArrayList<>(tempMovement));
       } catch (Exception e) {
           LogController.logError("Error parsing " + filename + ": " + e.getMessage());
           return false;
       }
       LogController.logInfo("Successfully parsed " + filename);
       return true;
    }
    /// Parses the conditionals that lead to the next movement a movement as defined in `program.json`.
    /// @param filename The file name of the movement to find the next movement(s) for
    /// @param nextMovements The mapping for the given filename as described in `program.json`
    /// @see #parseProgram()
    private void parseNextMovement(String filename, HashMap<String, String> nextMovements) {
        ArrayList<Supplier<String>> nextSuppliers = new ArrayList<>();
        for (String conditional : nextMovements.keySet()) {
            Supplier<Boolean> conditionalSupplier = conditionals.get(conditional);
            String next = nextMovements.get(conditional);
            nextSuppliers.add(() -> conditionalSupplier.get() ? next : "");
        }
        Supplier<String>[] nextSuppliersArray = nextSuppliers.toArray(new Supplier[0]);
        Supplier<String> nextMovementSupplier = () -> {
            for (Supplier<String> supplier : nextSuppliersArray) {
                String next = supplier.get();
                if (!next.isEmpty()) { return next; }
            }
            return "";
        };
        this.nextMovementSuppliers.put(filename, nextMovementSupplier);
    }

    /// Returns the current [AutoStep].
    /// @return The current auto step
    public AutoStep getCurrentStep() {
        return movementPaths.get(currentMovement).get(currentStepIndex);
    }
    /// Returns the file name of the current movement.
    /// @return The name of the current movement.
    public String getCurrentMovement() {
        return currentMovement;
    }
    /// Updates the current [AutoStep] if the current step's end condition is met.
    /// Advances to the next movement if the current step was the last step in the current movement.
    ///
    /// Will return `True` as long as there are more steps to complete, no matter if the current step was
    /// updated. If this returns `False`, that means that the program has reached its end.
    /// @return If there are any remaining steps
    public boolean updateStep() {
        // Checks if the step needs to be changed
        if (!movementPaths.get(currentMovement).get(currentStepIndex).endCondition()) { return true; }
        // Checks if the movement file needs to be changed
        if (currentStepIndex + 1 < movementPaths.get(currentMovement).size()) { currentStepIndex++; return true; }

        currentMovement = nextMovementSuppliers.get(currentMovement).get();
        currentStepIndex = 0;
        return !currentMovement.isEmpty();
    }
    /// Checks if there are steps remaining for the auto program to run through (including the current step).
    /// @return If the auto program is still running
    public boolean autoActive() {
        if (!movementPaths.get(currentMovement).get(currentStepIndex).endCondition()) { return true; }
        if (currentStepIndex + 1 < movementPaths.get(currentMovement).size()) { currentStepIndex++; return true; }
        return !currentMovement.isEmpty();
    }
}
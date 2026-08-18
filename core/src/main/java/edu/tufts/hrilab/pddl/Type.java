/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.pddl;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.StringJoiner;

/**
 * Defines a PDDL type and it's sub-types and super-types.
 */
public class Type {
    private final String name;
    private final List<Type> supertypes;
    private final List<Type> subtypes;
    private static final Logger log = LoggerFactory.getLogger(Type.class);

    public Type(String name) {
        this.name = name;
        this.supertypes = new ArrayList<>();
        this.subtypes = new ArrayList<>();
    }

    public void addSuperTypes(List<Type> superTypes) {
        this.supertypes.addAll(superTypes);
        superTypes.forEach(supertype -> supertype.addSubType(this));
    }

    private void addSubType(Type subType) {
        this.subtypes.add(subType);
    }

    // Turns the object into a PDDL formatted string
    public String generate() {
        StringJoiner subTypesStr = new StringJoiner(" ");

        if (!subtypes.isEmpty()) {
            for (Type subType : subtypes) {
                subTypesStr.add(subType.getName());
            }
            return (subTypesStr + " - " + name);
        }

        return name;

    }

    public String getName() {
        return name;
    }

    public List<Type> getSuperTypes() {
        return supertypes;
    }

    public List<Type> getSubTypes() {
        return subtypes;
    }

    //Returns true if the arg is a supertype of this type
    public boolean isSuper(Type type) {
        if (name.equals(type.getName())) { // If the types are the same, then they're not supers
            return false;
        }
        for (Type supertype : supertypes) { // Check all supertypes of this type
            if (supertype.getName().equals(type.getName())) {
                return true;
            }
            return supertype.isSuper(type); // Recurse with all supertypes of this type's supertypes
        }
        return false;
    }

    public String getCommonSuper(Type query) {
        if (name.equals(query.getName())) { // If the types are the same, then they're not supers
            return name;
        }

        String test = null;
        //Test all supers of the query
        for(Type querySuperType : query.getSuperTypes()) {
            test = getCommonSuper(querySuperType);
            if(test != null) {
                return test;
            }
        }
        //test all of this type's supers
        for(Type superType : getSuperTypes()) {
            test = superType.getCommonSuper(query);
            if(test != null) {
                return test;
            }
        }
        return null;
    }

    public String getSpecific(Type query) {
        if (name.equals(query.getName())) { // If the types are the same, then they're not supers
            return name;
        }

        Type test = query;

        while (!test.getSuperTypes().isEmpty()) {
            if (name.equals(test.getName())) {
                return name;
            }
            test = test.getSuperTypes().get(0);
        }

        if (getSuperTypes().isEmpty()) {
            return null;
        } else {
            return (getSuperTypes().get(0).getCommonSuper(query));
        }
    }
}

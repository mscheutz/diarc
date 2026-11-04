/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ArrayListInterface.hpp"
#include <stdio.h>
//NOTE: To get the signatures for the Java functions, use this on the Class:
// javap -p -s java/util/ArrayList
// and substitute the class path in place of java/util/ArrayList
ArrayListInterface::ArrayListInterface () {}

void ArrayListInterface::initialize (JNIEnv* newEnv, jobject existing_list) {
    initialize(newEnv);
    j_object = existing_list;
}
void ArrayListInterface::initialize (JNIEnv* newEnv) {

    env = newEnv;
    j_class = env->FindClass("java/util/ArrayList");
    if (j_class == NULL) {
        printf("X-TER-MIN-ATE\n");
        return;
    }

    j_constructor = env->GetMethodID(j_class, "<init>", "()V");
    if (j_constructor == NULL) {
        printf("A\n");
        return;
    }

    j_object = env->NewObject(j_class, j_constructor, NULL);

    jmethod_add = env->GetMethodID(j_class, "add", "(Ljava/lang/Object;)Z");
    if (jmethod_add == NULL)
        printf("B\n");

    jmethod_clear = env->GetMethodID(j_class, "clear", "()V");
    if (jmethod_clear == NULL)
        printf("C\n");

    jmethod_get = env->GetMethodID(j_class, "get", "(I)Ljava/lang/Object;");
    if (jmethod_get == NULL)
        printf("C\n");

    jmethod_size = env->GetMethodID(j_class, "size", "()I");
    if (jmethod_size == NULL)
        printf("D!\n");

    jmethod_ensureCapacity = env->GetMethodID(j_class, "ensureCapacity", "(I)V");
    if (jmethod_ensureCapacity == NULL)
        printf("E!\n");
}

bool ArrayListInterface::add(jobject newObject) {
    return static_cast<bool>(env->CallBooleanMethod(j_object, jmethod_add, newObject));
}

void ArrayListInterface::clear() {
    env->CallVoidMethod(j_object, jmethod_clear, NULL);
}

jobject ArrayListInterface::get(jint i) {
    return env->CallObjectMethod(j_object, jmethod_get, i);
}

int ArrayListInterface::size() {
    return env->CallIntMethod(j_object, jmethod_size, NULL);
}

void ArrayListInterface::ensureCapacity(int capacity) {
    env->CallVoidMethod(j_object, jmethod_ensureCapacity, (jint)capacity);
}

jobject ArrayListInterface::getJavaObject() {
    return j_object;
}

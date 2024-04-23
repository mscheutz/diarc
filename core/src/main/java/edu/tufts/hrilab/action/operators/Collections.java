/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */

package edu.tufts.hrilab.action.operators;

import java.util.*;

//TODO: Exception handling, adding missing methods (as needed)

public final class Collections {

  /** Collection operators */

  public static void clear(Collection collection) {
    collection.clear();
  }

  public static boolean isEmpty(Collection collection) {
    return collection.isEmpty();
  }

  public static int size(Collection collection) {
    return collection.size();
  }

  /** List operators **/

  public static <T> List<T> newArrayList(Class<T> cls) {
    return new ArrayList<>();
  }

  public static <T> boolean add(List<T> list, T element) {
    return list.add(element);
  }

  public static <T> void add(List<T> list, int index, T element) {
    list.add(index, element);
  }

  public static <T> boolean addAll(List<T> list, List<T> elements) {
    return list.addAll(elements);
  }

  public static boolean contains(List list, Object o) {
    return list.contains(o);
  }

  public static <T> T get(List<T> list, int index) {
    return list.get(index);
  }

  public static <T> T remove(List<T> list, int index) {
    return list.remove(index);
  }

  public static <T> boolean remove(List<T> list, T element) {
    return list.remove(element);
  }

  public static <T> T set(List<T> list, int index, T element) {
    return list.set(index, element);
  }

  public static <T> List<T> subList(List<T> list, int start, int end) {
    return list.subList(start, end);
  }

  /** Set operators **/

  public static <T> Set<T> newHashSet(Class<T> cls) {
    return new HashSet<>();
  }

  public static <T> boolean add(Set<T> set, T element) {
    return set.add(element);
  }

  public static boolean contains(Set set, Object o) {
    return set.contains(o);
  }

  public static <T> boolean remove(Set<T> set, T element) {
    return set.remove(element);
  }

  /** Map operators **/

  public static <K, V> Map<K, V> newHashMap(Class<K> key, Class<V> value) {
    return new HashMap<>();
  }

  public static boolean containsKey(Map map, Object key) {
    return map.containsKey(key);
  }

  public static boolean containsValue(Map map, Object value) {
    return map.containsValue(value);
  }

  public static <K, V> Set<Map.Entry<K, V>> entrySet(Map<K, V> map) {
    return map.entrySet();
  }

  public static <K, V> V get(Map<K, V> map, K key) {
    return map.get(key);
  }

  public static <K> Set<K> keySet(Map<K, ?> map) {
    return map.keySet();
  }

  public static <K, V> V put(Map<K, V> map, K key, V value) {
    return map.put(key, value);
  }

  public static <K, V> V remove(Map<K, V> map, K key) {
    return map.remove(key);
  }
}

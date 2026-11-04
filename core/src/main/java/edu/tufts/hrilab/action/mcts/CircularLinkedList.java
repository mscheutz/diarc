/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.mcts;

public class CircularLinkedList<T> {
    class Node {
        T value;
        Node nextNode;
        Node(T value) {
            this.value = value;
        }
    }

    private Node head = null;
    private Node tail = null;
    private Node current = null;

    public void add(T value) {
        Node newNode = new Node(value);

        if(head == null) {
            head = newNode;
            current = head;
        } else {
            tail.nextNode = newNode;
        }
        tail = newNode;
        tail.nextNode = head;
    }

    public T getNext() {
        T ret = current.value;
        current = current.nextNode;
        return ret;
    }

    public T getNext(T obj) {
        Node currentNode = head;
        do {
            if(currentNode.value.equals(obj)) {
                return currentNode.nextNode.value;
            }
            currentNode = currentNode.nextNode;
        } while (currentNode != head);
        return null;
    }

}

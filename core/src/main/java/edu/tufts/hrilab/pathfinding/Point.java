/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.pathfinding;

class Point {
    public int x, y;
    public int g; // steps from start
    public int h; // euclidean distance from end/goal
    public int f; // g + h
    public Point parent;

    public Point(int x, int y, Point parent) {
        this.x = x;
        this.y = y;
        this.f = 0;
        this.g = 0;
        this.h = 0;
        this.parent = parent;
    }

    public void setParent(Point p) {
        this.parent = p;
    }

    @Override
    public String toString() {
        return "[" + x + "," + y + ']';
    }

    @Override
    public boolean equals(Object o) {
        Point p = (Point) o;
        return p.x == x && p.y == y && o.getClass() == Point.class;
    }

}

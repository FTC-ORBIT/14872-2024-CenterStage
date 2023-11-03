package org.firstinspires.ftc.teamcode.OrbitUtils;

import java.util.ArrayList;

public class Feedback<T> {

    protected ArrayList<T> array;
    protected int arrayMaxSize;
    protected T current;
    protected T previous;
    private int currentIndex = 0;

    public Feedback(final T initialValue, final int maxFeedbacks) {
        array = new ArrayList<T>();
        arrayMaxSize = maxFeedbacks + 1;
        for (int i = 0; i < arrayMaxSize; i++) {
            array.add(initialValue);
        }
        current = initialValue;
        previous = initialValue;
    }

    public void update(final T value) {
        array.set(currentIndex, value);
        previous = current;
        current = value;
        currentIndex = currentIndex >= arrayMaxSize - 1 ? 0 : currentIndex + 1;
    }

    public T getFeedback(final int feedback) {
        try {
            return array.get((currentIndex - feedback - 1 + arrayMaxSize) % arrayMaxSize);
        } catch (Exception e) {
            System.out.println("getFeedback error: not in bounds of the array");
            return null;
        }
    }

    public T getLast() {
        return current;
    }

    public void reset(final T value) {
        for (int i = 0; i < arrayMaxSize; i++) {
            array.set(i, value);
        }
    }

}

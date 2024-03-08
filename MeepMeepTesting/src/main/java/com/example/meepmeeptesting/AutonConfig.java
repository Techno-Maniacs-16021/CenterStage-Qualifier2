package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;

public class AutonConfig {
    private Action start;
    private Action plusOne;
    private Action stack;
    private Action whitePixel;

    private Action moveBack;
    private Action plusthree;
    private Action secondMoveBack;
    private Action secondMoveFront;
    private Action park;

    public Action getStart() {
        return start;
    }

    public void setStart(Action start) {
        this.start = start;
    }

    public Action getPlusOne() {
        return plusOne;
    }

    public void setPlusOne(Action plusOne) {
        this.plusOne = plusOne;
    }

    public Action getStack() {
        return stack;
    }

    public void setStack(Action stack) {
        this.stack = stack;
    }

    public Action getWhitePixel() {
        return whitePixel;
    }

    public void setWhitePixel(Action whitePixel) {
        this.whitePixel = whitePixel;
    }

    public Action getMoveBack() {
        return moveBack;
    }

    public void setMoveBack(Action moveBack) {
        this.moveBack = moveBack;
    }

    public Action getPark() {
        return park;
    }

    public void setPark(Action park) {
        this.park = park;
    }

    public Action getCycle() {
        return cycle;
    }

    public void setCycle(Action cycle) {
        this.cycle = cycle;
    }

    public Action getPlusthree() {
        return plusthree;
    }

    public void setPlusthree(Action plusthree) {
        this.plusthree = plusthree;
    }

    public Action getSecondMoveBack() {
        return secondMoveBack;
    }

    public void setSecondMoveBack(Action secondMoveBack) {
        this.secondMoveBack = secondMoveBack;
    }

    public Action getSecondMoveFront() {
        return secondMoveFront;
    }

    public void setSecondMoveFront(Action secondMoveFront) {
        this.secondMoveFront = secondMoveFront;
    }

    private Action cycle;


}

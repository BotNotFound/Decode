package org.firstinspires.ftc.teamcode.module;

import androidx.annotation.NonNull;

import java.util.Locale;

public class Vector {
    public double x;
    public double y;
    public double z;

    // Constructors
    public Vector() {
        this(0, 0, 0);
    }

    public Vector(double x, double y) {
        this(x, y, 0);
    }

    public Vector(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // Copy
    public Vector copy() {
        return new Vector(x, y, z);
    }

    // Basic math
    public Vector add(Vector v) {
        return new Vector(x + v.x, y + v.y, z + v.z);
    }

    public Vector subtract(Vector v) {
        return new Vector(x - v.x, y - v.y, z - v.z);
    }

    public Vector multiply(double scalar) {
        return new Vector(x * scalar, y * scalar, z * scalar);
    }

    public Vector divide(double scalar) {
        if (scalar == 0) {
            return new Vector();
        }
        return new Vector(x / scalar, y / scalar, z / scalar);
    }

    // Magnitude
    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public double magnitude2D() {
        return Math.sqrt(x * x + y * y);
    }

    // Normalize
    public Vector normalized() {
        double mag = magnitude();
        if (mag == 0) {
            return new Vector();
        }
        return divide(mag);
    }

    // Dot product
    public double dot(Vector v) {
        return x * v.x + y * v.y + z * v.z;
    }

    // Cross product
    public Vector cross(Vector v) {
        return new Vector(
                y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x
        );
    }

    // Distance
    public double distanceTo(Vector v) {
        return subtract(v).magnitude();
    }

    // Rotate in 2D (FTC field-centric / mecanum)
    public Vector rotate2D(double radians) {
        double cos = Math.cos(radians);
        double sin = Math.sin(radians);

        return new Vector(
                x * cos - y * sin,
                x * sin + y * cos,
                z
        );
    }

    // Clamp magnitude (motor power limiting)
    public Vector clamp(double maxMagnitude) {
        double mag = magnitude();
        if (mag > maxMagnitude) {
            return normalized().multiply(maxMagnitude);
        }
        return copy();
    }

    @NonNull
    @Override
    public String toString() {
        final StringBuilder builder = new StringBuilder();
        return String.format(Locale.ENGLISH, "Vector(%.3f, %.3f, %.3f)", x, y, z);
    }
}

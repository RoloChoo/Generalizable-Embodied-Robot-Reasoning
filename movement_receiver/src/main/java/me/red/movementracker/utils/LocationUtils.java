package me.red.movementracker.utils;

import lombok.experimental.UtilityClass;

@UtilityClass
public class LocationUtils {

    public float normalizeYaw(float yaw) {
        while (yaw < 0) yaw += 360;
        while (yaw >= 360) yaw -= 360;

        return yaw;
    }

}

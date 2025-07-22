package me.red.movementracker.utils;

import lombok.experimental.UtilityClass;
import org.bukkit.ChatColor;

@UtilityClass
public class CC {

    public String translate(String input) {
        return ChatColor.translateAlternateColorCodes('&', input);
    }

}

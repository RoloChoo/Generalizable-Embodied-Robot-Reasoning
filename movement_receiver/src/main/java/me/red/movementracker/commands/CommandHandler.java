package me.red.movementracker.commands;

import me.red.movementracker.MovementTracker;
import revxrsal.commands.Lamp;
import revxrsal.commands.bukkit.BukkitLamp;
import revxrsal.commands.bukkit.actor.BukkitCommandActor;

public class CommandHandler {
    protected final MovementTracker plugin;

    protected Lamp<BukkitCommandActor> handler;
    protected Lamp.Builder<BukkitCommandActor> builder;

    public CommandHandler(MovementTracker plugin) {
        this.plugin = plugin;

        this.builder = BukkitLamp.builder(plugin);

        build();
    }

    public void build() {
        this.handler = builder.build();
        register();
    }

    public void register() {
        handler.register(
                new TrackerCommand(plugin)
        );
    }
}

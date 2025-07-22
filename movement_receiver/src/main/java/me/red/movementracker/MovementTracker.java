package me.red.movementracker;

import lombok.Getter;
import me.red.movementracker.commands.CommandHandler;
import me.red.movementracker.commands.TrackerCommand;
import me.red.movementracker.listener.PlayerListener;
import me.red.movementracker.mongo.MongoManager;
import me.red.movementracker.tracker.handler.TrackerHandler;
import org.bukkit.plugin.java.JavaPlugin;

@Getter
public final class MovementTracker extends JavaPlugin {

    private MongoManager mongoManager;
    private CommandHandler commandHandler;
    private TrackerHandler trackerHandler;

    @Override
    public void onEnable() {
        setupConfig();

        this.mongoManager = MongoManager.create(
                getConfig().getString("Mongo.Uri"),
                getConfig().getString("Mongo.Database")
        );

        this.trackerHandler = new TrackerHandler();
        this.commandHandler = new CommandHandler(this);

        getServer().getPluginManager().registerEvents(new PlayerListener(this), this);
        getLogger().info("MovementTracker è stato abilitato!");
    }

    @Override
    public void onDisable() {
        getLogger().info("MovementTracker è stato disabilitato!");
    }

    private void setupConfig() {
        saveDefaultConfig();
        getConfig().options().copyDefaults(true);
    }
}

package me.red.movementracker.commands;

import com.mongodb.client.model.Filters;
import me.red.movementracker.MovementTracker;
import me.red.movementracker.tracker.PlayerTracker;
import me.red.movementracker.utils.CC;
import org.bson.Document;
import org.bukkit.ChatColor;
import org.bukkit.entity.Player;
import revxrsal.commands.annotation.Command;
import revxrsal.commands.bukkit.actor.BukkitCommandActor;
import revxrsal.commands.bukkit.annotation.CommandPermission;

import java.util.concurrent.CompletableFuture;

public class TrackerCommand {
    private final MovementTracker plugin;

    public TrackerCommand(MovementTracker plugin) {
        this.plugin = plugin;
    }

    @Command("track help")
    @CommandPermission("movementtracker.help")
    public void onHelp(BukkitCommandActor actor) {
        actor.reply(CC.translate(
                """
                        &a&lMovementTracker - Ultra Simple
                        &7/track <player> [name] &8- &fStart tracking
                        &7/untrack <player> &8- &fStop tracking  
                        &7/track info <player> &8- &fShow current action
                        
                        &eFlat structure: Only current action gets updated!
                        &eNo arrays, no complexity - just direct field updates.
                        """
        ));
    }

    @Command("track")
    @CommandPermission("movementtracker.track")
    public void onTrack(Player player, Player target, String trackName) {
        if (!plugin.getTrackerHandler().trackPlayer(target, trackName)) {
            player.sendMessage(ChatColor.RED + "Already tracking!");
            return;
        }
        player.sendMessage(ChatColor.GREEN + "✓ Tracking " + target.getName());
    }

    @Command("track")
    @CommandPermission("movementtracker.track")
    public void onTrackDefault(Player player, Player target) {
        if (!plugin.getTrackerHandler().trackPlayer(target)) {
            player.sendMessage(ChatColor.RED + "Already tracking!");
            return;
        }
        player.sendMessage(ChatColor.GREEN + "✓ Tracking " + target.getName());
    }

    @Command("untrack")
    @CommandPermission("movementtracker.untrack")
    public void onUntrack(Player player, Player target) {
        if (!plugin.getTrackerHandler().isTracked(target)) {
            player.sendMessage(ChatColor.RED + "Not tracking!");
            return;
        }

        plugin.getTrackerHandler().removePlayer(target);
        player.sendMessage(ChatColor.GREEN + "✓ Stopped tracking " + target.getName());
    }

    @Command("track info")
    @CommandPermission("movementtracker.info")
    public void onTrackInfo(Player player, Player target) {
        if (!plugin.getTrackerHandler().isTracked(target)) {
            player.sendMessage(ChatColor.RED + "Not tracking this player!");
            return;
        }

        PlayerTracker tracker = plugin.getTrackerHandler().getTracker(target);
        if (tracker == null) {
            player.sendMessage(ChatColor.RED + "Tracker not found!");
            return;
        }

        CompletableFuture.runAsync(() -> {
            try {
                String documentId = tracker.getTrackName() + "_" + target.getUniqueId().toString();

                Document doc = plugin.getMongoManager().getMovements()
                        .find(Filters.eq("_id", documentId))
                        .first();

                if (doc == null) {
                    player.sendMessage(ChatColor.RED + "No data found!");
                    return;
                }

                // 1층 구조에서 직접 읽기
                String currentAction = doc.getString("current_action");
                Double currentYaw = doc.getDouble("current_yaw");
                Double currentPitch = doc.getDouble("current_pitch");
                Integer currentDuration = doc.getInteger("current_duration");
                Integer totalActions = doc.getInteger("total_actions");
                String status = doc.getString("status");

                player.sendMessage(CC.translate(
                        "&a=== " + target.getName() + " Current Status ===\n" +
                                "&7Action: &e" + (currentAction != null ? currentAction.toUpperCase() : "NONE") + "\n" +
                                "&7Yaw: &f" + (currentYaw != null ? currentYaw : 0) + "\n" +
                                "&7Pitch: &f" + (currentPitch != null ? currentPitch : 0) + "\n" +
                                "&7Duration: &b" + (currentDuration != null ? currentDuration : 0) + "\n" +
                                "&7Total Actions: &e" + (totalActions != null ? totalActions : 0) + "\n" +
                                "&7Status: " + ("tracking".equals(status) ? "&aActive" : "&cStopped")
                ));

            } catch (Exception e) {
                player.sendMessage(ChatColor.RED + "Error: " + e.getMessage());
                e.printStackTrace();
            }
        });
    }
}
package me.red.movementracker.listener;

import it.unimi.dsi.fastutil.objects.Object2LongOpenHashMap;
import me.red.movementracker.MovementTracker;
import me.red.movementracker.tracker.ActionType;
import me.red.movementracker.tracker.PlayerTracker;
import me.red.movementracker.tracker.TrackerAction;
import me.red.movementracker.utils.LocationUtils;
import org.bukkit.Bukkit;
import org.bukkit.Location;
import org.bukkit.entity.Player;
import org.bukkit.event.EventHandler;
import org.bukkit.event.Listener;
import org.bukkit.event.player.PlayerMoveEvent;
import org.bukkit.event.player.PlayerQuitEvent;
import org.bukkit.util.Vector;

public class PlayerListener implements Listener {

    private final MovementTracker plugin;
    private final Object2LongOpenHashMap<Player> lastTick = new Object2LongOpenHashMap<>();

    public PlayerListener(MovementTracker plugin) {
        this.plugin = plugin;

        Bukkit.getScheduler().runTaskTimer(plugin, () ->
                Bukkit.getOnlinePlayers().forEach(player -> {
                            if (!plugin.getTrackerHandler().isTracked(player)) return;

                            if (System.currentTimeMillis() - lastTick.getOrDefault(player, 0) > 50) {
                                float yaw = LocationUtils.normalizeYaw(player.getLocation().getYaw());
                                float pitch = player.getLocation().getPitch();

                                PlayerTracker tracker = plugin.getTrackerHandler().getTracker(player);
                                if (tracker != null) {
                                    tracker.add(new TrackerAction(tracker.getCurrentOrder(), System.currentTimeMillis(), ActionType.IDLE, yaw, pitch));
                                }
                            }
                        }
                ), 1L, 1L);
    }

    @EventHandler
    public void onQuit(PlayerQuitEvent event) {
        lastTick.remove(event.getPlayer());
        // 플레이어가 나갈 때 추적도 중지
        plugin.getTrackerHandler().removePlayer(event.getPlayer());
    }

    @EventHandler
    public void onPlayerMove(PlayerMoveEvent event) {
        Player player = event.getPlayer();
        if (!plugin.getTrackerHandler().isTracked(player)) return;

        Location from = event.getFrom();
        Location to = event.getTo();

        lastTick.put(event.getPlayer(), System.currentTimeMillis());

        Vector movement = new Vector(
                to.getX() - from.getX(),
                0,
                to.getZ() - from.getZ()
        );

        float yaw = LocationUtils.normalizeYaw(player.getLocation().getYaw());
        float pitch = player.getLocation().getPitch();

        double yawRadians = Math.toRadians(-yaw);

        Vector forward = new Vector(Math.sin(yawRadians), 0, Math.cos(yawRadians)).normalize();
        Vector right = new Vector(Math.sin(yawRadians + Math.PI/2), 0, Math.cos(yawRadians + Math.PI/2)).normalize();

        double forwardComponent = movement.dot(forward);
        double rightComponent = movement.dot(right);

        ActionType action = determineAction(forwardComponent, rightComponent);

        PlayerTracker tracker = plugin.getTrackerHandler().getTracker(player);
        if (tracker != null) {
            tracker.add(new TrackerAction(tracker.getCurrentOrder(), System.currentTimeMillis(), action, yaw, pitch));
        }
    }

    private ActionType determineAction(double forward, double right) {
        double threshold = 0.01;

        boolean isMovingForward = forward > threshold;
        boolean isMovingBackward = forward < -threshold;
        boolean isMovingRight = right > threshold;
        boolean isMovingLeft = right < -threshold;

        if (isMovingForward && isMovingRight) {
            return ActionType.FORWARD_LEFT;
        } else if (isMovingForward && isMovingLeft) {
            return ActionType.FORWARD_RIGHT;
        } else if (isMovingBackward && isMovingRight) {
            return ActionType.BACKWARD_LEFT;
        } else if (isMovingBackward && isMovingLeft) {
            return ActionType.BACKWARD_RIGHT;
        } else if (isMovingForward) {
            return ActionType.FORWARD;
        } else if (isMovingBackward) {
            return ActionType.BACKWARD;
        } else if (isMovingRight) {
            return ActionType.LEFT;
        } else if (isMovingLeft) {
            return ActionType.RIGHT;
        } else {
            return ActionType.IDLE;
        }
    }
}
package me.red.movementracker.tracker.handler;

import it.unimi.dsi.fastutil.objects.Object2ObjectArrayMap;
import lombok.Getter;
import me.red.movementracker.tracker.PlayerTracker;
import org.bukkit.entity.Player;

import java.util.UUID;

@Getter
public class TrackerHandler {

    // 추적 중인 플레이어들의 tracker 정보만 저장 (메모리에서 액션 데이터는 제거)
    private final Object2ObjectArrayMap<UUID, PlayerTracker> trackers = new Object2ObjectArrayMap<>();

    public boolean trackPlayer(Player player, String trackName) {
        if (trackers.containsKey(player.getUniqueId())) return false;

        trackers.put(player.getUniqueId(), new PlayerTracker(player.getUniqueId(), trackName));
        return true;
    }

    public boolean trackPlayer(Player player) {
        return trackPlayer(player, "default_" + System.currentTimeMillis());
    }

    public void removePlayer(Player player) {
        trackers.remove(player.getUniqueId());
    }

    public boolean isTracked(Player player) {
        return trackers.containsKey(player.getUniqueId());
    }

    public PlayerTracker getTracker(Player player) {
        return trackers.get(player.getUniqueId());
    }

    // getActions() 메소드는 제거 - 더 이상 메모리에 액션을 저장하지 않음
}
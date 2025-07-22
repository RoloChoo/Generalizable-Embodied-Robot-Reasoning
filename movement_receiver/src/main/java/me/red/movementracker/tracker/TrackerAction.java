package me.red.movementracker.tracker;

import org.bson.Document;
import org.bukkit.ChatColor;

public record TrackerAction(int order, long time, ActionType action, float yaw, float pitch) {

    public String toString() {
        return ChatColor.GREEN + "Direzione: " + ChatColor.YELLOW + action.name() +
                ChatColor.GREEN + " | Yaw: " + ChatColor.YELLOW + String.format("%.2f", yaw) +
                ChatColor.GREEN + " | Pitch: " + ChatColor.YELLOW + String.format("%.2f", pitch);
    }

    /**
     * 1층 구조로 평면화된 Document 생성
     */
    public Document toFlatDocument(String replayName, String playerId, long duration) {
        return new Document()
                .append("_id", replayName + "_" + playerId + "_" + order)
                .append("replay_name", replayName)
                .append("player_id", playerId)
                .append("order", order)
                .append("timestamp", time)
                .append("action", action.name().toLowerCase())
                .append("yaw", yaw)
                .append("pitch", pitch)
                .append("duration", duration)
                .append("created_at", System.currentTimeMillis());
    }

    /**
     * 기존 호환성을 위한 2층 구조 Document
     */
    public Document toDocument() {
        return new Document("action", action.name().toLowerCase())
                .append("yaw", yaw)
                .append("pitch", pitch)
                .append("order", order)
                .append("timestamp", time);
    }

    @Override
    public boolean equals(Object object) {
        if (!(object instanceof TrackerAction trackerAction)) return false;

        // 액션 타입이 다르면 다른 액션
        if (!trackerAction.action().equals(action)) return false;

        // yaw, pitch 차이가 임계값 이내면 같은 액션으로 간주
        float yawDiff = Math.abs(trackerAction.yaw() - yaw);
        float pitchDiff = Math.abs(trackerAction.pitch() - pitch);

        // 5도 이내의 변화는 같은 액션으로 간주
        return yawDiff <= 5.0f && pitchDiff <= 5.0f;
    }

    @Override
    public int hashCode() {
        // action만으로 해시코드 생성 (yaw, pitch는 제외)
        return action.hashCode();
    }
}
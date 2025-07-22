package me.red.movementracker.tracker;

import com.mongodb.client.model.Updates;
import lombok.Data;
import lombok.Getter;
import me.red.movementracker.mongo.MongoManager;
import org.bson.Document;
import org.bson.conversions.Bson;

import java.util.UUID;

@Data
public class PlayerTracker {
    private final UUID uuid;
    private final String trackName;
    private TrackerAction previousAction;

    @Getter
    private int currentOrder = 0;

    private boolean documentInitialized = false;

    public PlayerTracker(UUID uuid, String trackName) {
        this.uuid = uuid;
        this.trackName = trackName;
        initializeDocument();
    }

    /**
     * 처음에 빈 Document 생성 - 완전 1층 구조
     */
    private void initializeDocument() {
        try {
            String documentId = trackName + "_" + uuid.toString();

            // 이미 존재하는지 확인
            Document existing = MongoManager.get().getMovements()
                    .find(new Document("_id", documentId))
                    .first();

            if (existing == null) {
                Document initialDoc = new Document()
                        .append("_id", documentId)
                        .append("replay_name", trackName)
                        .append("player_id", uuid.toString())
                        .append("current_action", "idle")           // 현재 액션
                        .append("current_yaw", 0.0)                // 현재 yaw
                        .append("current_pitch", 0.0)              // 현재 pitch
                        .append("current_duration", 0)             // 현재 duration
                        .append("total_actions", 0)                // 총 액션 수
                        .append("status", "tracking");

                MongoManager.get().getMovements().insertOne(initialDoc);
            }

            documentInitialized = true;

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void add(TrackerAction action) {
        if (!documentInitialized) return;

        // 이전 액션과 다르면 새로운 액션으로 교체
        if (previousAction == null || !previousAction.equals(action)) {
            updateCurrentAction(action);
        } else {
            // 같은 액션이면 duration만 증가
            incrementCurrentDuration();
        }

        previousAction = action;
    }

    /**
     * 현재 액션 정보를 새로운 액션으로 교체 - 1층 구조에서 직접 수정
     */
    private void updateCurrentAction(TrackerAction action) {
        try {
            String documentId = trackName + "_" + uuid.toString();

            // 1층 구조에서 필드들을 직접 업데이트
            Bson update = Updates.combine(
                    Updates.set("current_action", action.action().name().toLowerCase()),
                    Updates.set("current_yaw", Math.round(action.yaw() * 10.0) / 10.0),
                    Updates.set("current_pitch", Math.round(action.pitch() * 10.0) / 10.0),
                    Updates.set("current_duration", 1),
                    Updates.inc("total_actions", 1)
            );

            MongoManager.get().getMovements().updateOne(
                    new Document("_id", documentId),
                    update
            );

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * 현재 액션의 duration만 +1 (가장 가벼운 연산)
     */
    private void incrementCurrentDuration() {
        try {
            String documentId = trackName + "_" + uuid.toString();

            // 단순히 current_duration 필드만 +1
            MongoManager.get().getMovements().updateOne(
                    new Document("_id", documentId),
                    Updates.inc("current_duration", 1)
            );

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * 추적 종료
     */
    public void finishTracking() {
        try {
            String documentId = trackName + "_" + uuid.toString();

            MongoManager.get().getMovements().updateOne(
                    new Document("_id", documentId),
                    Updates.set("status", "completed")
            );

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
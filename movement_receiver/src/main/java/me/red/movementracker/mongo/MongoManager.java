package me.red.movementracker.mongo;

import com.mongodb.client.MongoClient;
import com.mongodb.client.MongoClients;
import com.mongodb.client.MongoCollection;
import com.mongodb.client.MongoDatabase;
import lombok.Getter;
import org.bson.Document;

public class MongoManager {

    private static MongoManager instance;

    private MongoClient client;
    private MongoDatabase database;

    @Getter
    private MongoCollection<Document> movements;

    public MongoManager(String URI, String database){
        this.client = MongoClients.create(URI);
        this.database = this.client.getDatabase(database);

        this.movements = this.database.getCollection("movementracker");
    }

    public static MongoManager create(String URI, String database){
        if (instance == null){
            instance = new MongoManager(URI, database);
        }
        return instance;
    }

    public static MongoManager get(){
        return instance;
    }
}
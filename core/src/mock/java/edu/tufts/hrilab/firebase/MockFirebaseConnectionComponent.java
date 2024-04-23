/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.firebase;

import java.util.*;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

public class MockFirebaseConnectionComponent extends FirebaseConnectionComponent {

    private boolean useEmulator;

    public MockFirebaseConnectionComponent() {
        super();
        useEmulator = false;
    }

    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = super.additionalUsageInfo();
        options.add(Option.builder("firebaseGroup").hasArg().argName("name").desc("set firebase group to use").build());
        options.add(Option.builder("emulator").desc("connect to firestore emulator").build());

        return options;
    }

    @Override
    protected void parseArgs(CommandLine cmdLine) {
        super.parseArgs(cmdLine);
        if (cmdLine.hasOption("firebaseGroup")) {
            groupName = cmdLine.getOptionValue("firebaseGroup");
        }
        if (cmdLine.hasOption("emulator")) {
            useEmulator = true;
        }
    }
    @Override
    public boolean writeToDocument(String documentPath, Map<String, Object> data) {
        log.debug("[writeToDocument] writing to document at: {} with data: {}", documentPath, data);
        return true;
    }

    @Override
    public boolean updateDocument(String documentPath, Map<String, Object> data) {
        log.debug("[updateDocument] updating document at: {} with data: {}", documentPath, data);
        return true;
    }

    @Override
    public boolean writeToCollection(String collectionPath, Map<String, Object> data) {
        log.debug("[writeToCollection] writing to collection at: {} with data: {}", collectionPath, data);
        return true;
    }

    @Override
    public boolean deleteDocument(String path) {
        log.debug("[deleteDocument] deleting document at: {}", path);
        return true;
    }

    @Override
    public boolean deleteCollection(String path, int batchSize) {
        log.debug("[deleteCollection] deleting collection at: {}", path);
        return true;
    }

    @Override
    public List<Map<String, Object>> getCollectionDocumentsData(String path) {
        log.debug("[getCollectionDocumentsData] returning empty data for collection: {}", path);
        return new ArrayList<>();
    }

    @Override
    public Map<String, Object> getDocumentData(String path) {
        log.debug("[getDocumentData] returning empty data for document: {}", path);
        return new HashMap<>();
    }

    @Override
    public void attachCollectionListener(String path, Collectionlistener collectionlistener) {
        log.debug("[attachCollectionListener] {}, {}", path, collectionlistener);
    }

    @Override
    public void attachWhereGreaterThanCollectionListener(String param, Object value, String path, Collectionlistener collectionlistener) {
        log.debug("[attachWhereGreaterThanCollectionListener] {}, {}, {}, {}", param, value, path, collectionlistener);
    }

    @Override
    public String deleteFromStorage(String path) {
        log.debug("[deleteFromStorage] {}", path);
        return "";
    }

    @Override
    public boolean writeToStorage(byte[] data, Map<String, Object> metadata) {
        log.debug("[writeToStorage] in method");
        return true;
    }
}

/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.firebase;

import com.google.api.core.ApiFuture;
import com.google.auth.oauth2.GoogleCredentials;
import com.google.cloud.firestore.*;
import com.google.firebase.FirebaseApp;
import com.google.firebase.FirebaseOptions;
import com.google.firebase.cloud.FirestoreClient;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URI;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

public class DesktopFirebaseConnectionComponent extends FirebaseConnectionComponent {

    private Firestore firestore;
    private boolean useEmulator;

    public DesktopFirebaseConnectionComponent() {
        super();

        firestore = null;
        useEmulator = false;
    }

    @Override
    protected void init() {
        initializeFirebaseApp();

        resetExistingCollections();

        addFirebaseListeners();

        super.init();
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

    private boolean initializeFirebaseApp() {
        try {
            if (useEmulator) {
                FakeCreds credentials = new FakeCreds();
                FirebaseOptions options = new FirebaseOptions.Builder()
                        .setCredentials(credentials)
                        .setProjectId("demo-adecontrol-4c267")
                        .build();
                try {
                    FirebaseApp.initializeApp(options);
                } catch (IllegalStateException e) {
                    log.warn("[init] firebase app instance already exists", e);
                }
                firestore = FirestoreClient.getFirestore();
            } else {
                InputStream serviceAccount = new FileInputStream("C:\\Users\\TR-dev2\\source\\adecontrol-4c267-firebase-adminsdk-f3804-ac996ef58d.json");
                //InputStream serviceAccount = new FileInputStream("/home/brad/Downloads/adecontrol-4c267-firebase-adminsdk-f3804-876f4265b0.json");
                //InputStream serviceAccount = new FileInputStream("/home/will/Downloads/ReusableDownloads/firebase-admin.json");
                //InputStream serviceAccount = new FileInputStream("/home/mfawn/Documents/adecontrol-4c267-firebase-adminsdk-f3804-629731b931.json");
                //InputStream serviceAccount = new FileInputStream("/home/dev/Downloads/adecontrol-4c267-firebase-adminsdk-f3804-ac996ef58d.json");
                //InputStream serviceAccount = new FileInputStream("/home/pete/Downloads/adecontrol-4c267-firebase-adminsdk-f3804-ac996ef58d.json");

                GoogleCredentials credentials = GoogleCredentials.fromStream(serviceAccount);

                FirebaseOptions options = new FirebaseOptions.Builder()
                        .setCredentials(credentials)
                        .setDatabaseUrl("https://adecontrol-4c267.firebaseio.com/")
                        .build();
                FirebaseApp.initializeApp(options);
                firestore = FirestoreClient.getFirestore();
            }
        } catch (IOException e) {
            log.error("Exception in init", e);
            return false;
        }
        return true;
    }

    @Override
    public boolean writeToDocument(String documentPath, Map<String, Object> data) {
        ApiFuture<WriteResult> writeFuture = firestore.document(documentPath).set(data);

        try {
           writeFuture.get();
        } catch (Exception e) {
            log.error("[writeToDocument] error adding document: " + documentPath, e);
            return false;
        }
        log.debug("[writeToDocument] successfully added document: " + documentPath);
        return true;
    }

    @Override
    public boolean updateDocument(String documentPath, Map<String, Object> data) {
        ApiFuture<WriteResult> updateFuture = firestore.document(documentPath).update(data);

        try {
            updateFuture.get();
        } catch (Exception e) {
            log.error("[updateDocument] error updating document: " + documentPath, e);
            return false;
        }
        log.debug("[updateDocument] successfully updated document: " + documentPath);
        return true;
    }

    @Override
    public boolean writeToCollection(String collectionPath, Map<String, Object> data) {
        ApiFuture<DocumentReference> writeFuture = firestore.collection(collectionPath).add(data);

        try {
            writeFuture.get();
        } catch (Exception e) {
            log.error("[writeToCollection] error writing to collection: " + collectionPath, e);
            return false;
        }
        log.debug("[writeToCollection] successfully wrote to collection: " + collectionPath);
        return true;
    }

    @Override
    public boolean deleteDocument(String path) {
        ApiFuture<WriteResult> deleteFuture = firestore.document(path).delete();

        try {
            deleteFuture.get();
        } catch (Exception e) {
            log.error("[deleteDocument] error deleting document: " + path, e);
            return false;
        }
        log.debug("[deleteDocument] successfully deleted document: " + path);
        return true;
    }

    @Override
    public boolean deleteCollection(String path, int batchSize) {
        CollectionReference collection = firestore.collection(path);
        return deleteCollection(collection, batchSize);
    }

    boolean deleteCollection(CollectionReference collection, int batchSize) {
        try {
            // retrieve a small batch of documents to avoid out-of-memory errors
            ApiFuture<QuerySnapshot> future = collection.limit(batchSize).get();
            int deleted = 0;
            // future.get() blocks on document retrieval
            List<QueryDocumentSnapshot> documents = future.get().getDocuments();
            for (QueryDocumentSnapshot document : documents) {
                document.getReference().delete();
                ++deleted;
            }
            if (deleted >= batchSize) {
                // retrieve and delete another batch
                deleteCollection(collection, batchSize);
            }
        } catch (Exception e) {
            System.err.println("Error deleting collection : " + e.getMessage());
            return false;
        }
        return true;
    }

    @Override
    public void attachCollectionListener(String path, Collectionlistener collectionlistener) {
        CollectionReference ref = firestore.collection(path);
        ref.addSnapshotListener((queryDocumentSnapshots, e) -> {
            if (e != null) {
                log.error("Listen failed for collection: " + path, e);
                return;
            }

            log.debug("firestore listener for " + path + "triggered");

            if (queryDocumentSnapshots != null) {
                List<Map<String,Object>> data = new ArrayList<>();
                for (DocumentChange d : queryDocumentSnapshots.getDocumentChanges()) {
                    Map<String,Object> docData = new HashMap<>();
                    docData.put("docId", d.getDocument().getId());
                    docData.put("documentChangeType", d.getType().toString());
                    docData.putAll(d.getDocument().getData());
                    data.add(docData);
                }
                collectionlistener.onCollectionTrigger(data);
            } else {
                log.warn("DB event returned null: " + path);
            }
        });
    }

    @Override
    public void attachWhereGreaterThanCollectionListener(String param, Object value, String path, Collectionlistener collectionlistener) {
        CollectionReference ref = firestore.collection(path);
        ref.whereGreaterThan(param, value).addSnapshotListener(new EventListener<QuerySnapshot>() {

            @Override
            public void onEvent(@javax.annotation.Nullable QuerySnapshot queryDocumentSnapshots, @javax.annotation.Nullable FirestoreException e) {
                if (e != null) {
                    log.error("Listen failed for collection: " + path, e);
                    return;
                }

                log.debug("firestore listener for " + path + "triggered");

                if (queryDocumentSnapshots != null) {
                    List<Map<String,Object>> data = new ArrayList<>();
                    for (DocumentChange d : queryDocumentSnapshots.getDocumentChanges()) {
                        Map<String,Object> docData = new HashMap<>();
                        docData.put("docId", d.getDocument().getId());
                        docData.put("documentChangeType", d.getType().toString());
                        docData.putAll(d.getDocument().getData());
                        data.add(docData);
                    }
                    collectionlistener.onCollectionTrigger(data);
                } else {
                    log.warn("DB event returned null: " + path);
                }
            }
        });
    }

    @Override
    public List<Map<String,Object>> getCollectionDocumentsData(String path) {
        QuerySnapshot snapshot;
        try {
            snapshot = firestore.collection(path).get().get();
        } catch (Exception e) {
            log.error("[getCollectionDocumentsData] error getting document data from collection", e);
            return null;
        }

        List<Map<String,Object>> collectionData = new ArrayList<>();
        for (QueryDocumentSnapshot doc : snapshot.getDocuments()) {
                Map<String,Object> documentData = doc.getData();
                documentData.put("docId", doc.getId());
                collectionData.add(documentData);
        }

        return collectionData;
    }

    @Override
    public Map<String,Object> getDocumentData(String path) {
        try {
            return firestore.document(path).get().get().getData();
        } catch (Exception e) {
            log.error("[getDocumentData] error getting document data", e);
            return null;
        }
    }

    @Override
    public String deleteFromStorage(String path) {
        //"" == success, otherwise error message
        log.error("[deleteFromStorage] not yet implemented on desktop");
        return "not implemented";
    }

    @Override
    public boolean writeToStorage(byte[] data, Map<String,Object> metadata) {
        //Build StorageMetadata from metadata. For now check for ContentType as special field, otherwise treat all as custommetadata
        log.error("[writeToStorage] not yet implemented on desktop");
        return false;
    };

    //EW: For emulated firestore instance need to override GoogleCredentials class in order to
    //    not overwrite admin sdks special security permissions. For emulated demo projects there
    //    shouldn't even have to be any credentials.
    //    (UNAUTHORIZED errors occur even if emulated firestore rules are allow all reads/writes)
    class FakeCreds extends GoogleCredentials {
        Map<String,List<String>> headers;

        public FakeCreds() {
            super();
            headers = new HashMap<>();
            List<String> authLevel = new ArrayList<>();
            authLevel.add("Bearer owner");
            headers.put("Authorization", authLevel);
        }

        @Override
        public Map<String, List<String>> getRequestMetadata(URI uri) {
            return headers;
        }

        @Override
        public boolean hasRequestMetadata() {
            return true;
        }
        @Override
        public boolean hasRequestMetadataOnly() {
            return true;
        }

        @Override
        public void refresh() {}
    }
}

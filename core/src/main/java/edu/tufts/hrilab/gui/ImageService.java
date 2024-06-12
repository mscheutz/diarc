// depreciated. moved to MapGui
//package edu.tufts.hrilab.gui;
//
//import org.apache.commons.imaging.*;
//import org.springframework.stereotype.Service;
//import org.slf4j.Logger;
//import org.slf4j.LoggerFactory;
//
//import java.awt.image.BufferedImage;
//import java.io.IOException;
//import java.nio.file.Path;
//import java.nio.file.Paths;
//
//@Service
//public class ImageService {
//
//    private static final Logger log = LoggerFactory.getLogger(ImageService.class);
//
//    public String convertPGMtoPNG(String pgmPathStr) throws ImageReadException, ImageWriteException, IOException {
//        Path pgmPath = Paths.get(pgmPathStr);
//        String pngFilename = pgmPathStr.replace(".pgm", ".png");
//        Path pngPath = pgmPath.resolveSibling(pngFilename);
//
//        log.info("Reading PGM file from: {}", pgmPath);
//        BufferedImage image = Imaging.getBufferedImage(pgmPath.toFile());
//
//        log.info("Writing PNG file to: {}", pngPath);
//        Imaging.writeImage(image, pngPath.toFile(), ImageFormats.PNG);
//
//        log.info("Converted PGM to PNG: {} to {}", pgmPath, pngPath);
//
//        // Return the relative path to the PNG file, relative to wherever you serve your images from
//        return pngPath.getFileName().toString();
//    }
//
//}
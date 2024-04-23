/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.scale;

import java.nio.ByteBuffer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ScalePacket {

    private int report;
    private int status;
    private int unit;
    private byte exponent;
    private char raw_weight;

    private final static int DATA = 3;
    private final static int GRAMS = 2;
    private final static int OZ = 11;

    private Logger log = LoggerFactory.getLogger(this.getClass());

    public ScalePacket() {
        this.report = 0;
        this.status = 0;
        this.unit = 0;
        this.exponent = 0;
        this.raw_weight = 0;
    }

    public void readFrom(ByteBuffer b){
        this.report = b.get();
        this.status = b.get();
        this.unit = b.get();
        this.exponent = b.get();
//        this.raw_weight = MXTPacketUtil.readUnsignedShort(b);
        this.raw_weight = readUnsignedShort(b);
    }

    /**
     * Reads one unsigned short in little endian form from the given ByteBuffer, and returns it as a char.
     *
     * @param b The ByteBuffer to read from.
     * @return The char representation of the read unsigned short.
     */
    public static char readUnsignedShort(ByteBuffer b) {
        int secondByte = (0x000000FF & ((int)b.get()));
        int firstByte = (0x000000FF & ((int)b.get()));
        return (char) (firstByte << 8 | secondByte);
    }

    public double getWeight() {
        //TODO: Will: hardcoded values for now - do we care about the full functionality of the scale?
        if (this.report != DATA) {
            log.error("Scale not set properly - please adjust");
            return -1.0;
        }
        if (unit == OZ) {
            //28.3495 is the number of grams per one ounce
            return this.raw_weight * Math.pow(10.0, this.exponent) * 28.3495;
        } else if (unit != GRAMS) {
            log.error("Unknown scale unit - please adjust");
            return -1.0;
        }
        return this.raw_weight * Math.pow(10.0, this.exponent);
    }

    public int getSize() {
        return 6;
    }

}

/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.scale;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import org.usb4java.*;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;

public class ScaleComponent extends DiarcComponent {

  private final short VENDOR = 2338;
  private final short PRODUCT = -32765;
  private final int TIMEOUT = 5000;
  private byte IN_ENDPOINT = (byte) 0x82;
  private final byte INTERFACE = 0;
  private final double threshold = 5;

  public ScaleComponent() {
  }

  @Override
  protected void init() {
    int result = LibUsb.init(null);
    if (result != LibUsb.SUCCESS) {
      log.error("Unable to start libusb");
    }
  }

  @TRADEService
  @Action
  public boolean itemWeightGreaterThan(Symbol value) {
    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {
      log.error("[itemWeightIsGreaterThan] sleep interrupted");
    }
    Symbol formattedValue = Factory.createSymbol("n" + value.getName(), value.getType());
    return waitForMeasurement(formattedValue);
  }

  @TRADEService
  public boolean waitForMeasurement(Symbol value) {
    if (!value.getName().substring(0, 1).equalsIgnoreCase("n")) {
      log.error("waitForMeasurement failed to receive properly structured symbol");
      return false;
    } else {
      String weightValue = value.getName().substring(1);
      double target = Double.parseDouble(weightValue);
      boolean reachedTarget = false;
      while (!reachedTarget) {
        double curr = readWeightFromScale();
        if (curr == -1.0) {
          log.error("Scale reading failed");
          return false;
        } else if (curr >= threshold) {
          if (curr >= target) {
            reachedTarget = true;
            log.info("[waitForMeasurement] measurement of " + curr + " is greater than " + target);
          } else {
            log.info("[waitForMeasurement] measurement of " + curr + " is less than " + target);
            return false;
          }
        }
        try {
          Thread.sleep(50);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
    return true;
  }

  public Device findDevice(short vendorId, short productId)
  {
    // Read the USB device list
    DeviceList list = new DeviceList();
    int result = LibUsb.getDeviceList(null, list);
    if (result < 0) throw new LibUsbException("Unable to get device list", result);

    try
    {
      // Iterate over all devices and scan for the right one
      for (Device device: list)
      {
        DeviceDescriptor descriptor = new DeviceDescriptor();
        result = LibUsb.getDeviceDescriptor(device, descriptor);
        if (result != LibUsb.SUCCESS) throw new LibUsbException("Unable to read device descriptor", result);
        if (descriptor.idVendor() == vendorId && descriptor.idProduct() == productId) return device;
      }
    }
    finally
    {
      // Ensure the allocated device list is freed
      LibUsb.freeDeviceList(list, true);
    }

    // Device not found
    return null;
  }

  private double readWeightFromScale() {
//    DeviceHandle handle = LibUsb.openDeviceWithVidPid(null, VENDOR,
//            PRODUCT);
    Device device = findDevice(VENDOR, PRODUCT);
    if (device == null) {
      log.error("[readWeightFromScale] unable to open device");
    }
    DeviceHandle handle = new DeviceHandle();
    int openResult = LibUsb.open(device, handle);
    if (openResult != LibUsb.SUCCESS) {
      log.error("Unable to open USB device. Result code: " + openResult);
      return -1.0;
    }
    boolean detach = (LibUsb.kernelDriverActive(handle, 0) == 1);
    if (detach) {
      int result = LibUsb.detachKernelDriver(handle, INTERFACE);
      if (result != LibUsb.SUCCESS) {
        log.error("Unable to detach kernel from scale");
        return -1.0;
      }
    }
    // Claim the ADB interface
    int result = LibUsb.claimInterface(handle, INTERFACE);
    if (result != LibUsb.SUCCESS) {
      log.error("Unable to claim scale interface");
      return -1.0;
    }
    try {
      ScalePacket p = new ScalePacket();
      ByteBuffer buffer = BufferUtils.allocateByteBuffer(p.getSize()).order(
              ByteOrder.LITTLE_ENDIAN);
      IntBuffer transferred = BufferUtils.allocateIntBuffer();
      result = LibUsb.bulkTransfer(handle, IN_ENDPOINT, buffer,
              transferred, TIMEOUT);
      if (result != LibUsb.SUCCESS) {
        log.error("Unable to read data: " + result);
        throw new LibUsbException("Unable to read data", result);
      }
      p.readFrom(buffer);
      return p.getWeight();
    } finally {
      LibUsb.close(handle);
    }
  }
}

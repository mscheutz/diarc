import sys
import os
import queue
import sounddevice as sd
import vosk
import json
import logging
import time

from pytrade.wrapper import TRADEWrapper
from ai.thinkingrobots.trade import TRADE
import ai.thinkingrobots.trade.TRADEService;
from jpype import JImplements, JOverride, JString
from edu.tufts.hrilab.interfaces import ASRInterface  # Assume you have an ASRInterface defined in Java

# Initialize Logging
logging.basicConfig(stream=sys.stdout, level=logging.INFO)
logging.info("Starting ASR Service...")

model_path = "/Users/mitchellabrams/vosk-model-small-en-us-0.15"
if not os.path.exists(model_path):
    logging.error("Please download and unzip the Vosk model first.")
    sys.exit()

model = vosk.Model(model_path)
samplerate = 16000
device = None  # default input device
q = queue.Queue()

def callback(indata, frames, time_info, status):
    if status:
        logging.warning(f"Sounddevice Status: {status}")
    q.put(bytes(indata))

@JImplements(ASRInterface)
class ASRComponent:
    def __init__(self):
        self.active = False
        self.last_text = ""



    @JOverride
    def startRecognition(self):
        self.active = True
        logging.info("ASR Recognition Started.")

        with sd.RawInputStream(samplerate=samplerate, blocksize=8000, device=device,
                               dtype='int16', channels=1, callback=callback):
            rec = vosk.KaldiRecognizer(model, samplerate)

            while True:
                data = q.get()
                if not self.active:
                    break

                if rec.AcceptWaveform(data):
                    result = json.loads(rec.Result())
                    final_text = result.get("text", "").strip()

                    if not final_text:
                        final_text = "No speech recognized"

                    self.last_text = final_text
                    logging.info(f"[ASR] Final: {final_text}")
                    print(f"Recognized Text (Python): {final_text}")

                    #java_string = JString(final_text)  # Convert to a proper Java String
                    #logging.info(f"Returning to Java as JString: {java_string}")
                    return final_text  # Return as Java String

        logging.info("Returning default 'No speech recognized' to Java")
        return "No speech recognized"  # Ensure return is always a Java String


    @JOverride
    def stopRecognition(self):
        self.active = False
        logging.info("Stopping ASR Service...")

    @JOverride
    def resetRecognition(self):
        self.last_text = ""
        logging.info("Resetting ASR Service...")

    @JOverride
    def getLastRecognizedText(self):
        logging.info("Calling getLastRecognizedText")
        test_value = "test message from ASR"
        print(f"Sending test text to Java: {test_value}")  # Explicitly print
        return test_value  # Hardcoded test string

    @JOverride
    def isRecognitionActive(self):
        return self.active

if __name__ == '__main__':
    wrapper = TRADEWrapper()
    asr_component = ASRComponent()
    TRADE.registerAllServices(asr_component, "")

    logging.info("ASR Service registered with TRADE.")
    #asr_component.startRecognition()
    time.sleep(4)
    #wrapper.call_trade("getLastRecognizedText")
    #wrapper.call_trade("stopRecognition")

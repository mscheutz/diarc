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
import ai.thinkingrobots.trade.TRADEService
from jpype import JImplements, JOverride
from edu.tufts.hrilab.interfaces import ASRInterface  # Assuming ASRInterface is defined in Java

# Initialize Logging
logging.basicConfig(stream=sys.stdout, level=logging.INFO)
logging.info("Starting ASR Service...")

# Load ASR Model
model_path = "/Users/mitchellabrams/vosk-model-small-en-us-0.15"
if not os.path.exists(model_path):
    logging.error("Please download and unzip the Vosk model first.")
    sys.exit()

model = vosk.Model(model_path)
samplerate = 16000
device = None  # Use default input device
q = queue.Queue()

# Audio Callback
def callback(indata, frames, time_info, status):
    if status:
        logging.warning(f"Sounddevice Status: {status}")
    q.put(bytes(indata))

@JImplements(ASRInterface)
class ASRComponent:
    def __init__(self):
        self.active = False
        self.last_text = ""
        self.partial_text = ""

    @JOverride
    def startRecognition(self):
        self.active = True
        logging.info("ASR Recognition Started.")

        with sd.RawInputStream(samplerate=samplerate, blocksize=8000, device=device,
                               dtype='int16', channels=1, callback=callback):
            rec = vosk.KaldiRecognizer(model, samplerate)

            while self.active:
                data = q.get()
                if not self.active:
                    break

                if rec.AcceptWaveform(data):  # Final recognized speech
                    result = json.loads(rec.Result())
                    self.last_text = result.get("text", "")
                    logging.info(f"[ASR] Final: {self.last_text}")

                    # Send full recognized sentence
                    TRADE.call_service("incrementalParse", self.last_text)

                else:  # Partial results for real-time incremental parsing
                    partial = json.loads(rec.PartialResult())
                    new_partial = partial.get("partial", "").strip()

                    # Only send new words if they haven't been sent before
                    if new_partial and new_partial != self.partial_text:
                        self.partial_text = new_partial
                        logging.info(f"[ASR] Incremental Parse: {self.partial_text}")
                        TRADE.call_service("incrementalParse", self.partial_text)

    @JOverride
    def stopRecognition(self):
        self.active = False
        logging.info("Stopping ASR Service...")

    @JOverride
    def resetRecognition(self):
        self.last_text = ""
        self.partial_text = ""
        logging.info("Resetting ASR Service...")

    @JOverride
    def getLastRecognizedText(self):
        return self.last_text

    @JOverride
    def isRecognitionActive(self):
        return self.active

if __name__ == '__main__':
    wrapper = TRADEWrapper()
    asr_component = ASRComponent()
    TRADE.registerAllServices(asr_component, "")

    logging.info("ASR Service registered with TRADE.")
    asr_component.startRecognition()
    time.sleep(1)  # Allow time for ASR to process
    wrapper.call_trade("getLastRecognizedText")
    wrapper.call_trade("stopRecognition")

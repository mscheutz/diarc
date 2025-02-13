import json
import logging
import os
import queue
import re
import sys
import time

import ai.thinkingrobots.trade.TRADEService
import sounddevice as sd
import vosk
from ai.thinkingrobots.trade import TRADE
from edu.tufts.hrilab.interfaces import (
    ASRInterface,
)  # Assume you have an ASRInterface defined in Java
from jpype import JImplements, JOverride, JString
from pytrade.wrapper import TRADEWrapper

# Initialize Logging
logging.basicConfig(stream=sys.stdout, level=logging.INFO)
logging.info("Starting ASR Service...")

model_path = (
    "/home/toeleric/research/boundmpc_speech/src/diarc/vosk-model-small-en-us-0.15"
)
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

        # ✅ Define the action mapping dictionary inside the class
        self.ACTION_MAP = {
            # Grasp mug by a specific part
            r"INSTRUCT\(speaker,listener,graspObject\(listener,mug\),\{by\(mug,handle\)\}\)": "GRASP(MUG_BY_HANDLE)",
            r"INSTRUCT\(speaker,listener,graspObject\(listener,mug\),\{by\(mug,top\)\}\)": "GRASP(MUG_BY_TOP)",
            r"INSTRUCT\(speaker,listener,graspObject\(listener,mug\),\{from\(mug,top\)\}\)": "GRASP(MUG_BY_TOP)",
            # These map to the same action
            r"INSTRUCT\(speaker,listener,graspObject\(listener,mug\),\{by\(mug,bottom\)\}\)": "GRASP(MUG_BY_BOTTOM)",
            r"INSTRUCT\(speaker,listener,graspObject\(listener,mug\),\{from\(mug,bottom\)\}\)": "GRASP(MUG_BY_BOTTOM)",
            # Grasping near a specific part
            r"INSTRUCT\(speaker,listener,graspObject\(listener,mug\),\{near\(mug,handle\)\}\)": "GRASP(MUG_NEAR_HANDLE)",
            # General grasping
            r"INSTRUCT\(speaker,listener,graspObject\(listener,mug\),\{\}\)": "GRASP(MUG_DEFAULT)",
            # Grasping mug from table
            r"INSTRUCT\(speaker,listener,graspObject\(listener,mug\),\{on\(mug,table\)\}\)": "GRASP(MUG_ON_TABLE)",
            # Greeting messages
            r"^GREETING\(.*\)": "SAY(GREETING)",
        }

    def action_mapper(self, parsed_text):
        """Map parsed text to a robot action using regex pattern matching."""
        import re  # ✅ Import regex module

        for pattern, action in self.ACTION_MAP.items():
            if re.fullmatch(pattern, parsed_text):
                return action  # ✅ Return the mapped action

        return "NONE"  # ✅ Default to "NONE" if no match is found

    @JOverride
    def startRecognition(self):
        self.active = True
        logging.info("ASR Recognition Started.")

        with sd.RawInputStream(
            samplerate=samplerate,
            blocksize=8000,
            device=device,
            dtype="int16",
            channels=1,
            callback=callback,
        ):
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

                    # java_string = JString(final_text)  # Convert to a proper Java String
                    # logging.info(f"Returning to Java as JString: {java_string}")
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

    @JOverride
    def logParse(self, parsed_text):
        """Receive and print any parse result from Java."""
        logging.info(f"[Python] Received Parse Result: {parsed_text}")
        # print(f"[Python] Parsed Utterance: {parsed_text}")

    @JOverride
    def mapParsedTextToAction(self, parsed_text):
        """Map parsed text to an action and log it."""
        try:
            # ✅ Convert Java String to native Python string
            if isinstance(parsed_text, JString):
                parsed_text = str(parsed_text)
            # logging.info(f"[Python] Mapping parsed text to action: {parsed_text}")

            # ✅ Call the dictionary-based action mapping function
            action = self.action_mapper(parsed_text)

            if action is None:
                action = "NONE"

            logging.info(f"[Python] Mapped action: {action}")
            return action

        except Exception as e:
            logging.error(f"Error in mapParsedTextToAction: {str(e)}")
            return (
                "ERROR"  # ✅ Return a safe default string to prevent Java from crashing
            )


if __name__ == "__main__":
    wrapper = TRADEWrapper()
    asr_component = ASRComponent()
    TRADE.registerAllServices(asr_component, "")

    logging.info("ASR Service registered with TRADE.")
    # asr_component.startRecognition()
    time.sleep(4)
    # wrapper.call_trade("getLastRecognizedText")
    # wrapper.call_trade("stopRecognition")

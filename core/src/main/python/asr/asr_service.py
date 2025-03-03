import datetime
import json
import logging
import os
import queue
import re
import sys
import time

from pytrade.wrapper import TRADEWrapper
import ai.thinkingrobots.trade.TRADEService
import sounddevice as sd
import vosk
from ai.thinkingrobots.trade import TRADE
from edu.tufts.hrilab.interfaces import (
    ASRInterface,
)  # Assume you have an ASRInterface defined in Java
from jpype import JImplements, JOverride, JString

# Initialize Logging
formatter = logging.Formatter(
    fmt="%(asctime)s.%(msecs)03d - %(name)s - %(levelname)s - %(message)s",
    datefmt="%H:%M:%S",
)
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(formatter)
logger = logging.getLogger("Logger")
logger.addHandler(handler)
logger.info("Starting ASR Service...")

# model_path = (
#     "/home/toeleric/research/boundmpc_speech/src/diarc/vosk-model-small-en-us-0.15"
# )
model_path = (
    "/home/toeleric/research/boundmpc_speech/src/diarc/vosk-model-en-us-0.22"
)
if not os.path.exists(model_path):
    logging.error("Please download and unzip the Vosk model first.")
    sys.exit()

model = vosk.Model(model_path)
samplerate = 16000
# sd.default.device = 11
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
        self.rec = vosk.KaldiRecognizer(model, samplerate)


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
            # General passing
            r"INSTRUCT\(speaker,listener,pass\(listener,mug\),\{\}\)": "PASS(MUG_DEFAULT)",
            r"INSTRUCT\(speaker,listener,pass\(listener,cup\),\{\}\)": "GRASP(CUP_DEFAULT)",
            r"INSTRUCT\(speaker,listener,pass\(listener,screwdriver\),\{\}\)": "GRASP(SCREWDRIVER_DEFAULT)",
            # Grasping mug from table
            r"INSTRUCT\(speaker,listener,graspObject\(listener,mug\),\{on\(mug,table\)\}\)": "GRASP(MUG_ON_TABLE)",
            # Greeting messages
            r"^GREETING\(.*\)": "SAY(GREETING)",

            # Pass the mug while keeping it upright
            r"INSTRUCT\(speaker,listener,and\(pass\(listener,mug\),keep\(listener,upright\)\),\{\}\)": "PASS(MUG,MANNER(UPRIGHT))",

            # Pass the cup while keeping it upright
            r"INSTRUCT\(speaker,listener,and\(pass\(listener,cup\),keep\(listener,upright\)\),\{\}\)": "PASS(CUP,MANNER(UPRIGHT))",

            # Pass the bottle while keeping it upright
            r"INSTRUCT\(speaker,listener,and\(pass\(listener,bottle\),keep\(listener,upright\)\),\{\}\)": "PASS(BOTTLE,MANNER(UPRIGHT))",

            # Pass the mug while avoiding the laptop
            r"INSTRUCT\(speaker,listener,and\(pass\(listener,mug\),avoid\(listener,laptop\)\),\{\}\)": "PASS(MUG, AVOID(AGENT, LAPTOP))",

            # Pass the screwdriver while going faster
            r"INSTRUCT\(speaker,listener,and\(pass\(listener,mug\),go\(listener,faster\)\),\{\}\)": "PASS(SCREWDRIVER, GO(AGENT, FASTER))",

            # Put the apple on the towel in the box
            r"INSTRUCT(speaker,listener,placeObject(listener,apple),{in(towel,box),on(apple,towel)})": "PUT(APPLE, ON(APPLE, TOWEL), IN(APPLE,BOX))",

            # Put the apple on the towel in the box
            r"INSTRUCT(speaker,listener,placeObject(listener,apple),{on(apple,towel)})": "PUT(APPLE, ON(APPLE, TOWEL))"

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
        logger.info("ASR Recognition Started.")

        with sd.RawInputStream(
            samplerate=samplerate,
            blocksize=10,
            device=device,
            dtype="int16",
            channels=1,
            callback=callback,
        ):
            while True:
                data = q.get()
                if not self.active:
                    break

                if self.rec.AcceptWaveform(data):
                    result = json.loads(self.rec.Result())
                    final_text = result.get("text", "").strip()

                    if not final_text:
                        final_text = "No speech recognized"

                    self.last_text = final_text
                    logger.info(f"[ASR] Final: {final_text}")
                    # print(f"Recognized Text (Python): {final_text}")

                    # java_string = JString(final_text)  # Convert to a proper Java String
                    # logger.info(f"Returning to Java as JString: {java_string}")
                    return final_text  # Return as Java String

        logger.info("Returning default 'No speech recognized' to Java")
        return "No speech recognized"  # Ensure return is always a Java String

    @JOverride
    def stopRecognition(self):
        self.active = False
        logger.info("Stopping ASR Service...")

    @JOverride
    def resetRecognition(self):
        self.last_text = ""
        logger.info("Resetting ASR Service...")

    @JOverride
    def getLastRecognizedText(self):
        logger.info("Calling getLastRecognizedText")
        test_value = "test message from ASR"
        print(f"Sending test text to Java: {test_value}")  # Explicitly print
        return test_value  # Hardcoded test string

    @JOverride
    def isRecognitionActive(self):
        return self.active

    @JOverride
    def logParse(self, parsed_text):
        """Receive and print any parse result from Java."""
        logger.info(f"[Python] Received Parse Result: {parsed_text}")
        # print(f"[Python] Parsed Utterance: {parsed_text}")

    @JOverride
    def mapParsedTextToAction(self, parsed_text):
        """Map parsed text to an action and log it."""
        try:
            # ✅ Convert Java String to native Python string
            if isinstance(parsed_text, JString):
                parsed_text = str(parsed_text)
            # logger.info(f"[Python] Mapping parsed text to action: {parsed_text}")

            # ✅ Call the dictionary-based action mapping function
            parsed_text = self.action_mapper(parsed_text)

            if parsed_text is None:
                parsed_text = "NONE"

            logger.info(f"[Python] Mapped action: {parsed_text}")
            return parsed_text

        except Exception as e:
            logging.error(f"Error in mapParsedTextToAction: {str(e)}")
            return (
                "ERROR"  # ✅ Return a safe default string to prevent Java from crashing
            )


if __name__ == "__main__":
    wrapper = TRADEWrapper()
    asr_component = ASRComponent()
    TRADE.registerAllServices(asr_component, "")

    logger.info("ASR Service registered with TRADE.")
    # asr_component.startRecognition()
    while True:
        time.sleep(10)
    # wrapper.call_trade("getLastRecognizedText")
    # wrapper.call_trade("stopRecognition")

#Contains modified relevant classes from the whisper_streaming project https://github.com/ufal/whisper_streaming
from faster_whisper import WhisperModel
import openwakeword
from openwakeword.model import Model
import pyaudio
import sys
import numpy as np
import time
import logging

import io
import math
import torch

logger = logging.getLogger(__name__)
SAMPLING_RATE = 16000

class VADIterator:
    def __init__(self,
                 model,
                 threshold: float = 0.5,
                 sampling_rate: int = SAMPLING_RATE,
                 min_silence_duration_ms: int = 500,  # makes sense on one recording that I checked
                 speech_pad_ms: int = 100             # same
                 ):

        """
        Class for stream imitation

        Parameters
        ----------
        model: preloaded .jit silero VAD model

        threshold: float (default - 0.5)
            Speech threshold. Silero VAD outputs speech probabilities for each audio chunk, probabilities ABOVE this value are considered as SPEECH.
            It is better to tune this parameter for each dataset separately, but "lazy" 0.5 is pretty good for most datasets.

        sampling_rate: int (default - 16000)
            Currently silero VAD models support 8000 and 16000 sample rates

        min_silence_duration_ms: int (default - 100 milliseconds)
            In the end of each speech chunk wait for min_silence_duration_ms before separating it

        speech_pad_ms: int (default - 30 milliseconds)
            Final speech chunks are padded by speech_pad_ms each side
        """

        self.model = model
        self.threshold = threshold
        self.sampling_rate = sampling_rate

        if sampling_rate not in [8000, 16000]:
            raise ValueError('VADIterator does not support sampling rates other than [8000, 16000]')

        self.min_silence_samples = sampling_rate * min_silence_duration_ms / 1000
        self.speech_pad_samples = sampling_rate * speech_pad_ms / 1000
        self.reset_states()

    def reset_states(self):

        self.model.reset_states()
        self.triggered = False
        self.temp_end = 0
        self.current_sample = 0

    def __call__(self, x, return_seconds=False):
        """
        x: torch.Tensor
            audio chunk (see examples in repo)

        return_seconds: bool (default - False)
            whether return timestamps in seconds (default - samples)
        """

        if not torch.is_tensor(x):
            try:
                x = torch.Tensor(x)
            except:
                raise TypeError("Audio cannot be casted to tensor. Cast it manually")

        window_size_samples = len(x[0]) if x.dim() == 2 else len(x)
        self.current_sample += window_size_samples

        speech_prob = self.model(x, self.sampling_rate).item()

        if (speech_prob >= self.threshold) and self.temp_end:
            self.temp_end = 0

        if (speech_prob >= self.threshold) and not self.triggered:
            self.triggered = True
            speech_start = self.current_sample - self.speech_pad_samples
            return {'start': int(speech_start) if not return_seconds else round(speech_start / self.sampling_rate, 1)}

        if (speech_prob < self.threshold - 0.15) and self.triggered:
            if not self.temp_end:
                self.temp_end = self.current_sample
            if self.current_sample - self.temp_end < self.min_silence_samples:
                return None
            else:
                speech_end = self.temp_end + self.speech_pad_samples
                self.temp_end = 0
                self.triggered = False
                return {'end': int(speech_end) if not return_seconds else round(speech_end / self.sampling_rate, 1)}

        return None

class FixedVADIterator(VADIterator):
    '''It fixes VADIterator by allowing to process any audio length, not only exactly 512 frames at once.
    If audio to be processed at once is long and multiple voiced segments detected,
    then __call__ returns the start of the first segment, and end (or middle, which means no end) of the last segment.
    '''

    def reset_states(self):
        super().reset_states()
        self.buffer = np.array([],dtype=np.float32)

    def __call__(self, x, return_seconds=False):
        self.buffer = np.append(self.buffer, x)
        ret = None
        while len(self.buffer) >= 512:
            r = super().__call__(self.buffer[:512], return_seconds=return_seconds)
            self.buffer = self.buffer[512:]
            if ret is None:
                ret = r
            elif r is not None:
                if 'end' in r:
                    ret['end'] = r['end']  # the latter end
                if 'start' in r and 'end' in ret:  # there is an earlier start.
                    # Remove end, merging this segment with the previous one.
                    del ret['end']
        return ret if ret != {} else None

class ASRBase:

    sep = " "   # join transcribe words with this character (" " for whisper_timestamped,
    # "" for faster-whisper because it emits the spaces when neeeded)

    def __init__(self, lan, modelsize=None, cache_dir=None, model_dir=None, logfile=sys.stderr):
        self.logfile = logfile

        self.transcribe_kargs = {}
        if lan == "auto":
            self.original_language = None
        else:
            self.original_language = lan

        self.model = self.load_model(modelsize, cache_dir, model_dir)


    def load_model(self, modelsize, cache_dir):
        raise NotImplemented("must be implemented in the child class")

    def transcribe(self, audio, init_prompt=""):
        raise NotImplemented("must be implemented in the child class")

    def use_vad(self):
        raise NotImplemented("must be implemented in the child class")


class FasterWhisperASR(ASRBase):
    """Uses faster-whisper library as the backend. Works much faster, appx 4-times (in offline mode). For GPU, it requires installation with a specific CUDNN version.
    """

    sep = ""

    def load_model(self, modelsize=None, cache_dir=None, model_dir=None):
        from faster_whisper import WhisperModel
        if model_dir is not None:
            #print(f"Loading whisper model from model_dir {model_dir}. modelsize and cache_dir parameters are not used.")
            model_size_or_path = model_dir
        elif modelsize is not None:
            model_size_or_path = modelsize
        else:
            raise ValueError("modelsize or model_dir parameter must be set")


        # this worked fast and reliably on NVIDIA L40
        # model = WhisperModel(model_size_or_path, device="cuda", compute_type="float16", download_root=cache_dir)

        # or run on GPU with INT8
        # tested: the transcripts were different, probably worse than with FP16, and it was slightly (appx 20%) slower
        #model = WhisperModel(model_size, device="cuda", compute_type="int8_float16")

        # or run on CPU with INT8
        # tested: works, but slow, appx 10-times than cuda FP16
        model = WhisperModel(modelsize, device="cpu", compute_type="int8")
        return model

    def transcribe(self, audio, init_prompt=""):

        # tested: beam_size=5 is faster and better than 1 (on one 200 second document from En ESIC, min chunk 0.01)
        segments, info = self.model.transcribe(audio, language=self.original_language, initial_prompt=init_prompt, beam_size=5, word_timestamps=True, condition_on_previous_text=True, **self.transcribe_kargs)
        ##print(info)  # info contains language detection result

        return list(segments)

    def ts_words(self, segments):
        o = []
        for segment in segments:
            for word in segment.words:
                if segment.no_speech_prob > 0.9:
                    continue
                # not stripping the spaces -- should not be merged with them!
                w = word.word
                t = (word.start, word.end, w)
                o.append(t)
        return o

    def segments_end_ts(self, res):
        return [s.end for s in res]

    def use_vad(self):
        self.transcribe_kargs["vad_filter"] = True

    def set_translate_task(self):
        self.transcribe_kargs["task"] = "translate"

class HypothesisBuffer:

    def __init__(self, logfile=sys.stderr):
        self.commited_in_buffer = []
        self.buffer = []
        self.new = []

        self.last_commited_time = 0
        self.last_commited_word = None

        self.logfile = logfile

    def insert(self, new, offset):
        # compare self.commited_in_buffer and new. It inserts only the words in new that extend the commited_in_buffer, it means they are roughly behind last_commited_time and new in content
        # the new tail is added to self.new

        new = [(a+offset,b+offset,t) for a,b,t in new]
        self.new = [(a,b,t) for a,b,t in new if a > self.last_commited_time-0.1]

        if len(self.new) >= 1:
            a,b,t = self.new[0]
            if abs(a - self.last_commited_time) < 1:
                if self.commited_in_buffer:
                    # it's going to search for 1, 2, ..., 5 consecutive words (n-grams) that are identical in commited and new. If they are, they're dropped.
                    cn = len(self.commited_in_buffer)
                    nn = len(self.new)
                    for i in range(1,min(min(cn,nn),5)+1):  # 5 is the maximum
                        c = " ".join([self.commited_in_buffer[-j][2] for j in range(1,i+1)][::-1])
                        tail = " ".join(self.new[j-1][2] for j in range(1,i+1))
                        if c == tail:
                            words = []
                            for j in range(i):
                                words.append(repr(self.new.pop(0)))
                            words_msg = " ".join(words)
                            #print(f"removing last {i} words: {words_msg}")
                            break

    def flush(self):
        # returns commited chunk = the longest common prefix of 2 last inserts.

        commit = []
        while self.new:
            na, nb, nt = self.new[0]

            if len(self.buffer) == 0:
                break

            if nt == self.buffer[0][2]:
                commit.append((na,nb,nt))
                self.last_commited_word = nt
                self.last_commited_time = nb
                self.buffer.pop(0)
                self.new.pop(0)
            else:
                break
        self.buffer = self.new
        self.new = []
        self.commited_in_buffer.extend(commit)
        return commit

    def pop_commited(self, time):
        while self.commited_in_buffer and self.commited_in_buffer[0][1] <= time:
            self.commited_in_buffer.pop(0)

    def complete(self):
        return self.buffer

class OnlineASRProcessor:

    def __init__(self, asr, tokenizer=None, buffer_trimming=("segment", 15), logfile=sys.stderr):
        """asr: WhisperASR object
        tokenizer: sentence tokenizer object for the target language. Must have a method *split* that behaves like the one of MosesTokenizer. It can be None, if "segment" buffer trimming option is used, then tokenizer is not used at all.
        ("segment", 15)
        buffer_trimming: a pair of (option, seconds), where option is either "sentence" or "segment", and seconds is a number. Buffer is trimmed if it is longer than "seconds" threshold. Default is the most recommended option.
        logfile: where to store the log.
        """
        self.asr = asr
        self.tokenizer = tokenizer
        self.logfile = logfile

        self.init()

        self.buffer_trimming_way, self.buffer_trimming_sec = buffer_trimming

    def init(self, offset=None):
        """run this when starting or restarting processing"""
        self.audio_buffer = np.array([],dtype=np.float32)
        self.transcript_buffer = HypothesisBuffer(logfile=self.logfile)
        self.buffer_time_offset = 0
        if offset is not None:
            self.buffer_time_offset = offset
        self.transcript_buffer.last_commited_time = self.buffer_time_offset
        self.commited = []

    def insert_audio_chunk(self, audio):
        self.audio_buffer = np.append(self.audio_buffer, audio)

    def prompt(self):
        """Returns a tuple: (prompt, context), where "prompt" is a 200-character suffix of commited text that is inside of the scrolled away part of audio buffer.
        "context" is the commited text that is inside the audio buffer. It is transcribed again and skipped. It is returned only for debugging and logging reasons.
        """
        k = max(0,len(self.commited)-1)
        while k > 0 and self.commited[k-1][1] > self.buffer_time_offset:
            k -= 1

        p = self.commited[:k]
        p = [t for _,_,t in p]
        prompt = []
        l = 0
        while p and l < 200:  # 200 characters prompt size
            x = p.pop(-1)
            l += len(x)+1
            prompt.append(x)
        non_prompt = self.commited[k:]
        return self.asr.sep.join(prompt[::-1]), self.asr.sep.join(t for _,_,t in non_prompt)

    def process_iter(self):
        """Runs on the current audio buffer.
        Returns: a tuple (beg_timestamp, end_timestamp, "text"), or (None, None, "").
        The non-emty text is confirmed (committed) partial transcript.
        """

        prompt, non_prompt = self.prompt()
        print(f"PROMPT: {prompt}", file=sys.stderr)
        print(f"CONTEXT: {non_prompt}", file=sys.stderr)
        print(f"transcribing {len(self.audio_buffer)/SAMPLING_RATE:2.2f} seconds from {self.buffer_time_offset:2.2f}", file=sys.stderr)
        res = self.asr.transcribe(self.audio_buffer, init_prompt=prompt)

        # transform to [(beg,end,"word1"), ...]
        tsw = self.asr.ts_words(res)

        self.transcript_buffer.insert(tsw, self.buffer_time_offset)
        o = self.transcript_buffer.flush()
        self.commited.extend(o)
        completed = self.to_flush(o)
        print(f">>>>COMPLETE NOW: {completed}", file=sys.stderr)
        the_rest = self.to_flush(self.transcript_buffer.complete())
        print(f"INCOMPLETE: {the_rest}", file=sys.stderr)

        # there is a newly confirmed text

        if o and self.buffer_trimming_way == "sentence":  # trim the completed sentences
            if len(self.audio_buffer)/SAMPLING_RATE > self.buffer_trimming_sec:  # longer than this
                self.chunk_completed_sentence()


        if self.buffer_trimming_way == "segment":
            s = self.buffer_trimming_sec  # trim the completed segments longer than s,
        else:
            s = 30 # if the audio buffer is longer than 30s, trim it

        if len(self.audio_buffer)/SAMPLING_RATE > s:
            self.chunk_completed_segment(res)

            # alternative: on any word
            #l = self.buffer_time_offset + len(self.audio_buffer)/self.SAMPLING_RATE - 10
            # let's find commited word that is less
            #k = len(self.commited)-1
            #while k>0 and self.commited[k][1] > l:
            #    k -= 1
            #t = self.commited[k][1]
            #print("chunking segment")
            #self.chunk_at(t)

        print(f"len of buffer now: {len(self.audio_buffer)/SAMPLING_RATE:2.2f}", file=sys.stderr)
        return self.to_flush(o)

    def chunk_completed_sentence(self):
        if self.commited == []: return
        print(self.commited, file=sys.stderr)
        sents = self.words_to_sentences(self.commited)
        for s in sents:
            print(f"\t\tSENT: {s}", file=sys.stderr)
        if len(sents) < 2:
            return
        while len(sents) > 2:
            sents.pop(0)
        # we will continue with audio processing at this timestamp
        chunk_at = sents[-2][1]

        print(f"--- sentence chunked at {chunk_at:2.2f}", file=sys.stderr)
        self.chunk_at(chunk_at)

    def chunk_completed_segment(self, res):
        if self.commited == []: return

        ends = self.asr.segments_end_ts(res)

        t = self.commited[-1][1]

        if len(ends) > 1:

            e = ends[-2]+self.buffer_time_offset
            while len(ends) > 2 and e > t:
                ends.pop(-1)
                e = ends[-2]+self.buffer_time_offset
            if e <= t:
                print(f"--- segment chunked at {e:2.2f}", file=sys.stderr)
                self.chunk_at(e)
            else:
                print(f"--- last segment not within commited area", file=sys.stderr)
        else:
            print(f"--- not enough segments to chunk", file=sys.stderr)

    def chunk_at(self, time):
        """trims the hypothesis and audio buffer at "time"
        """
        self.transcript_buffer.pop_commited(time)
        cut_seconds = time - self.buffer_time_offset
        self.audio_buffer = self.audio_buffer[int(cut_seconds*SAMPLING_RATE):]
        self.buffer_time_offset = time

    def words_to_sentences(self, words):
        """Uses self.tokenizer for sentence segmentation of words.
        Returns: [(beg,end,"sentence 1"),...]
        """

        cwords = [w for w in words]
        t = " ".join(o[2] for o in cwords)
        s = self.tokenizer.split(t)
        out = []
        while s:
            beg = None
            end = None
            sent = s.pop(0).strip()
            fsent = sent
            while cwords:
                b,e,w = cwords.pop(0)
                w = w.strip()
                if beg is None and sent.startswith(w):
                    beg = b
                elif end is None and sent == w:
                    end = e
                    out.append((beg,end,fsent))
                    break
                sent = sent[len(w):].strip()
        return out

    def finish(self):
        """Flush the incomplete text when the whole processing ends.
        Returns: the same format as self.process_iter()
        """
        o = self.transcript_buffer.complete()
        f = self.to_flush(o)
        print(f"last, noncommited: {f}", file=sys.stderr)
        self.buffer_time_offset += len(self.audio_buffer)/16000
        return f


    def to_flush(self, sents, sep=None, offset=0, ):
        # concatenates the timestamped words or sentences into one sequence that is flushed in one line
        # sents: [(beg1, end1, "sentence1"), ...] or [] if empty
        # return: (beg1,end-of-last-sentence,"concatenation of sentences") or (None, None, "") if empty
        if sep is None:
            sep = self.asr.sep
        t = sep.join(s[2] for s in sents)
        if len(sents) == 0:
            b = None
            e = None
        else:
            b = offset + sents[0][0]
            e = offset + sents[-1][1]
        return (b,e,t,'voice')

class VACOnlineASRProcessor(OnlineASRProcessor):
    '''Wraps OnlineASRProcessor with VAC (Voice Activity Controller).

    It works the same way as OnlineASRProcessor: it receives chunks of audio (e.g. 0.04 seconds),
    it runs VAD and continuously detects whether there is speech or not.
    When it detects end of speech (non-voice for 500ms), it makes OnlineASRProcessor to end the utterance immediately.
    '''

    def __init__(self, online_chunk_size, *a, **kw):
        self.online_chunk_size = online_chunk_size

        self.online = OnlineASRProcessor(*a, **kw)

        # VAC:
        import torch
        model, _ = torch.hub.load(
            repo_or_dir='snakers4/silero-vad',
            model='silero_vad'
        )
        self.vac = FixedVADIterator(model)  # we use the default options there: 500ms silence, 100ms padding, etc.

        self.logfile = self.online.logfile
        self.init()

    def init(self):
        self.online.init()
        self.vac.reset_states()
        self.current_online_chunk_buffer_size = 0

        self.is_currently_final = False

        self.status = None  # or "voice" or "nonvoice"
        self.audio_buffer = np.array([],dtype=np.float32)
        self.buffer_offset = 0  # in frames

    def clear_buffer(self):
        self.buffer_offset += len(self.audio_buffer)
        self.audio_buffer = np.array([],dtype=np.float32)


    def insert_audio_chunk(self, audio):
        res = self.vac(audio)
        self.audio_buffer = np.append(self.audio_buffer, audio)

        if res is not None:
            frame = list(res.values())[0]-self.buffer_offset
            if 'start' in res and 'end' not in res:
                self.status = 'voice'
                send_audio = self.audio_buffer[frame:]
                self.online.init(offset=(frame+self.buffer_offset)/SAMPLING_RATE)
                self.online.insert_audio_chunk(send_audio)
                self.current_online_chunk_buffer_size += len(send_audio)
                self.clear_buffer()
            elif 'end' in res and 'start' not in res:
                self.status = 'nonvoice'
                send_audio = self.audio_buffer[:frame]
                self.online.insert_audio_chunk(send_audio)
                self.current_online_chunk_buffer_size += len(send_audio)
                self.is_currently_final = True
                self.clear_buffer()
            else:
                beg = res["start"]-self.buffer_offset
                end = res["end"]-self.buffer_offset
                self.status = 'nonvoice'
                send_audio = self.audio_buffer[beg:end]
                self.online.init(offset=(beg+self.buffer_offset)/SAMPLING_RATE)
                self.online.insert_audio_chunk(send_audio)
                self.current_online_chunk_buffer_size += len(send_audio)
                self.is_currently_final = True
                self.clear_buffer()
        else:
            if self.status == 'voice':
                self.online.insert_audio_chunk(self.audio_buffer)
                self.current_online_chunk_buffer_size += len(self.audio_buffer)
                self.clear_buffer()
            else:
                # We keep 1 second because VAD may later find start of voice in it.
                # But we trim it to prevent OOM.
                self.buffer_offset += max(0,len(self.audio_buffer)-SAMPLING_RATE)
                self.audio_buffer = self.audio_buffer[-SAMPLING_RATE:]


    def process_iter(self):
        if self.is_currently_final:
            return self.finish()
        elif self.current_online_chunk_buffer_size > SAMPLING_RATE*self.online_chunk_size:
            self.current_online_chunk_buffer_size = 0
            ret = self.online.process_iter()
            return ret
        else:
            ##print("no online update, only VAD", self.status, file=self.logfile)
            return (None, None, "", self.status)

    def finish(self):
        text = ''
        #Process trailing audio data that does not reach online_chunk_size time. Helpful for one word utterances shorter
        #   than chunk_size seconds and for capturing the tail end of inputs
        if 0 < self.current_online_chunk_buffer_size <= SAMPLING_RATE*self.online_chunk_size:
           print("Processing incomplete input...", file=sys.stderr)
           self.current_online_chunk_buffer_size = 0
           iter_output = self.online.process_iter()
           if iter_output[2] != '':
               text = iter_output[2]
        ret = self.online.finish()
        ret = (ret[0],ret[1],text+ret[2],ret[3])
        self.current_online_chunk_buffer_size = 0
        self.is_currently_final = False
        return ret

PACKET_SIZE = 65536


def send_one_line(socket, text, pad_zeros=False):
    """Sends a line of text over the given socket.

    The 'text' argument should contain a single line of text (line break
    characters are optional). Line boundaries are determined by Python's
    str.splitlines() function [1]. We also count '\0' as a line terminator.
    If 'text' contains multiple lines then only the first will be sent.

    If the send fails then an exception will be raised.

    [1] https://docs.python.org/3.5/library/stdtypes.html#str.splitlines

    Args:
        socket: a socket object.
        text: string containing a line of text for transmission.
    """
    text.replace('\0', '\n')
    lines = text.splitlines()
    first_line = '' if len(lines) == 0 else lines[0]
    # TODO Is there a better way of handling bad input than 'replace'?
    data = first_line.encode('utf-8', errors='replace') + b'\n' + (b'\0' if pad_zeros else b'')
    for offset in range(0, len(data), PACKET_SIZE):
        bytes_remaining = len(data) - offset
        if bytes_remaining < PACKET_SIZE:
            padding_length = PACKET_SIZE - bytes_remaining
            packet = data[offset:] + (b'\0' * padding_length if pad_zeros else b'')
        else:
            packet = data[offset:offset+PACKET_SIZE]
        socket.sendall(packet)


def receive_one_line(socket):
    """Receives a line of text from the given socket.

    This function will (attempt to) receive a single line of text. If data is
    currently unavailable then it will block until data becomes available or
    the sender has closed the connection (in which case it will return an
    empty string).

    The string should not contain any newline characters, but if it does then
    only the first line will be returned.

    Args:
        socket: a socket object.

    Returns:
        A string representing a single line with a terminating newline or
        None if the connection has been closed.
    """
    data = b''
    while True:
        packet = socket.recv(PACKET_SIZE)
        if not packet:  # Connection has been closed.
            return None
        data += packet
        if b'\0' in packet:
            break
    # TODO Is there a better way of handling bad input than 'replace'?
    text = data.decode('utf-8', errors='replace').strip('\0')
    lines = text.split('\n')
    return lines[0] + '\n'


def receive_lines(socket):
    try:
        data = socket.recv(PACKET_SIZE)
    except BlockingIOError:
        return []
    if data is None:  # Connection has been closed.
        return None
    # TODO Is there a better way of handling bad input than 'replace'?
    text = data.decode('utf-8', errors='replace').strip('\0')
    lines = text.split('\n')
    if len(lines)==1 and not lines[0]:
        return None
    return lines

import socket

class Connection:
    '''it wraps conn object'''
    PACKET_SIZE = 32000*5*60 # 5 minutes # was: 65536

    def __init__(self, conn):
        self.conn = conn
        self.last_line = ""

        self.conn.setblocking(True)

    def send(self, line):
        '''it doesn't send the same line twice, because it was problematic in online-text-flow-events'''
        if line == self.last_line:
            return
        send_one_line(self.conn, line)
        self.last_line = line

    def receive_lines(self):
        in_line = receive_lines(self.conn)
        return in_line

    def non_blocking_receive_audio(self):
        try:
            r = self.conn.recv(self.PACKET_SIZE)
            return r
        except ConnectionResetError:
            return None

import io
import soundfile
import librosa

# wraps socket and ASR object, and serves one client connection.
# next client should be served by a new instance of this object
class ServerProcessor:

    def __init__(self, c, online_asr_proc, min_chunk):
        self.connection = c
        self.online_asr_proc = online_asr_proc
        self.min_chunk = min_chunk

        self.last_end = None

        self.is_first = True

    def receive_audio_chunk(self):
        # receive all audio that is available by this time
        # blocks operation if less than self.min_chunk seconds is available
        # unblocks if connection is closed or a chunk is available
        out = []
        minlimit = self.min_chunk*SAMPLING_RATE
        while sum(len(x) for x in out) < minlimit:
            raw_bytes = self.connection.non_blocking_receive_audio()
            if not raw_bytes:
                break
            #            #print("received audio:",len(raw_bytes), "bytes", raw_bytes[:10])
            sf = soundfile.SoundFile(io.BytesIO(raw_bytes), channels=1,endian="LITTLE",samplerate=SAMPLING_RATE, subtype="PCM_16",format="RAW")
            audio, _ = librosa.load(sf,sr=SAMPLING_RATE,dtype=np.float32)
            out.append(audio)
        if not out:
            return None
        conc = np.concatenate(out)
        if self.is_first and len(conc) < minlimit:
            return None
        self.is_first = False
        return np.concatenate(out)

    def format_output_transcript(self,o):
        # output format in stdout is like:
        # 0 1720 Takhle to je
        # - the first two words are:
        #    - beg and end timestamp of the text segment, as estimated by Whisper model. The timestamps are not accurate, but they're useful anyway
        # - the next words: segment transcript

        # This function differs from whisper_online.output_transcript in the following:
        # succeeding [beg,end] intervals are not overlapping because ELITR protocol (implemented in online-text-flow events) requires it.
        # Therefore, beg, is max of previous end and current beg outputed by Whisper.
        # Usually it differs negligibly, by appx 20 ms.

        if o[0] is not None:
            beg, end = o[0]*1000,o[1]*1000
            if self.last_end is not None:
                beg = max(beg, self.last_end)

            self.last_end = end
            print("%1.0f %1.0f %s" % (beg,end,o[2]),flush=True,file=sys.stderr)
            return "%1.0f %1.0f %s" % (beg,end,o[2])
        else:
            logger.debug("No text in this segment")
            return None

    def send_result(self, o):
        msg = self.format_output_transcript(o)
        if msg is not None:
            self.connection.send(msg)

    def process(self):
        # handle one client connection
        self.online_asr_proc.init()
        consecutive_nonvoice = 0
        consecutive_nonvoice_thresh = 5
        result = ""
        while consecutive_nonvoice < consecutive_nonvoice_thresh:
            a = self.receive_audio_chunk()
            if a is None:
                break
            self.online_asr_proc.insert_audio_chunk(a)
            o = online.process_iter()
            print("process iter output", o, file=sys.stderr)
            if o[3] == 'nonvoice' or o[3] == None:
                consecutive_nonvoice += 1
            else:
                consecutive_nonvoice = 0
                consecutive_nonvoice_thresh = 2
            if o[2] != '':
                result += o[2]
        o = online.finish()  # this should be working
        print("finish output (discarded):", o, file=sys.stderr)
        print("process_iter concat (used):", result, file=sys.stderr)
        return result

import subprocess
import argparse
import re

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    #parser.add_argument("--ww", action=argparse.BooleanOptionalAction)
    parser.add_argument("--whisper_model", type=str, default='small.en')
    parser.add_argument("--ww_model_path", type=str, default='/home/eric/Downloads/hey_ava_model_float32.tflite') #ready_modelv2_5_float32
    parser.add_argument("--ww_detection_thresh", type=float, default=0.8)
    args = parser.parse_args()
    min_chunk_size = 1.0 #0.5

    ww_model = Model(wakeword_models=[args.ww_model_path])#, enable_speex_noise_suppression=True,vad_threshold=0.1)

    asr = FasterWhisperASR("en",args.whisper_model)
    online = VACOnlineASRProcessor(min_chunk_size, asr)

    openwakeword.utils.download_models()
    match = re.match(".*/(.*).tflite",args.ww_model_path)
    if (match):
        model_name = match.group(1)
    else:
        match = re.match(".*/(.*).onnx",args.ww_model_path)
        if (match):
            model_name = match.group(1)
        else:
            print("ww_model_path not formatted correctly", file=sys.stderr)
            model_name = args.ww_model_path

    chunk = 4096
    channels = 1
    p = pyaudio.PyAudio()
    while True:
        format = pyaudio.paInt16
        try:
            mic_stream = p.open(
                format=format,
                channels=channels,
                rate=SAMPLING_RATE,
                input=True,
                frames_per_buffer=chunk,
                #input_device_index=4
            )
        except OSError as error:
            print(f"[WARN]: Unable to access microphone. {error}", file=sys.stderr)
            exit()
        output = ""
        ww_detected = False
        while not ww_detected:
            audio = np.frombuffer(mic_stream.read(chunk), dtype=np.int16)
            prediction = ww_model.predict(audio)
            print(prediction, file=sys.stderr)
            sys.stderr.flush()
            if prediction[model_name] > args.ww_detection_thresh:
                ww_detected = True

        mic_stream.stop_stream()
        mic_stream.close()
        ww_model.reset()
        #format = pyaudio.paFloat32
        print("ww detected")
        sys.stdout.flush()

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(("localhost", 43007))
                s.listen(1)
                print('Listening on'+str(("localhost", 43007)),file=sys.stderr)
                record_sp = subprocess.Popen(["arecord", "-f", "S16_LE", "-c1", "-r", "16000", "-t", "raw", "-D", "default"], stdout=subprocess.PIPE)#, "|", "nc", "localhost", "43007"])
                subprocess.Popen(["nc","localhost","43007"], stdin=record_sp.stdout)
                conn, addr = s.accept()
                print('Connected to client on {}'.format(addr),file=sys.stderr)
                connection = Connection(conn)
                proc = ServerProcessor(connection, online, min_chunk_size)
                output = proc.process()
                conn.close()
                print('Connection to client closed',file=sys.stderr)
                record_sp.terminate()
        #print('Connection closed, terminating.')
        if output == "":
            print("<unk>")
        else:
            print(output)
        sys.stdout.flush()

    ##TODO: Could not get working using mic input through pyaudio. Using the above client to run and feed in 'arecord' output
    ##      Revisit - whisper_streaming online object was expected byte string array 16000 sample_rate 16 bit little endian
    # try:
    #     mic_stream = p.open(
    #         format=format,
    #         channels=channels,
    #         rate=SAMPLING_RATE,
    #         input=True,
    #         frames_per_buffer=chunk,
    #         #input_device_index=4
    #     )
    # except OSError as error:
    #     #print(f"[WARN]: Unable to access microphone. {error}")
    #     exit()

    ##VAC
    #while input("x:"):
    #    consecutive_nonvoice = 0
    #    while consecutive_nonvoice < 10:   # processing loop:
    #        audio = np.frombuffer(mic_stream.read(chunk), dtype=np.float32)
    #        online.insert_audio_chunk(audio)
    #        o = online.process_iter()
    #        #print(o) # do something with current partial output at the end of this audio processing
    #        if o[3] == 'nonvoice' or o[3] == None:
    #            consecutive_nonvoice += 1
    #        #if o[2] != '':
    #        else:
    #            consecutive_nonvoice = 0
    #
    #    o = online.finish()
    #    #print(o)  # do something with the last output
    #    online.init()  # refresh if you're going to re-use the object for the next audio
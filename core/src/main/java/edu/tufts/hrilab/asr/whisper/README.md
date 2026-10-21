# WhisperComponent

Requires [whisper.cpp](https://github.com/ggerganov/whisper.cpp).

Follow the instructions to build and install.
On Linux the libwhisper so/dylib should be installed at `/usr/local/lib/libwhisper.so`.
If so, you do not need to add the `-lib` argument to your config.

Models are downloaded from [Huggingface](https://huggingface.co/ggerganov/whisper.cpp) automatically and cached in `$HOME/.diarc/whisper`.


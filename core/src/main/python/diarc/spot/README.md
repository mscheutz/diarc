## Project Description

The main use of this project is to provide a DIARC interface with the Spot API.

## Files

- `spot_trade_wrapper.py` provides basically everything you'd need to control the spot from DIARC. Much of this is not implemented, but the interfaces are all there.

## Requirements

You will need the [spot wrapper](https://github.com/bdaiinstitute/spot_wrapper). Clone it anywhere on your machine (we'll call this <spot-wrapper-dir>)

Create a venv virtual environment:
* `python3 -m venv <spot-diarc-venv>`
* `source <spot-diarc-venv>`

Add pytrade diarc to venv:
* `pip install -e <diarc-dir>/core/src/main/python/`

And add dependencies:
* `pip install -e <spot-wrapper-dir>`
* `pip install -r <spot-wrapper-dir>/requirements.txt`

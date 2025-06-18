## Project Description

The main use of this project is to provide a DIARC interface with the Spot API.

## Files

- `spot_trade_wrapper.py` provides basically everything you'd need to control the spot from DIARC. Much of this is not implemented, but the interfaces are all there.

## Requirements

Your venv should be created with python 3.10. Ideally this would be enforced with setup.py or a pyproject.toml, but we don't have that yet.

Other than that, you need what's in `requirements.txt`. There's probably a lot in here that's not entirely necessary, it may be worth recreating it on a fresh machine to see what's actually needed.

You also will need the [spot wrapper](https://github.com/bdaiinstitute/spot_wrapper). Clone it where you'd like (I put it in this `spot` package) and install it with:

`pip install -e .`

while in the package. If you don't install it in the `spot` package, make sure to update your `PYTHONPATH` env var.
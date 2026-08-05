## Project Description

The main use of this project is to provide real-robot actions for integration with a sim-to-real RL project. "Hanoi" is our first example.

## Files

- `hanoi_action.py` provides high level actions for use with a policy
- `kortex_controller` interfaces with the Kinova Kortex API directly. This was done because our usual moveit interface was too slow for this use case. Could be replaced with ROS, probably (but not moveit)
- `utilities.py` is from the Kinova API examples
- `DIARC_env.py` was our first attempt at this integration. It provides gymnasium environment, and is a more generic way of providing actions/observations, but was too complicated for this initial test.

## Requirements

Your venv should be created with python 3.8. Ideally this would be enforced with setup.py or a pyproject.toml, but we don't have that yet.

Other than that, you need what's in `requirements.txt` and also the [kinova api .whl](https://artifactory.kinovaapps.com/ui/native/generic-public/kortex/API/2.6.0/kortex_api-2.6.0.post3-py3-none-any.whl).

When downloaded, you can install it in your venv with:

`pip install ~/Downloads/kortex_api-2.6.0.post3-py3-none-any.whl`

Or wherever your path to the whl is.

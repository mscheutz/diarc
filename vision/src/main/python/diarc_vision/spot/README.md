This is the python package required to use the Spot hand camera. This requires 
cloning the spot_wrapper repo and setting up a python virtual environment.

1. Clone spot_wrapper repo:

   `git clone git@github.com:bdaiinstitute/spot_wrapper.git`

    We'll call your local spot_wrapper directory [SPOT_WRAPPER] in the following instructions.


2. Setup python virtual environment. These instructions use venv, but you can use any virtual
environment you want.

    * `python3 -m venv [path_to_your_new_env]`
    * `source [path_to_your_new_env]/bin/activate`
    * `pip3 install -e [SPOT_WRAPPER]/spot_wrapper/`
    * `pip3 install -r [SPOT_WRAPPER]/spot_wrapper/requirements.txt`
    * `pip3 install -e [DIARC_HOME]/vision/src/main/python/`

Now, you should be ready to run the DIARC vision component which makes use of the spot_camera_capture.py
module defined here.
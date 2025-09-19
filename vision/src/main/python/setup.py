from setuptools import setup, find_packages

setup(
    name="diarc",
    version="1.0.0",
    packages=find_packages(),
    install_requires=[
        'numpy',
        'opencv-python',
        'PyYAML',
        'grpcio',
        'protobuf'
    ],
    author="Thinking Robots, Inc., Tufts University",
    author_email="info@thinkingrobots.ai",
    description="Python package for DIARC vision component's python modules.",
    long_description="Python package that contains python modules used by the c++ code in the vision component"
                     "to communicate with various 3rd party python modules.",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.8'
)
from setuptools import setup, find_packages

setup(
    name="diarc",
    version="1.0.0",
    packages=find_packages(),
    install_requires=[
        'JPype1==1.5.0',
        'numpy',
        'grpcio',
        'protobuf'
    ],
    author="Thinking Robots, Inc., Tufts University",
    author_email="info@thinkingrobots.ai",
    description="Python package for DIARC components and wrappers",
    long_description="Python implementation of DIARC components including TRADE integration and wrappers for Flex and Spot robots",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.8'
)
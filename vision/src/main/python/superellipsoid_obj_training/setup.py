from setuptools import setup, find_packages

package_name = 'superellipsoid_obj_training'

setup(
    name=package_name,
    version='0.1.0',
    description='Training utilities for Supereggs encoder',
    packages=find_packages(),
    install_requires=[
        'torch',
        'numpy',
        'matplotlib',
        'tensorboard',
    ],
    include_package_data=True,
)

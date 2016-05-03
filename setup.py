#!/usr/bin/env python

from setuptools import setup

setup(name='roytherobot-py',
    version='0.1',
    description='Module to control Roy the Robot',
    url='https://github.com/tuslisoftware/roytherobot-py',
    author='Tusli Software LLC',
    author_email='info@tuslisoftware.com',
    license='MIT',
    packages=['roytherobot', 'roytherobot.demo'],
    install_requires=[
        'pyserial',
        ],
    zip_safe=False,
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Operating System :: MacOS',
        'Operating System :: Microsoft :: Windows :: Windows 7',
        'Operating System :: POSIX :: Linux',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 3',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        ],
    )

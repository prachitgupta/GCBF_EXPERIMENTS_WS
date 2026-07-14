#!/usr/bin/env python

import os
from glob import glob

from setuptools import setup, find_packages


setup(
    name="gcbfplus",
    version="0.0.0",
    description='Jax Official Implementation of CoRL Paper: : S Zhang, O So, K Garg, C Fan: '
                '"GCBF+: A Neural Graph Control Barrier Function Framework for Distributed Safe Multi-Agent Control"',
    author="Songyuan Zhang",
    author_email="szhang21@mit.edu",
    url="https://github.com/MIT-REALM/gcbfplus",
    install_requires=[],
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/gcbfplus"]),
        ("share/gcbfplus", ["package.xml"]),
        (os.path.join("share", "gcbfplus", "launch"), glob("launch/*.py")),
    ],
    entry_points={
        "console_scripts": [
            "gcbf_state_bridge = gcbfplus.gcbf_state_bridge:main",
            "gcbf_actor = gcbfplus.gcbf_actor:main",
            "gcbf_monitor = gcbfplus.gcbf_monitor:main",
        ],
    },
)

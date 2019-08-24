from setuptools import setup, find_packages

setup(
    name='RPi_NRF24',
    version='0.1',

    packages=find_packages(),
    install_requires=[
        'spidev'
    ],

    author='Michal Krawczyk',
    description='Python Library for NRF24L01+',
    long_description=open('README.md').read(),
)
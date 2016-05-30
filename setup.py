from setuptools import setup

with open('requirements.txt') as f:
    required = f.read().splitlines()

setup(
    name='pymbserver',
    description='A modbus TCP server',
    long_description='',
    author='Loic Lefebvre',
    author_email='loic.celine@free.fr',
    license='MIT',
    url='https://github.com/sourceperl/pymbserver',
    platforms='any',
    install_requires=required,
    scripts=[
        'scripts/pymbserver'
    ]
)

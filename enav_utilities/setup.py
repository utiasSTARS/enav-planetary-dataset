from setuptools import setup, find_packages

setup(
    name="enav_utilities",
    version="0.1.0",
    description="Data fetching for the The Canadian planetary emulation terrain energy-aware rover navigation dataset (ENAV)",

    author="Olivier Lamarre",
    author_email="olivier.lamarre@robotics.utias.utoronto.ca",

    url="https://starslab.ca/enav-planetary-dataset",
    license="MIT",

    packages=find_packages(where="src"),
    package_dir={"": "src"},

    install_requires=[
        "affine==1.3.*",
        "click==7.1.*",
        "matplotlib==1.5.*",
        "numpy==1.11.*",
        "pathlib==1.0.*",
        "pyparsing==2.0.*",
        "rasterio==0.36.*",
        "utm==0.7.*"
    ],
)
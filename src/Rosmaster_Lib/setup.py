from setuptools import setup, find_packages

setup(
    name="Rosmaster_Lib",
    version="3.3.9",
    description="Yahboom Rosmaster Robot Control Library for Python (ROS2 compatible)",
    long_description=(
        open("README.md").read() if __import__("os").path.exists("README.md") else ""
    ),
    long_description_content_type="text/markdown",
    author="Yahboom Technology",
    url="https://www.yahboom.net/",
    packages=find_packages(),
    py_modules=["Rosmaster_Lib"],
    package_dir={"": "."},
    include_package_data=True,
    install_requires=[
        "pyserial",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
    ],
    python_requires=">=3.6",
)

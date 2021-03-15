import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="open3d_ros_helper",
    version="0.2.0.3",
    author="Seunghyeok Back",
    author_email="shback@gist.ac.kr",
    description="A helper tool for jointly using open3d and ROS",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/SeungBack/open3d-ros-helper",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 2.7",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)

# atlas_rtls

## Overview

This is the ATLAS Real-Time Localization System (RTLS). ATLAS is based on ultra-wideband using the time-difference of arrival (TDOA) topology in which the mobile nodes transmit periodically and the infrastructure-based nodes are mostly passive.

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.2579935.svg)](https://doi.org/10.5281/zenodo.2579935)

**Keywords:** 

### License

The source code of the localization is released under [BSD 3-Clause license](LICENSE).

**Authors:** Janis Tiemann, Yehya Elmasry

**Maintainer:** Janis Tiemann, janis.tiemann@tu-dortmund.de

**Affiliation:** Communication Networks Institute (CNI), TU Dortmund University

### Publications

If you use this work in an academic context, please cite the following publication(s):

* J. Tiemann, Y. Elmasry, L. Koring, C. Wietfeld, **ATLAS FaST: Fast and Simple Scheduled TDOA for Reliable Ultra-Wideband Localization**, In IEEE International Conference on Robotics and Automation (ICRA), Montréal, Canada, May 2019. (accepted for presentation). 

        @InProceedings{Tiemann2019atlasfast,
                Author = {Janis Tiemann and Yehya Elmasry and Lucas Koring and Christian Wietfeld},
                Title = {ATLAS FaST: Fast and Simple Scheduled TDOA for Reliable Ultra-Wideband Localization},
                Booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
                Year = {2019},
                Publishingstatus = {accepted for presentation},
                Address = {Montréal, Canada},
                Month = {May}
        }

The atlas_rtls package has been tested under [ROS] Kinetic and Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)

		sudo apt-get install libeigen3-dev


#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/tudo-cni-atlas/atlas_rtls.git
	cd ../
	catkin_make -j1


## Usage

Due to the numerous configuration files that are required to run the ATLAS RTLS, a seperate deployment-specific config repository is needed. An example is provided here:
[Example config and raw data](https://github.com/tudo-cni-atlas/atlas_config_example/)

The firmware for the nodes will be provided upon completion of academic review.


## Nodes

### atlas_node_bridge

Connects an anchor node through the serial protocol interface.

### atlas_core

Collects the TOA messages from multiple node bridges and assembles samples.

### atlas_loc

Calculates position results from the sample messages of the atlas core.

### atlas_fast

Provides scheduling for ATLAS FaST enabled mobile tags.

### atlas_viz

Enables visualization of the anchor nodes within [rviz].


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/tudo-cni-atlas/atlas_rtls/issues).

[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org

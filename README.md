# Self-Adaptive Body Sensor Network (SA-BSN)

![GitHub](https://img.shields.io/github/license/lesunb/bsn) ![GitHub release (latest by date)](https://img.shields.io/github/v/release/lesunb/bsn) [![DOI](https://zenodo.org/badge/233956479.svg)](https://zenodo.org/badge/latestdoi/233956479)

The Self-Adaptive Body Sensor Network (SA-BSN) features an exempalr of self-adaptive system [[1]](https://doi.org/10.1109/SEAMS51251.2021.00037) designed for experimentation on solutions for adaptation in the domain of Self-Adaptive Software Systems. Body Sensor Networks (BSNs) are networks of wearable and implantable sensors that collect physiological data (e.g., heart beat rate, blood oxigenation) from the human body. These networks are often considered safety-critical, as they enable real-time monitoring of vital signs and other health-related parameters. In addition, they interface a body of knowledge of ever evolving diseases and health conditions with a the vastness of human individuality. No solution to indentification of health conditions is comprehensive enough to tackle the current nor future diseases that may pose a threat to human condition. The self-adaptive body sensor network paves the way to such ambitious goal.

The SA-BSN provides a platform for researchers and developers to explore and evaluate adaptive solutions in the Self-Adaptive Software Systems domain. It has been utilized in various experiments and studies, as referenced in the following publications: [[2]](https://doi.org/10.1145/3194133.3194147)[[3]](https://doi.org/10.1109/SEAMS.2019.00020)[[4]](https://doi.org/10.1145/3387939.3391595).

## Prerequisites

[Docker](https://docs.docker.com/get-docker/)

[Visual Studio Code](https://code.visualstudio.com/)

[DevContainers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension for vscode.

It is highly recommended to run on Linux!!

## Getting Started

1. Clone this repository to your machine or create your own [fork](https://docs.github.com/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo) if you want to commit your own changes.

```
git clone https://github.com/lesunb/bsn_rome_24.git
```

2. With docker and Visual Studio Code properly installed, open this repository root in vscode.

3. You shall see a prompt requesting to open the repo inside of a container, accept it; (Reopen in container).

<img src="https://user-images.githubusercontent.com/12820045/200204482-808e8885-e511-44a5-9db4-778ed161913b.png" width="500">

4. Once you have accepted it, the window will reload and build the development image and SA-BSN (this should take some minutes).

### Build the SA-BSN

To compile the SA-BSN

```
catkin_make
```

### Execute the SA-BSN

The SA-BSN's execution relies on a single command, but first you need to ensure roscore to be running.

```
roscore
```

Then, in another terminal.

```
mon launch bsn.launch
```

Finally you need to execute the script that input the real data to SA-BSN

```
cd RoME_execution
python3 script.py
```

## Common Mistakes

### Different OS version vscode

If you deal with a message like this, just click on allow, it shouldn't be a problem

<img src="https://lh3.googleusercontent.com/drive-viewer/AEYmBYStY_oHKTlc4JdgloXj1mHMQubB9cUij83UODQG4_xpA0MiWh8PYtGHUR4keMmkfCHZFcqHPdy_o0j8VZqJKYQ-hJ9-aQ=w1920-h945" width="500">

### Windows CRLF error

If you are running on windows you might find this error with some files. 

<img src="https://lh3.googleusercontent.com/drive-viewer/AEYmBYRsWxJ3CH5owLiOcqB_K9GBiXisjUlTKyULKqxFrUtGwCaaQYggBw1MjhGac9AOjHubb2uOiBcld_um5NknMQCZ4yQmLg=w1920-h945" width="700">

That happens because windows has a different [line ending file](https://www.cs.toronto.edu/~krueger/csc209h/tut/line-endings.html) format and it copies it inside linux container.

To solve this you can explicitate the format in vscode.

Go to these two files, postCreate.sh and additional_bashrc.sh: 

<img src="https://lh3.googleusercontent.com/drive-viewer/AEYmBYSfp7GCLOaQ60Yz0z7LPAm39wQb5M067-qdDLYuNtNVIwD76qbkdqFYyZBpMlJH0CV5LH7_-rmgs73s7Sa8_uGBXWR4VQ=w1920-h945" width="500">

In the bottom right corner click on CRLF, change it to LF and save the files.

<img src="https://lh3.googleusercontent.com/drive-viewer/AEYmBYTjgEmvMQyu8cRgub-R6g6nQxg84-NetBnyHRGGu6qzGe_86N17-SfJfUIrw1TXg8PBJw7HOVh1CjNsnZWXjhQLsik_1w=w1920-h945" width="500">

Then rebuild the container.

### In case of error due to the ROS path

You might want to source the setup.bash inside the catkin workspace:
```
source devel/setup.bash
```

Main Authors

* [**Ricardo Caldas**](https://rdinizcal.github.io/)
* [**Gabriel Levi**](https://github.com/gabrielevi10)
* [**Léo Moraes**](https://github.com/leooleo)  
* [**Eric B. Gil**](https://github.com/ericbg27)
* [**Samuel Couto**](https://github.com/SCouto97)

Adviser: [**Genaína N. Rodrigues**](https://genaina.github.io/)

## Acknowledgment

This study was financed in part by CAPES-Brasil – Finance Code 001, through CAPES scholarship, by CNpq under grant number 306017/2018-0, by University of Brasilia under Call DPI/DPG 03/2020, by FAPDF Call 03/2018 by the Wallenberg Al, Autonomous Systems and Software Program (WASP) funded by the Knut and Alice Wallenberg Foundation

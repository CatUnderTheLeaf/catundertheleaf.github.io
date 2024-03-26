---
layout: post
title: "How to make an AWS DeepRacer car simulation and control it with ROS1"
categories: aws
---

## Introduction

A lot of people have heard of AWS DeepRacer, but if not, here is short information from their site:
> [AWS DeepRacer](https://aws.amazon.com/deepracer/?nc=sn&loc=0) gives you an interesting and fun way to get started with reinforcement learning (RL). RL is an advanced machine learning (ML) technique that takes a very different approach to training models than other machine learning methods. Its super power is that it learns very complex behaviors without requiring any labeled training data, and can make short term decisions while optimizing for a longer term goal.

> Developers of all skill levels can get hands on with machine learning through a cloud-based 3D racing simulator, fully autonomous 1/18th scale race car driven by reinforcement learning, and global racing league.

But... only 10 hours free for 30 days with the AWS Free Tier... Maybe there is something that can be launched locally?

> [deepracer-for-cloud](https://aws-deepracer-community.github.io/deepracer-for-cloud/) provides a quick and easy way to get up and running with a DeepRacer training environment in AWS or Azure, using either the Azure N-Series Virtual Machines or AWS EC2 Accelerated Computing instances, or locally on your own desktop or server.

That is a good option: train your car locally for any number of hours and then just upload the model. What else can be done?

But what if you want to gain more control of your car? Or you want to somehow incorporate `Behavioral cloning` and train your car with manual steering and velocity inputs in hope that it will have a better performance than with random inputs of RL? Or you just want to simply simulate a car, race track and play with it?

## Software

It turned out there are 3 software configurations:

- __DeepRacer v1__: Ubuntu 16.04, ROS1 Kinetic(EOL) and Python 2(EOL)
- __DeepRacer v2__: Ubuntu 20.04, ROS2 Foxy Fitzroy(EOL) and Python 3.8
- __deepracer-for-cloud__: Ubuntu 20.04, ROS1 Noetic and Python 3.8

As can be seen the last configuration is the right option to simulate and train a car locally. Noetic is the last version of ROS1 and Python 3.8 will meet its EOL in October 2024.

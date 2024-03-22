---
layout: post
title: "How to make an AWS DeepRace car simulation and control it with ROS1"
categories: aws
---

## Introduction

[AWS DeepRacer](https://aws.amazon.com/deepracer/?nc=sn&loc=0) gives you an interesting and fun way to get started with reinforcement learning (RL). RL is an advanced machine learning (ML) technique that takes a very different approach to training models than other machine learning methods. Its super power is that it learns very complex behaviors without requiring any labeled training data, and can make short term decisions while optimizing for a longer term goal.

Developers of all skill levels can get hands on with machine learning through a cloud-based 3D racing simulator, fully autonomous 1/18th scale race car driven by reinforcement learning, and global racing league.

But... only 10 hours free for 30 days with the AWS Free Tier...

[deepracer-for-cloud](https://aws-deepracer-community.github.io/deepracer-for-cloud/) helps as you can train locally and then only upload trained model.

But what if you want to gain more control of your car? Or you want to somehow help training with additional manual steering and velocity inputs?

## Software
